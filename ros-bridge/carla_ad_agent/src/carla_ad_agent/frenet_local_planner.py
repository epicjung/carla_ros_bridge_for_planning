#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

from collections import deque
import math
import time
import rospy
import numpy as np
import carla
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from carla_waypoint_types.srv import GetWaypoint
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from vehicle_pid_controller import VehiclePIDController
from misc import distance_vehicle
from misc import point_distance
from spline import Spline2D
from spline import Spline
import matplotlib.pyplot as plt
import frenet_optimal_trajectory as fot
import carla_ros_bridge.transforms as trans
import copy 
import csv 
from itertools import izip 

MAX_DECEL = 3.0
MAX_ACCEL = 2.0
MIN_LENGTH = 8.0
MAX_LENGTH = 30.0
PATH_SIZE = 15
PATH_OFFSET = 0.4
W_SMC = 0.05 # weight for smoothness cost
W_GC = 0.1 # weight for global following cost
W_SC = 0.4 # weight for static obstacle cost
W_DC = 0.4 # weight for dynamic obstacle cost
SIGMA = 0.9
X_ENLARGE = 2.0
Y_ENLARGE = 1.0

class FrenetCandidate:
    def __init__(self):
        self.sp = None
        self.x = []
        self.y = []
        self.max_curv = 0.0
        self.smc = 0.0 # smoothness cost
        self.gc = 0.0 # global-following cost
        self.sc = 0.0 # static obstacle cost
        self.dc = 0.0 # dynamic obstacle cost
        self.collision_len = -1.0 # collision arc length
        self.collision_type = -1 # 0: STATIC, 1: DYNAMIC
        self.ttc = 100.0

class Obstacle:
    def __init__(self):
        self.s = 0.0
        self.p = 0.0
        self.idx = 0
        self.type = 0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.ros_transform = None
        self.carla_transform = None
        self.bbox = None
        

class FrenetLocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    # MIN_DISTANCE_PERCENTAGE = 0.6 # Town 1
    MIN_DISTANCE_PERCENTAGE = 0.9 # Town 4
    def __init__(self, role_name, opt_dict=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param opt_dict: dictionary of arguments with the following semantics:

            target_speed -- desired cruise speed in Km/h

            sampling_radius -- search radius for next waypoints in seconds: e.g. 0.5 seconds ahead

            lateral_control_dict -- dictionary of arguments to setup the lateral PID controller
                                    {'K_P':, 'K_D':, 'K_I'}

            longitudinal_control_dict -- dictionary of arguments to setup the longitudinal
                                         PID controller
                                         {'K_P':, 'K_D':, 'K_I'}
        """
        self._current_waypoint = None
        self.target_waypoint = None
        self._vehicle_controller = None
        self._waypoints_queue = deque(maxlen=20000)
        self._global_waypoints = []
        self._buffer_size = 20
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self._vehicle_yaw = None
        self._current_speed = None
        self._current_speed_2d = None
        self._current_pose = None
        self._current_accel = None
        self._is_planned = False
        self._is_updated = False
   
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()
        self.map = self.world.get_map()

        # obstacles
        self.obs = {}
        self._collision_check = [0.0 for _ in range(2*PATH_SIZE+1)]

        # global path related
        self.tx = None
        self.ty = None
        self.tyaw = None
        self.tc = None
        self.csp = None 

        # initial state
        self.c_speed = 0.0
        self.c_d = 0.0
        self.c_dd = 0.0
        self.c_ddd = 0.0
        self.s = 0.0
        self.c_i = 0

        # local path for visualization
        self._local_path = Marker()
        self._local_path.header.frame_id = "map"
        self._local_path.id = 2000
        self._local_path.type = 8 # points 
        self._local_path.action = 0
        self._local_path.scale.x = 0.3
        self._local_path.scale.y = 0.2
        self._local_path.color.r = 1.0
        self._local_path.color.a = 0.5

        self._local_buffer = Marker()
        self._local_buffer.header.frame_id = "map"
        self._local_buffer.id = 3000
        self._local_buffer.type = 8 # points 
        self._local_buffer.action = 0
        self._local_buffer.scale.x = 0.3
        self._local_buffer.scale.y = 0.2
        self._local_buffer.color.b = 1.0
        self._local_buffer.color.a = 1.0

        # footage
        self._footage = Marker()
        self._footage.header.frame_id = "map"
        self._footage.id = 4000
        self._footage.type = 4
        self._footage.action = 0
        self._footage.scale.x = 0.5
        self._footage.color.r = 1.0
        self._footage.color.g = 1.0
        self._footage.color.a = 1.0

        self._target_point_publisher = rospy.Publisher(
            "/next_target", PointStamped, queue_size=1)
        rospy.wait_for_service('/carla_waypoint_publisher/{}/get_waypoint'.format(role_name))

        # visualization
        self._local_waypoints_publisher = rospy.Publisher(
            "/carla/{}/local_waypoints".format(role_name), Marker, queue_size=10)
        self._path_candidates_publisher = rospy.Publisher(
            "/carla/{}/path_candidates".format(role_name), MarkerArray, queue_size=1)
        self._lookahead_publisher = rospy.Publisher(
            "/carla/{}/look_ahead".format(role_name), Marker, queue_size=1)
        self._footage_waypoints_publisher = rospy.Publisher(
            "/carla/{}/footage".format(role_name), Marker, queue_size=10)
        self._obs_velocity_publisher = rospy.Publisher(
            "/carla/obs/velocity", Marker, queue_size=10)
        

        # current status of the vehicle
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_status".format(role_name),
            CarlaEgoVehicleStatus, self.vehicle_status_updated)

        self._local_buffer_publisher = rospy.Publisher(
            "/carla/{}/local_buffer".format(role_name), Marker, queue_size=10)
        
        # self._get_path = rospy.ServiceProxy(
        #     '/spline_info'
        # )

        self._get_waypoint_client = rospy.ServiceProxy(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(role_name), GetWaypoint)
        rospy.logwarn("Initialize Frenet Planner")
        # initializing controller
        self._init_controller(opt_dict)

    def get_waypoint(self, location):
        """
        Helper to get waypoint from a ros service
        """
        try:
            response = self._get_waypoint_client(location)
            return response.waypoint
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            if not rospy.is_shutdown:
                rospy.logwarn("Service call failed: {}".format(e))

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on updated vehicle status
        """
        # xy acceleration [m/s^2]
        self._current_accel = math.sqrt(vehicle_status.acceleration.linear.x ** 2 + 
                                        vehicle_status.acceleration.linear.y ** 2)

    def odometry_updated(self, odo):
        """
        Callback on new odometry
        """

        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6

        # xy speed [m/s]]
        self._current_speed_2d = math.sqrt(odo.twist.twist.linear.x ** 2 + 
                                           odo.twist.twist.linear.y ** 2)

        self._current_pose = odo.pose.pose
        quaternion = (
            odo.pose.pose.orientation.x,
            odo.pose.pose.orientation.y,
            odo.pose.pose.orientation.z,
            odo.pose.pose.orientation.w
        )
        _, _, self._vehicle_yaw = euler_from_quaternion(quaternion)
        self._is_updated = True

    def _init_controller(self, opt_dict):
        """
        Controller initialization.

        :param opt_dict: dictionary of arguments.
        :return:
        """
        # default params
        self._current_speed = 0.0  # Km/h
        self._current_pose = Pose()
        args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.01,
            'K_I': 1.4}
        args_longitudinal_dict = {
            'K_P': 0.2,
            'K_D': 0.05,
            'K_I': 0.1}

        # parameters overload
        if opt_dict:
            if 'lateral_control_dict' in opt_dict:
                args_lateral_dict = opt_dict['lateral_control_dict']
            if 'longitudinal_control_dict' in opt_dict:
                args_longitudinal_dict = opt_dict['longitudinal_control_dict']

        self._current_waypoint = self.get_waypoint(self._current_pose.position)
        self._vehicle_controller = VehiclePIDController(args_lateral=args_lateral_dict,
                                                        args_longitudinal=args_longitudinal_dict)

        self._global_plan = False

    def set_global_plan(self, current_plan):
        """
        set a global plan to follow
        """
        self._waypoints_queue.clear()
        self._waypoint_buffer.clear()
        self._global_waypoints = []
        self.tx = None
        self.ty = None
        self.tyaw = None
        self.tc = None
        self.csp = None 

        # generate center line
        x = []
        y = []
        for i in range(len(current_plan)):
            c_wp = current_plan[i].pose
            if i == 0:
                self._waypoints_queue.append(c_wp)
                self._global_waypoints.append(c_wp)
                x.append(c_wp.position.x)
                y.append(c_wp.position.y)
            else:
                p_wp = current_plan[i-1].pose
                if p_wp.position.x != c_wp.position.x and \
                    p_wp.position.y != c_wp.position.y:
                    self._waypoints_queue.append(c_wp)
                    self._global_waypoints.append(c_wp)
                    x.append(c_wp.position.x)
                    y.append(c_wp.position.y)
                    
            
        # plt.plot(x, y, 'go')

        # check validity of global path
        if self.is_valid_path(current_plan):
            self.tx, self.ty, self.tyaw, self.tc, self.csp = fot.generate_target_course(x, y)
            self._is_planned = True
            # plt.plot(self.tx, self.ty, 'rx')

        # plt.show()

    def is_valid_path(self, current_plan):
        return len(current_plan) >= 2

    def locate_vehicle(self, pos):

        min_idx = -1
        iteration = 0
        sk_last = 0.0

        # find ego_vehicle w.r.t global path
        if self._global_waypoints:
            distance = np.array([distance_vehicle(wp, pos) for wp in self._global_waypoints])
            # min_val = np.amin(distance)
            # min_idx = np.where(distance == min_val)[0][0]
            min_idx = np.where(distance == np.amin(distance))[0][0]

        # calculate arc length
        c_s = self.csp.s[min_idx]
        ss = [None for _ in range(4)]
        # ss[0] = (c_s - 7.0) if c_s - 7.0 >= 0.0 else c_s  
        ss[0] = c_s - 0.5
        ss[2] = c_s + 0.5
        ss[1] = (ss[0] + ss[2]) / 2
        c_x = pos.x
        c_y = pos.y
        
        # calculate optimal arc length and lateral offset
        while True:
            s1 = ss[0]
            s2 = ss[1]
            s3 = ss[2]
            # print("get sp: c_s: {}, ss0: {}, ss1:{}, ss2:{}").format(c_s, ss[0], ss[1], ss[2])
            ds1 = point_distance(c_x, c_y, self.csp.calc_position(s1)[0], self.csp.calc_position(s1)[1]) 
            ds2 = point_distance(c_x, c_y, self.csp.calc_position(s2)[0], self.csp.calc_position(s2)[1]) 
            ds3 = point_distance(c_x, c_y, self.csp.calc_position(s3)[0], self.csp.calc_position(s3)[1])
            # print("ds1: {}, ds2:{}, ds3:{}").format(ds1, ds2, ds3) 
            sk = 0.5 * ((s2**2 - s3**2)*ds1 + (s3**2 - s1**2)*ds2 + (s1**2 - s2**2)*ds3) \
                / ((s2-s3)*ds1 + (s3-s1)*ds2 + (s1-s2)*ds3)
            terminate_cond = 1.0 / (len(self.csp.s) * 1000.0)
            ss[3] = sk
            
            # print("get sp: iteration: {} last: {} now: {}").format(iteration, sk_last, sk)

            if iteration > 0:
                if abs(sk_last - sk) < terminate_cond:
                    lat_offset = point_distance(self.csp.calc_position(sk)[0], self.csp.calc_position(sk)[1], c_x, c_y)
                    lat_offset = self.get_spline_dir(c_x, c_y, sk) * lat_offset
                    return sk, lat_offset, min_idx
            
            sk_last = sk

            # eliminate the value which gives the largetst P(s) among s1 s2 s3 sk
            max_P = -1 * float("inf")
            max_idx = 0

            for i in range(4):
                s = ss[i]
                P = (s-s2)*(s-s3)*ds1 / ((s1-s2)*(s1-s3)) + \
                    (s-s1)*(s-s3)*ds2 / ((s2-s1)*(s2-s3)) + \
                    (s-s1)*(s-s2)*ds3 / ((s3-s1)*(s3-s2))
                if P > max_P:
                    max_P = P
                    max_idx = i

            if max_idx < 3:
                ss[max_idx] = sk
                for i in range(3):
                    for j in range(i+1, 3, 1):
                        if ss[i] == ss[j]:
                            if ss[j] < 0.5:
                                ss[j] = ss[j] + 0.0001
                            else:
                                ss[j] = ss[j] - 0.0001
            
            # return rough estimate if fails to find the optimal solution
            if iteration > 100:
                rospy.logwarn("Fail to find optimal lateral offset")
                lat_offset = point_distance(self.csp.calc_position(c_s)[0], self.csp.calc_position(c_s)[1], c_x, c_y)
                return c_s, lat_offset, min_idx

            iteration += 1

    def get_waypoint_dir(self, waypoint, pos):
        carla_position = carla.Location()
        carla_position.x = waypoint.position.x
        carla_position.y = -waypoint.position.y
        carla_position.z = waypoint.position.z
        carla_waypoint = self.map.get_waypoint(carla_position)
        next_waypoints = carla_waypoint.next(1.0)
        c_wx = waypoint.position.x
        c_wy = waypoint.position.y
        n_wx = next_waypoints[0].transform.location.x
        n_wy = -next_waypoints[0].transform.location.y
        dx = n_wx - c_wx
        dy = n_wy - c_wy 
        slope = 0.0

        if dx != 0.0:
            slope = dy / dx
        else:
            rospy.logwarn("Invalid lateral direction")
            return 1

        checker = pos.y - c_wy - slope*(pos.x - c_wx)

        # 1st and 4th quadrants
        if (dy > 0 and dx > 0) or (dy < 0 and dx > 0):
            if checker <= 0:
                return -1
            else:
                return 1
        # 2nd and 3rd quadrants
        elif (dy > 0 and dx < 0) or (dy < 0 and dx < 0):
            if checker <= 0:
                return 1
            else:
                return -1 

    def get_spline_dir(self, c_x, c_y, s):
        c_wx = self.csp.calc_position(s)[0]
        c_wy = self.csp.calc_position(s)[1]
        n_wx = self.csp.calc_position(s + 0.1)[0]
        n_wy = self.csp.calc_position(s + 0.1)[1]
        dx = n_wx - c_wx
        dy = n_wy - c_wy 
        slope = 0.0

        if dx != 0.0:
            slope = dy / dx
        else:
            rospy.logwarn("Invalid lateral direction")
            return 1

        checker = c_y - c_wy - slope*(c_x - c_wx)

        # 1st and 4th quadrants
        if (dy > 0 and dx > 0) or (dy < 0 and dx > 0):
            if checker <= 0:
                return -1
            else:
                return 1
        # 2nd and 3rd quadrants
        elif (dy > 0 and dx < 0) or (dy < 0 and dx < 0):
            if checker <= 0:
                return 1
            else:
                return -1 

    def length_to_index(self, start_idx, length):
        s_start = self.csp.s[start_idx]
        for i in range(self.c_i, len(self.csp.s), 1):
            if self.csp.s[i] > s_start + length:
                return i 
        return len(self.csp.s)-1
        
    def get_path_length(self):
        length = self._current_speed_2d / MAX_DECEL * self._current_speed_2d + MIN_LENGTH

        # path length by obstacles
        for key in self.obs:
            ob_info = self.obs[key]
            if ob_info.idx >= self.c_i: 
                arc_diff = ob_info.s - self.s
                if arc_diff < length and arc_diff <= MAX_LENGTH:
                    length = arc_diff
        
        # Clip the length
        length = max(MIN_LENGTH, min(length, MAX_LENGTH))
        return length

    def get_obstacle(self):
        start = time.time()
        actor_list = self.world.get_actors()
        # print("Actor list: {}").format(len(actor_list))
        # print("----Actor----")
        for actor in actor_list:
            if "role_name" in actor.attributes:
                if actor.attributes["role_name"] == 'autopilot':
                    carla_transform = actor.get_transform()
                    ros_transform = trans.carla_transform_to_ros_pose(carla_transform)
                    x = ros_transform.position.x
                    y = ros_transform.position.y
                    z = ros_transform.position.z 
                    # r, p, y = euler_from_quaternion(ros_transform.orientation)
                    distance = point_distance(x, y, self._current_pose.position.x, self._current_pose.position.y)
                    # print("obs distance: {}").format(distance)
                    if distance < 70.0:
                        s, p, idx = self.locate_vehicle(ros_transform.position)
                        ob = Obstacle()
                        ob.s = s
                        ob.p = p
                        ob.idx = idx
                        ob.carla_transform = carla_transform
                        ob.ros_transform = ros_transform
                        ob.vx = actor.get_velocity().x
                        ob.vy = actor.get_velocity().y
                        ob.vz = actor.get_velocity().z
                        speed = math.sqrt(ob.vx**2 + ob.vy**2 + ob.vz**2)
                        if  speed < 0.5:
                            ob.type = 0 # static
                        else:
                            ob.type = 1 # dynamic
                            self.visualize_velocity(ob.carla_transform.location, actor.get_velocity())
                        ob.bbox = actor.bounding_box # in local carla frame
                        # print("x: {}, y: {}, z:{}").format(x, y, z)
                        # print("{}: s: {}, p: {}, idx: {}").format(actor.id, s, p, idx)
                        self.obs[actor.id] = ob 
                            
        end = time.time()
        # print("Obstacle: {}").format(len(self.obs))
        # rospy.logwarn("Get obstacle time: %f", end - start)

    def get_angle_wrt_route(self):
        path_yaw = self.csp.calc_yaw(self.s) # global path yaw [deg]
        angle_diff = self._vehicle_yaw - path_yaw
        if abs(angle_diff) >= math.pi:
            sign = 1.0 if angle_diff >= 0 else -1.0
            angle_diff = angle_diff - 2 * math.pi * sign
        return angle_diff

    def generate_path(self, s_start, s_end, offset, angle_diff):
        cand_list = []
        coverage = PATH_SIZE * offset

        for i in range(2*PATH_SIZE+1):
            cand_list.append(self.generate_candidate(s_start, s_end, coverage, angle_diff))    
            coverage -= offset
        return cand_list

    def generate_candidate(self, s_start, s_end, offset, angle_diff):
        cand = FrenetCandidate()
        s = [s_start, s_end]
        p = [self.c_d, self.c_d + offset]
        sp = Spline(s, p)
        sp.set_boundary(1, math.tan(angle_diff), 1, 0.0)
        sp.set_two_points()
        cand.sp = sp
        return cand

    def test(self):
        s = []
        p = []
        s.append(0.0)
        s.append(8.8)
        p.append(0.0)
        p.append(-1.0)
        spline = Spline(s, p)
        spline.set_boundary(1, math.tan(0.1), 1, 0.0)
        spline.set_points()

        s = list(np.arange(0.0, 8.8, 0.2))
        ss = []
        pp = []
        for i_s in s:
            ss.append(i_s)
            pp.append(spline.calc(i_s))
        plt.plot(ss, pp, 'rx')
        print(ss)
        print(pp)
        plt.show()

    def convert_cartesian(self, cand_list, s_start, s_end):
        cx = self._current_pose.position.x
        cy = self._current_pose.position.y
        ds = 0.4
        for i in range(len(cand_list)):
            sp = cand_list[i].sp
            i_x = cx
            i_y = cy
            yaw = self._vehicle_yaw
            smoothness_cost = 0.0
            max_curv = 0.0
            sum_curv = 0.0
            
            s = list(np.arange(s_start, s_end, ds))
            for i_s in s:
                x0 = self.csp.sx.calc(i_s)
                y0 = self.csp.sy.calc(i_s)
                dx = self.csp.sx.calcd(i_s)
                dy = self.csp.sy.calcd(i_s)
                ddx = self.csp.sx.calcdd(i_s)
                ddy = self.csp.sy.calcdd(i_s)
                k0 = (dx*ddy-ddx*dy) / ((dx*dx+dy*dy)**1.5)
                sum_curv += k0
                p0 = sp.calc(i_s)
                dp = sp.calcd(i_s)
                ddp = sp.calcdd(i_s)
                A = math.sqrt(dp*dp + (1-p0*k0)**2)

                if (1-p0*k0) > 0:
                    B = 1
                elif (1-p0*k0) == 0:
                    B = 0
                else:
                    B = -1

                kc = B/A * (k0 + ((1-p0*k0)*ddp + k0*dp*dp) / (A**2))
                
                # insert converted points
                cand_list[i].x.append(i_x)
                cand_list[i].y.append(i_y)
                
                # propagate to the next
                i_x += A*B*math.cos(yaw)*ds
                i_y += A*B*math.sin(yaw)*ds
                yaw += A*B*kc*ds

                # find max k
                if abs(kc) > max_curv:
                    max_curv = abs(kc)

                # smoothness cost
                smoothness_cost += kc**2*ds
              
                # plt.plot(i_x, i_y, 'rx')
            cand_list[i].max_curv = max_curv
            cand_list[i].smc = smoothness_cost
        # plt.show()

    def visualize_lookahead(self, waypoint):
        marker = Marker()
        marker.id = 1000
        marker.header.frame_id = "map"
        marker.type = 1
        marker.action = 0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.pose = waypoint
        self._lookahead_publisher.publish(marker)

    def visualize_velocity(self, pos, vel):
        marker = Marker()
        marker.id = 6000
        marker.header.frame_id = "map"
        marker.type = 0
        marker.action = 0
        start = Point()
        start.x = pos.x
        start.y = -pos.y
        start.z = pos.z
        end = Point()
        end.x = start.x + vel.x
        end.y = start.y - vel.y
        end.z = start.z + vel.z
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.2
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points.append(start)
        marker.points.append(end)
        self._obs_velocity_publisher.publish(marker)
    
    def visualize_candidate(self, cand_list):
        
        candidates = MarkerArray()

        for i in range(len(cand_list)):
            path = Marker()
            path.id = i
            path.ns = 'candidates'
            path.type = 4 # LINE_STRIP
            path.action = 0
            path.scale.x = 0.05

            if self._collision_check[i]:
                path.color.r = 1.0
            else:
                path.color.g = 1.0
            path.color.a = 1.0

            for j in range(len(cand_list[i].x)):
                point = Point()
                point.x = cand_list[i].x[j]
                point.y = cand_list[i].y[j]
                path.points.append(point)
            path.header.frame_id = "map"
            candidates.markers.append(path)
        
        self._path_candidates_publisher.publish(candidates)

    def set_global_cost(self, cand_list, s_end):
        total = 0.0
        offsets = []
        for i in range(len(cand_list)):
            offset = cand_list[i].sp.calc(s_end)
            total += abs(offset)
            offsets.append(abs(offset))
        # normalize
        for i in range(len(offsets)):
            cand_list[i].gc = offsets[i] / total

    def check_lanes(self, cand_list, s_start, s_end):
        # find ego's current waypoint
        self._current_waypoint = self.get_waypoint(self._current_pose.position)
        
        # signed distance from position to closest waypoint
        distance = distance_vehicle(self._current_waypoint.pose, self._current_pose.position)
        direction = self.get_waypoint_dir(self._current_waypoint.pose, self._current_pose.position)
        distance *= direction

        # collision check
        lane_width = self._current_waypoint.lane_width
        lane_change = self._current_waypoint.lane_change
        s = list(np.arange(s_start, s_end, 0.5))

        for i in range(len(cand_list)):
            sp = cand_list[i].sp
            left_allowed = 0.0
            right_allowed = 0.0
            for i_s in s:
                offset_diff = sp.calc(i_s) - self.c_d
                if lane_change == "Right":
                    # left lane is not available
                    left_allowed = 0.5*lane_width - distance 
                    if offset_diff > left_allowed:
                        self._collision_check[i] = 1.0
                        break
                elif lane_change == "Left":
                    # right lane is not avaiable
                    right_allowed = -0.5*lane_width - distance 
                    if offset_diff < right_allowed:
                        self._collision_check[i] = 1.0
                        break
                elif lane_change == "NONE":
                    # no lanes are available
                    left_allowed = 0.5*lane_width - distance 
                    right_allowed = -0.5*lane_width - distance 
                    if offset_diff > left_allowed and offset_diff < right_allowed:
                        self._collision_check[i] = 1.0
                        break
                else:
                    # all lane is avaiable
                    continue

            # if self._collision_check[i]:
                # print("----Path {}---").format(i)
                # print("offset_diff: {}").format(offset_diff)
                # print("left allowed: {}").format(left_allowed)
                # print("right allowed: {}").format(right_allowed)

    def check_obs(self, cand_list):
        for i in range(len(cand_list)):
            static_collide = False
            dynamic_collide = False
            
            for key in self.obs:
                collision_len = 0.0
                ob = self.obs[key]
                ob.bbox.location.z = 0.0

                static_bbox = carla.BoundingBox()
                static_bbox.location.x = copy.copy(ob.bbox.location.x)
                static_bbox.location.y = copy.copy(ob.bbox.location.y)
                static_bbox.location.z = copy.copy(ob.bbox.location.z)
                static_bbox.extent.x = copy.copy(ob.bbox.extent.x)
                static_bbox.extent.y = copy.copy(ob.bbox.extent.y)
                static_bbox.extent.z = copy.copy(ob.bbox.extent.z)

                # enlarge box as a buffer
                static_bbox.extent.x += X_ENLARGE
                static_bbox.extent.y += Y_ENLARGE

                # if ob.type == 1:

                for j in range(len(cand_list[i].x)):
                    world_point = carla.Location()
                    world_point.x = cand_list[i].x[j]
                    world_point.y = -cand_list[i].y[j]
                    world_point.z = ob.carla_transform.location.z
                    
                    # calculate collision point
                    if j > 0:
                        collision_len += point_distance(cand_list[i].x[j], cand_list[i].y[j], 
                                            cand_list[i].x[j-1], cand_list[i].y[j-1])

                    # check static collision
                    if ob.type == 0:
                        if static_bbox.contains(world_point, ob.carla_transform):
                            # print("cand {}: static collision").format(i)
                            cand_list[i].collision_type = 0
                            static_collide = True
                            break

                    # check dynamic
                    else:
                        lat_diff = ob.p - self.c_d
                        if abs(lat_diff) >= self._current_waypoint.lane_width / 2.0: # located in the another lane
                            t = list(np.arange(0.0, 5.0, 0.5))
                            for i_t in t:
                                vertices = static_bbox.get_world_vertices(ob.carla_transform)

                                # move vertices 
                                for v in vertices:
                                    v.x += i_t*ob.vx
                                    v.y += i_t*ob.vy
                                    v.z += i_t*ob.vz                             

                                if self.point_inside_vertices(world_point, vertices):
                                    # print("cand {}: dynamic collision").format(i)
                                    # print("collision length: {}, ttc: {}").format(collision_len, i_t)
                                    if i_t < cand_list[i].ttc: # if colliding obstacle collides faster
                                        cand_list[i].collision_len = collision_len
                                        cand_list[i].collision_type = 1
                                        cand_list[i].ttc = i_t
                                    dynamic_collide = True
                                    break
                                else:
                                    dynamic_collide = False
                        else:
                            break
                            
                        if dynamic_collide:
                            break               

                if static_collide:
                    self._collision_check[i] = 1.0
                    # print("{}: static collision").format(i)
                    break

    def point_inside_vertices(self, point, vertices):
        vx = [v.x for v in vertices]
        vy = [v.y for v in vertices]
        vz = [v.z for v in vertices]
        return point.x >= min(vx) and point.x <= max(vx) \
                and point.y >= min(vy) and point.y <= max(vy) \
                and point.z >= min(vz) and point.z <= max(vz) 
        
    def guassian_convolution(self, cand_list):
        for i in range(len(cand_list)):
            static_cost = 0.0
            for k in range(-PATH_SIZE, PATH_SIZE + 1, 1):
                Rki = 0.0
                g = 0.0
                if k+i < 0 or k+i > 2*PATH_SIZE:
                    Rki = 0.5
                else:
                    Rki = self._collision_check[k+i]
                if Rki == 1.0:
                    g = 1.0 / math.sqrt(2*math.pi*SIGMA) * math.exp(-k**2 / (2*SIGMA**2))
                static_cost += g * Rki
            cand_list[i].sc = static_cost

    def set_obs_cost(self, cand_list, s_start, s_end):
        start = time.time()
        # print("---set_obs_cost---")
        self.check_lanes(cand_list, s_start, s_end)
        self.check_obs(cand_list)
        end = time.time()
        # rospy.logwarn("Collsion check time: %f", end - start)
        start = time.time()
        self.guassian_convolution(cand_list)
        self.set_dynamic_cost(cand_list)
        end = time.time()
        # rospy.logwarn("Set cost time: %f", end - start)

    def set_dynamic_cost(self, cand_list):
        L_CUTIN = 30.0
        L_FOLLOW = 30.0
        for i in range(len(cand_list)):
            if cand_list[i].collision_type == 1 and self._current_speed > 0.0: # Dynamic
                # print("Path candidate {}").format(i)
                s = cand_list[i].collision_len
                ob_ttc = cand_list[i].ttc
                ego_ttc = s / self._current_speed
                delta = ob_ttc - ego_ttc
                acc = 0.0

                if ob_ttc == 0.0: # collision from the beginning
                    # print("Collision from the beginning")
                    self._collision_check[i] = 1.0
                    cand_list[i].dc = float('inf') # tmp
                else:
                    if delta > 0: # Cut in
                        if s + L_CUTIN < self._current_speed * ob_ttc: # with current speed is fine
                            acc = 0.0
                        else: # require some acceleration 
                            acc = 2 * (s + L_CUTIN - self._current_speed * ob_ttc) / (ob_ttc**2)

                        if acc > MAX_ACCEL: # exceeds the maximum acceleration 
                            self._collision_check[i] = 1.0
                            cand_list[i].dc = float('inf') # tmp
                            # print("No Cut in: exceed max acc")
                            # cand_list[i].dc = abs(MAX_DECEL) * (s + L_CUTIN)
                        else: # viable
                            # print("Cut in")
                            cand_list[i].dc = abs(acc) * (s + L_CUTIN)
                    else: # Following
                        if L_FOLLOW > s:
                            L_FOLLOW = s
                        acc = 2 * (s-L_FOLLOW-self._current_speed*ob_ttc) / (ob_ttc**2)
                        if acc < -MAX_DECEL:
                            # print("No follow: exceed max decel")
                            self._collision_check[i] = 1.0
                            cand_list[i].dc = float('inf')
                        elif acc > MAX_ACCEL:
                            # print("No follow: exceed max acc")
                            self._collision_check[i] = 1.0
                            cand_list[i].dc = float('inf')
                        else:
                            # print("Follow")
                            self._collision_check[i] = 1.0 # wait 
                            cand_list[i].dc = abs(acc) * (s - L_FOLLOW)



    def find_optimal_path(self, cand_list):
        min_cost = float("inf")
        min_idx = -1
        print("---cost---")
        for i in range(len(cand_list)):
            cost = W_SMC*cand_list[i].smc + \
                    W_GC*cand_list[i].gc + \
                    W_SC*cand_list[i].sc + \
                    W_DC*cand_list[i].dc
            # print("{}: col: {}, smc: {}, gc: {}, sc: {}, dc: {}, sum:{}") \
            #     .format(i, self._collision_check[i], W_SMC*cand_list[i].smc, W_GC*cand_list[i].gc, \
            #     W_SC*cand_list[i].sc, W_SC*cand_list[i].dc, cost)

            if cost < min_cost and not self._collision_check[i]:
                min_cost = cost
                min_idx = i
        
        # TO-DO: not found case
        return min_idx

    def run_step(self, target_speed):
        """
        Execute one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """

        self._collision_check = [0.0 for _ in range(2*PATH_SIZE+1)]

        if not self._is_updated or not self._is_planned:
            return

        # Locate obstacle
        self.get_obstacle()

        # Generate dynamic path
        start = time.time()
        path_len = self.get_path_length()
        # path_len = MAX_LENGTH
        s_start = self.s
        s_end = self.s + path_len
        end_idx = self.length_to_index(self.c_i, path_len)
        angle_diff = self.get_angle_wrt_route()
        cand_list = self.generate_path(
                s_start, s_end, PATH_OFFSET, angle_diff)
        # print("-----Generate Dynamic Path")
        # print("Path length: {} m").format(path_len)
        # print("End index: {}").format(end_idx)
        # print("Angle diff: {}").format(angle_diff)
        # print("Candidates: {}").format(len(cand_list))
        self.convert_cartesian(cand_list, s_start, s_end)
        end = time.time()
        gen_time = start - end
        rospy.logwarn("Generation time: %f", end - start)

        # Select best path
        start = time.time()
        self.set_global_cost(cand_list, s_end)
        self.set_obs_cost(cand_list, s_start, s_end)
        opt_idx = self.find_optimal_path(cand_list)
        end = time.time()
        sel_time = start - end
        # print("Optimal path idx: {}").format(opt_idx)
        rospy.logwarn("Selection time: %f", end - start)

        # record
        f = open('./record.csv', 'w')
        with f:
            writer = csv.writer(f)
            writer.writerow([gen_time, sel_time])
        f.close()
        # visualization
        self.visualize_candidate(cand_list)

        # footage
        self._footage.points.append(self._current_pose.position)
        self._footage_waypoints_publisher.publish(self._footage)

        # Generate waypoints
        self._waypoints_queue.clear()
        
        # Send optimal path to local
        if opt_idx != -1:
            self._local_path.points = [] # clear
            for i in range(len(cand_list[opt_idx].x)):
                waypoint = Pose()
                waypoint.position.x = cand_list[opt_idx].x[i]
                waypoint.position.y = cand_list[opt_idx].y[i]
                self._local_path.points.append(waypoint.position)
                self._waypoints_queue.append(waypoint)
            self._local_waypoints_publisher.publish(self._local_path)

        # start = time.time()
        # if self._global_waypoints:
        #     path = fot.frenet_optimal_planning(
        #         self.csp, self.s, self.c_speed, self.c_d, self.c_dd, self.c_ddd, None)

        #     if not path:
        #         print("-------No local path----------")
        #     else:
        #         self._local_path.points = []
        #         for i in range(len(path.t)):
        #             if i == 0:
        #                 continue
        #             waypoint = Pose()
        #             waypoint.position.x = path.x[i]
        #             waypoint.position.y = path.y[i]
        #             self._local_path.points.append(waypoint.position)
        #             # print(path.x[i])
        #             # print(path.y[i])
        #             self._waypoints_queue.append(waypoint)                
            
        #     self._local_waypoints_publisher.publish(self._local_path)

        # end = time.time()
        
        # rospy.logwarn("Planning time: %f", end - start)
        # print("Local waypoints {}").format(len(self._waypoints_queue))

        if not self._waypoints_queue:
            control = CarlaEgoVehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False

            return control

        #   Buffering the waypoints
        if len(self._waypoint_buffer) <= self._buffer_size:
            for i in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(self._waypoints_queue.popleft())
                else:
                    break

        self._local_buffer.points = []
        for i in range(len(self._waypoint_buffer)):
            self._local_buffer.points.append(self._waypoint_buffer[i].position)
        self._local_buffer_publisher.publish(self._local_buffer)
        # print("Local waypoints buffer {}").format(len(self._waypoint_buffer))
        # plt.show()

        # target lookahead point
        min_lookahead = self._buffer_size / 2
        max_lookahead = self._buffer_size - 1
        lookahead = min_lookahead + int(self._buffer_size / 2 * 1.0 * self._current_speed / target_speed) 
        lookahead = min(lookahead, max_lookahead)
        target_route_point = self._waypoint_buffer[lookahead]
        self.visualize_lookahead(target_route_point)
        # print("Current speed: {}").format(self._current_speed)
        # print("Lookahead: {}").format(lookahead)
        # print("Target route point: {} {}").format(target_route_point.position.x, target_route_point.position.y)

        # for us redlight-detection
        self.target_waypoint = self.get_waypoint(target_route_point.position)

        target_point = PointStamped()
        target_point.header.frame_id = "map"
        target_point.point.x = target_route_point.position.x
        target_point.point.y = target_route_point.position.y
        target_point.point.z = target_route_point.position.z
        self._target_point_publisher.publish(target_point)

        # move using PID controllers
        control = self._vehicle_controller.run_step(
            target_speed, self._current_speed, self._current_pose, target_route_point)
        # print("---PIC controller---")
        # print("Steer: {}").format(control.steer)
        # print("Throttle: {}").format(control.throttle)
        # print("Brake: {}").format(control.brake)
        # print("Target speed: {}").format(target_speed)
        # print("Current speed: {}").format(self._current_speed)

        # get current ego position in Frenet coordinate
        c_s, c_d, c_i = self.locate_vehicle(self._current_pose.position)
        self.s = c_s
        self.c_d = c_d
        # print("--- Frenet ---")
        # print("Arc length: {}, Lateral offest: {}, index: {}").format(c_s, c_d, c_i)


        # # calculate directional vector w.r.t the route
        # path_yaw = self.csp.calc_yaw(c_s) # global path yaw [deg]
        # dy = math.atan(path_yaw) 
        # s_vec = np.array([dy, 1]) 
        # s_vec /= np.linalg.norm(s_vec) # normalized s-direction vector
        # if not math.isnan(self._vehicle_yaw):
        #     dy = math.atan(self._vehicle_yaw)
        # ego_vec = np.array([dy, 1]) # normalized vehicle heading vector
        # ego_vec /= np.linalg.norm(ego_vec)
        # d_vec = np.array([-s_vec[1], s_vec[0]]) # normalized d-direction vector
        
        # # update vehicle state information for Frenet coordinate
        # self.c_d_d = d_vec.dot(ego_vec) * self._current_speed_2d # current lateral speed [m/s]
        # self.c_d_dd = d_vec.dot(ego_vec) * self._current_accel
        # self.s = c_s
        # self.c_d = c_d
        # self.c_i = c_i
        # self.c_speed = self._current_speed_2d
        # print("Vehicle acceleration: {} m/s^2").format(self._current_accel)
        # print("Lateral speed: {} km/h").format(self.c_d_d*3.6)
        # print("Lateral acceleration: {} m/s^2").format(self.c_d_dd)

        # purge the queue of obsolete waypoints
        max_index = -1

        # set min distance percent according to the curvature
        # sampling_radius = target_speed * 1 / 3.6  # 1 seconds horizon
        sampling_radius = self._current_speed * 1 / 3.6  # 1 seconds horizon
        sampling_radius = MIN_LENGTH * 0.9 if sampling_radius < MIN_LENGTH * 0.9 else sampling_radius
        c_ahead = self.csp.calc_curvature(self.s + sampling_radius * self.MIN_DISTANCE_PERCENTAGE) # curvature ahead
        min_distance = sampling_radius * self.MIN_DISTANCE_PERCENTAGE
        
        # print("min_distance: {}").format(min_distance)
        for i, route_point in enumerate(self._waypoint_buffer):
            if distance_vehicle(
                    route_point, self._current_pose.position) < min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        return control





