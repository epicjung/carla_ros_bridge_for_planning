# [EE585] Mobile Robotics and Autonomous Navigation
: Guideline for CARLA simulation

담당자: Minho Oh, Euigon Epic Jung
유형: Notification for EE585
작성일시: 2021년 8월 30일 오후 4:18

- **Contents**

---

# [CARLA Installation]

> This part guides how to install CARLA and use it with ROS.

## Requirements

***""Under installation guides are based on Ubuntu 18.04 in Local Desktop.""***

### 1. NVIDIA Graph Driver

***"" I think NVIDIA Graph Drivier was pre-installed on each PC.""***

### 2. Docker

**Ref. Link: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)**

**Summary for docker installation**

```python
# Uninstall old version if you need
sudo apt-get remove docker docker-engine docker.io containerd runc

# Set up the repository
# Update the apt package index and install packages to allow apt to use a repository over HTTPS:
sudo apt-get update
sudo apt-get install \
  apt-transport-https \
  ca-certificates \
  curl \
  gnupg \
  lsb-release

# Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
# Use the following command to set up the stable repository.
echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
# 1. Update the apt package index, and install the latest version of Docker Engine and containerd: 
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

# 2. Verify that Docker Engine is installed correctly by running the hello-world image.
sudo docker run hello-world
```

### 3. nvidia-docker 2

**Ref. Link: 
[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html)
[](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)[https://nodoudt.tistory.com/50](https://nodoudt.tistory.com/50)**

**Summary for docker installation**

```python
# Set Repository
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update

# Set Repository Key
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -

# Uninstall the nvidia-docker 1.0
sudo docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
sudo apt-get purge nvidia-docker

# Install nvidia-docker 2.0
sudo apt-get install nvidia-docker2
sudo pkill -SIGHUP dockerd
```

## Preparation

1. Get the docker file for carla-ros-bridge by the following command.

    ```python
    git clone https://gitlab.com/Minho5/carla_ros_bridge
    ```

2. Please download the "CARLA_0.9.10.1.tar.gz" and "AdditionalMaps_0.9.10.1.tar.gz" from [https://github.com/carla-simulator/carla/releases/tag/0.9.10.1](https://github.com/carla-simulator/carla/releases/tag/0.9.10.1).
And, please put CARLA simulator in the same directory as the Dockerfile as following example.

    **~/carla_ros_bridge
          |- AdditionalMaps_0.9.10.1.tar.gz
          |- CARLA_0.9.10.1.tar.gz
          |- Dockerfile.melodic
          |- launch_container.sh
          |- LICENSE
          |- README.md**

3. Build Docker image by following command

    ```python
    sudo docker build -t carla:0.9.10 -f Dockerfile.melodic .
    ```

4. Create Docker Container and Launch the docker by following command.

    ```python
    sudo ./launch_container.sh
    ```

    If the execution is successful, you will see the following figure.

    ![%5BEE585%5D%20Mobile%20Robotics%20and%20Autonomous%20Navigation%20%202436ca3e82df498b9b885164af1f83d4/Untitled.png](%5BEE585%5D%20Mobile%20Robotics%20and%20Autonomous%20Navigation%20%202436ca3e82df498b9b885164af1f83d4/Untitled.png)

    *** If you get the error message such as "time out(?) like following example, just ignore it.

## Cautions when you using Docker.

---

# [Run Demo]

### Why Carla ROS bridge?

- Obtain sensor data (lidar, semnatic lidar, cameras, GNSS, Radar, IMU) via ROS topics
- Obtain object data (Transforms, traffic light status, collision, lane invasion) via ROS topics
- Control autonomous agents (Steer/Throttle/Brake) by publishing the value via ROS topics or via RVIZ
- Control CARLA simulation by playing and pausing and setting simulation parameters

### Run **CARLA**

```python
**# Terminal 1.
# After "sudo ./launch_container.sh"**
cd CARLA_0.9.10.1
./CarlaUE4.sh
```

### CARLA+ROS Demo.

: Turn on the carla_ros_bridge node by opening *"bridge.py"*

```python
**# Terminal 2.**
**# After "sudo ./launch_container.sh"**
**# "roslaunch" include running the ROS CORE
# Option 1: run only carla_ros_bridge
roslaunch carla_ros_bridge carla_ros_bridge.launch**

**# Option 2: run bridge with rviz, which is visualizator of ROS**
# = **carla_ros_bridge with all arg true + rviz**
**roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch**

**# Option 3: run bridge with a random vehicle with manual control**
# = **carla_ros_bridge**                           
#   **+ carla_example_ego_vehicle.launch** 
#   **+ carla carla_manual_control.launch**
**roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

# Option 4: Demo for autonomous driving
# = carla_ros_bridge
#   + carla_example_ego_vehicle
#   + carla_waypoint_publisher
#   + carla_ad_agent
#   + carla_manual_control
roslaunch carla_ad_demo carla_ad_demo.launch

# Option 5: Demo for scenario running
# = carla_ros_bridge
#   + carla_example_ego_vehicle
#   + carla_spectator_camera
#   + carla_waypoint_publisher
#   + carla_ros_scenario_runner
#   + carla_twist_to_control
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch**
```

---

# Framework for ad_demo

![Untitled](%5BEE585%5D%20Mobile%20Robotics%20and%20Autonomous%20Navigation%20%202436ca3e82df498b9b885164af1f83d4/Untitled%201.png)

1. *carla_ad_demo_with_rviz.launch* launches *carla_ad_demo.launch*
2. *carla_ad_demo.launch* runs *carla_ad_agent.py*
3. In *carla_ad_agent.py,* BasicAgent is constructed in its constructor.

```python
def __init__(self, role_name, target_speed, avoid_risk):
        """
        Constructor
        """
        self._route_assigned = False
        self._global_plan = None
        self._agent = None
        self._target_speed = target_speed
        rospy.on_shutdown(self.on_shutdown)

        # wait for ego vehicle
        vehicle_info = rospy.wait_for_message(
            "/carla/{}/vehicle_info".format(role_name), CarlaEgoVehicleInfo)

        self._route_subscriber = rospy.Subscriber(
            "/carla/{}/waypoints".format(role_name), Path, self.path_updated)

        self._target_speed_subscriber = rospy.Subscriber(
            "/carla/{}/target_speed".format(role_name), Float64, self.target_speed_updated)

        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd".format(role_name), CarlaEgoVehicleControl, queue_size=1)

        self._agent = BasicAgent(role_name, vehicle_info.id,  # pylint: disable=no-member
                                 avoid_risk)
```

1. The local planner is set to BasicAgent which is located at *basic_agent.py.*
    - The globally planned path (given) is set to the local planner of the agent in the run_step() function.
    - run_step() function is where a step operation is executed so please look at run_step() functions carefully in the scripts to better understand how the program works.

```python
def run_step(self):
      """
      Execute one step of navigation.
      """
      control = CarlaEgoVehicleControl()
      control.steer = 0.0
      control.throttle = 0.0
      control.brake = 0.0
      control.hand_brake = False

      if not self._agent:
          rospy.loginfo("Waiting for ego vehicle...")
          return control

      if not self._route_assigned and self._global_plan:
          rospy.loginfo("Assigning plan...")
          self._agent._local_planner.set_global_plan(  # pylint: disable=protected-access
              self._global_plan.poses)
          self._route_assigned = True
      else:
          control = self._agent.run_step(self._target_speed)

      return control
```

1. The return value of the run_step() function in *my_local_planner.py* is "control" which is published as CarlaEgoVehicleControl.msg (*catkin_ws/src/ros-bridge/carla_msgs/msg*)
    - By giving the target point to the vehicle controller, Carla uses its built-in PID controllers to obtain control value to follow that target point.

```python
# target waypoint
target_route_point = self._waypoint_buffer[0]

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

# purge the queue of obsolete waypoints
max_index = -1

sampling_radius = target_speed * 1 / 3.6  # 1 seconds horizon
min_distance = sampling_radius * self.MIN_DISTANCE_PERCENTAGE

for i, route_point in enumerate(self._waypoint_buffer):
    if distance_vehicle(
            route_point, self._current_pose.position) < min_distance:
        max_index = i
if max_index >= 0:
    for i in range(max_index + 1):
        self._waypoint_buffer.popleft()

return control
```

# [1st Task: Path Planning]

> **Implement the Local Planning and reach the goal through a given scenario.**

## Task

![demo.gif](%5BEE585%5D%20Mobile%20Robotics%20and%20Autonomous%20Navigation%20%202436ca3e82df498b9b885164af1f83d4/demo.gif)

An example video for local planner

![Untitled](%5BEE585%5D%20Mobile%20Robotics%20and%20Autonomous%20Navigation%20%202436ca3e82df498b9b885164af1f83d4/Untitled%202.png)

The start and goal of the given task

- Modify *my_local_planner.py *****to reach the goal safely
    - *my_local_planner.py* ****is in *catkin_ws/src/ros-bridge/carla_ad_agent/src/*
- Your local planner should follow the globally planned path as much as possible
- Scenarios:
    - Dynamic and static vehicles will be spawned in the street. Their spawning locations will be identical each time you spawn them, but their destinations might change.
        - FYI, to evaluate your program fairly, the npcs' spawning points and travelling routes will be identical during evaluation.
    - You can ignore the traffic lights. The other npcs will not follow the traffic rules as well.
    - You should not cross any solid line, both yellow and white. Invasion of solid lines will be counted.
- Things you could change:
    - These are the features that are already programmed
        - PID controller that follows the target point
        - Obstacle (including actors) detection around the ego vehicle
        - Global path planner
    - You may use LiDAR to locate the road markings and other obstacles that cannot be obtained by locating the actors and create an occupancy grid map
    - You can modify any of the above features to accomplish the given task.
- How to run

    ```bash
    # 1. Open world
    bash ~/CARLA_0.9.10.1/CarlaUE4.sh

    # 2. Spawn npcs
    rosrun carla_ad_demo spawn_npc.py

    # 3. Run your grogram
    rosrun carla_ad_demo carla_ad_demo_with_rviz.launch

    ```

## Evaluation

[Ranking](https://www.notion.so/ee3fe651305d4252a0235df60a7f25a8)

1. **Progress [%]**
    - Arc-length distance from the start to the current pose divided by the total arc-length of the global path
    - If goal is reached **(0.0 km/h speed within 4.0 m from the goal for 2 seconds)**, this criterion will be marked as 100%.
    - Scores
        - Rank 1: 10 points
        - Rank 2: 8 points
        - Rank 3: 6 points
        - Rank 4: 4 points
        - Rank 5~: 2 points
2. **Goal reached** 
    - Scores
        - 10 points if reached and 0 points if otherwise
3. **Run time [sec]**
    - **5 minutes max.**
    - Timer stops when the goal is reached
    - Scores
        - Rank 1: 10 points
        - Rank 2: 8 points
        - Rank 3: 6 points
        - Rank 4: 4 points
        - Rank 5~: 2 points
4. **Collision per meter**
    - The number of occasion when an ego vehicle collides with other actors or static obstacles per meter
    - Note that this criterion will be included in the ranking only when **progress is 100% or goal is reached**.
    - Scores
        - Rank 1: 5 points
        - Rank 2: 4 points
        - Rank 3: 3 points
        - Rank 4: 2 points
        - Rank 5~: 1 points
5. **Invasion per meter [only when progress is 100%]**
    - The number of occasion when an ego vehicle invades not allowed lines per meter
    - Note that this criterion will be included in the ranking only when **progress is 100% or goal is reached**.
    - Scores
        - Rank 1: 5 points
        - Rank 2: 4 points
        - Rank 3: 3 points
        - Rank 4: 2 points
        - Rank 5~: 1 points
6. **Extra credit**
    - Your own start-to-goal global path planning (+10 points)
    - Your own controller (+10 points)
    - Use sensors for detection (+10 points)
    - Please explain what you've changed in **readme.txt** file for TA.
- **Simulation result for your own test**
    - You can see the following text  if you close the program (i.e. ctrl+C), reach the goal, or exceed 5 mins of time limit.

        ![Untitled](%5BEE585%5D%20Mobile%20Robotics%20and%20Autonomous%20Navigation%20%202436ca3e82df498b9b885164af1f83d4/Untitled%203.png)

        Example result

- **Warnings**
    - Do not modify *evaluate.py* to get the better results. We will use our own code to evaluate your program

# [2nd Task: SLAM on CARLA]

> **Mount the open source for a state-of-the-art SLAM technology on the given platform in carla environment, and Improve it.**

## Task

Platform - sensor Extrinsic / Intrinsic

sensor_noise(?)

## Evaluation

Guideline for evo

Topics:

- Ground Truth: "/ee585/gt_odmetry"
- SLAM result odometry: "/ee585/slam_odometry"

[Evaluation contents](https://www.notion.so/e18f778cc47c4e0fac48b6505e8a068e)