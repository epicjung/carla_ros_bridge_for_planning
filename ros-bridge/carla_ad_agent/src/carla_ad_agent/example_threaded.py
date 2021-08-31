import glob
import os
import sys
try:
    sys.path.append(glob.glob('/home/michelle/carla_sim_8/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Imports
try:
    import carla
    from carla import ColorConverter as cc
except ImportError:
    raise RuntimeError('cannot import carla, make sure carla package is accessible')


import argparse
import random
import time
import weakref
import tqdm
import math
from PIL import Image
import threading
from subprocess import call
import logging

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
try:
    import queue
except ImportError:
    import Queue as queue
try:
    import pandas as pd
except ImportError:
    raise RuntimeError('cannot import pandas, make sure pandas package is installed')
try:
    import cv2
except ImportError:
    raise RuntimeError('cannot import opencv, make sure opencv package is installed')


# ==============================================================================


def get_font(pygame):
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def to_bgra_array(image, height=None, width=None):
    """Convert a CARLA raw image to a BGRA numpy array."""
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    if height is None or width is None:
        array = np.reshape(array, (image.height, image.width, 4))
    else:
        array = np.reshape(array, (height, width, 4))
    return array


def to_rgb_array(image, height=None, width=None):
    """Convert a CARLA raw image to a RGB numpy array."""
    array = to_bgra_array(image, height, width)
    # Convert BGRA (Carla) to RGB (PyGame and OpenCV).
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array


def vector_to_numpy (vector, txyp=True):
    array = []
    for i in range (len(vector)):
        array.append(np.asarray(np.frombuffer(vector[i].raw_data, dtype=np.dtype("int64"))))
    array = np.asarray(array)
    if txyp:
        array = array[:,[2,0,1,3]]
    return array


def rgb2gray(rgb):
        return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])


def rgb2pygamegray(rgb):
        return rgb.dot([0.298, 0.587, 0.114])[:,:,None].repeat(3,axis=2)


def should_quit(pygame):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def draw_image(surface, image, position=0, blend=False):
    image_surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (position, 0))

# ==============================================================================
# -- Carla Simulator -----------------------------------------------------------
# ==============================================================================


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, args):
        self.CARLA_MIN_DELTA_TIME = 0.4  # seconds
        self.world = world
        self.sensors = sensors
        self.args = args
        self.frame = None
        self.delta_seconds = 1.0 / args.fps
        self._queues = []
        self._settings = None

        # Check that FPS is at least > (1/MIN_DELTA)
        if self.delta_seconds > self.CARLA_MIN_DELTA_TIME:
            self.delta_seconds = self.CARLA_MIN_DELTA_TIME
            self.args.fps = 1.0/self.CARLA_MIN_DELTA_TIME

    def __enter__(self):
        self._settings = self.world.get_settings()

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        for x in data:
            if x is not None:
                assert x.frame == self.frame
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            try:
                data = sensor_queue.get(timeout=timeout)
                if data.frame == self.frame:
                    return data
            except queue.Empty:
                return None

# ==============================================================================
# -- run loop -----------------------------------------------------------------
# ==============================================================================


def run(args, client, number_surfaces = 1):
    t = threading.currentThread()
    print ("in thread")
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    actor_list = []
    pygame.init()
    save_delta_seconds = 1.0/ args.fps
    accumulative_delta_seconds = 0
    number_samples = 0
    if args.number_samples is -1:
        number_samples = -2
    if args.preview:
        display = pygame.display.set_mode(
            ((number_surfaces) * args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('RGB')

    font = get_font(pygame)
    clock = pygame.time.Clock()

    world = client.get_world()

    try:
        # Get the map and the starting point of the car
        m = world.get_map()

        # The libraries        
        blueprint_library = world.get_blueprint_library()

        vehicle = None
        while vehicle is None:
            print("Scenario not yet ready")
            time.sleep(1)
            possible_vehicles = world.get_actors().filter('vehicle.*')
            for vehicle_pos in possible_vehicles:
                if vehicle_pos.attributes['role_name'] == "hero":
                    vehicle = vehicle_pos
        start_pose = vehicle.get_transform()
        vehicle.set_simulate_physics(True)

        # Append the car to the list of actors
        actor_list.append(vehicle)

        # Set Autopilot
        if args.autopilot:
            # vehicle.apply_control(carla.VehicleControl(throttle=0.1, brake=0.1))
            vehicle.set_autopilot(True)
            print("Autopilot is set")
        else:
            waypoint = m.get_waypoint(start_pose.location)

        # Create a RGB camera
        sensor = blueprint_library.find('sensor.camera.rgb')
        sensor.set_attribute("image_size_x", str(args.width))
        sensor.set_attribute("image_size_y", str(args.height))
        sensor.set_attribute("fov", "110")
        sensor.set_attribute("enable_postprocess_effects", "True")

        camera_rgb = world.spawn_actor(sensor,
            carla.Transform(carla.Location(x=1.6, z=1.7), carla.Rotation(pitch=0)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)

        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_rgb, args=args) as sync_mode:
            pbar = tqdm.tqdm(total=args.number_samples)

            while number_samples < args.number_samples:
                if should_quit(pygame):
                    return

                # Advance the simulation and wait for the data.
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # vehicle next point 
                if not args.autopilot:
                    waypoint = random.choice(waypoint.next(random.uniform(0.001, 0.02)))
                    vehicle.set_transform(waypoint.transform)

                # Get the RGB Image
                array_rgb = to_rgb_array(image_rgb)

                # Sum the delta time from simulation
                accumulative_delta_seconds += snapshot.timestamp.delta_seconds

                if args.preview:
                    # Draw RGB image
                    draw_image(display, array_rgb, position=0*args.width)

                    if accumulative_delta_seconds >= save_delta_seconds:
                        clock.tick()
                        pygame.display.flip()
                
                if accumulative_delta_seconds >= save_delta_seconds:
                    # reset delta seconds time
                    accumulative_delta_seconds = 0
                    if args.number_samples is not -1:
                        number_samples += 1
                        pbar.update(1)

        while getattr(t, "do_run", True):
            world.wait_for_tick()

    finally:

        print('destroying actors.')
        for actor in actor_list:
            print ("\tactor %s"%actor)
            actor.destroy()

        pygame.quit()
        print('done.')
