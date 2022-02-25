from asyncio.windows_events import NULL
from concurrent.futures import process
import glob
import os
from re import sub
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import cv2
import dynamic_weather
import generate_traffic
import visualize_sensors

#-------------------------------------
#****** CAMERA IMAGE DIMENSIONS ******
#-------------------------------------
IMG_WIDTH = 640
IMG_HEIGHT = 480

actor_list = []

def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IMG_HEIGHT, IMG_WIDTH, 4))
    i3 = i2[:, :, :3]  # remove the alpha (basically, remove the 4th index  of every pixel. Converting RGBA to RGB)
    cv2.imshow("", i3)  # show it.
    cv2.waitKey(1)
    return i2/255.0  # normalize

def process_dist(measurement):
    m = np.array(measurement.raw_data)
    print(m)

def refresh_weather(w, et, uf):
        timestamp = world.wait_for_tick(seconds=30.0).timestamp
        et += timestamp.delta_seconds
        if et > uf:
            w.tick(dynamic_weather.argsDefiner.speed * et)
            world.set_weather(w.weather)
            sys.stdout.write('\r' + str(w) + 12 * ' ')
            sys.stdout.flush()
            et = 0.0
            
try:
    #----------------------------------------------
    #****** CONNECT TO THE SIMULATION SERVER ******
    #----------------------------------------------
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    
    world = client.get_world()
    
    blueprint_library = world.get_blueprint_library()

    #------------------------------------
    #****** SPAWN TEST SUBJECT CAR ******
    #------------------------------------
    model3 = blueprint_library.filter('model3')[0]
    spawn = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(model3, spawn)
    #subject.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0)) #code for manual control
    vehicle.set_autopilot(True)
    
    #--------------------------------------------------------
    #****** SPAWN RGB CAMERA AND FIX IT TO THE VEHICLE ******
    #--------------------------------------------------------
    rgb_bp = blueprint_library.find('sensor.camera.rgb')
    rgb_bp.set_attribute('image_size_x', f'{IMG_WIDTH}')
    rgb_bp.set_attribute('image_size_y', f'{IMG_HEIGHT}')
    rgb_bp.set_attribute('fov', '100')

    spawn = carla.Transform(carla.Location(x=2.5, z=0.7))
    rgb_cam = world.spawn_actor(rgb_bp, spawn, attach_to=vehicle)

    #---------------------------------------------------
    #****** SPAWN LIDAR AND FIX IT TO THE VEHICLE ******
    #---------------------------------------------------
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar = world.spawn_actor(lidar_bp, spawn, attach_to=vehicle)

    actor_list.append(vehicle)
    actor_list.append(rgb_cam)
    actor_list.append(lidar)

    #lidar.listen(lambda data: process_dist(data))

    print('generating traffic')
    generate_traffic.generate()
    print('opening sensors window')
    visualize_sensors.run_simulation(visualize_sensors.argsDefiner(),client)

    
    speed_factor = dynamic_weather.argsDefiner.speed
    update_freq = 0.1 / speed_factor

    weather = dynamic_weather.Weather(world.get_weather())

    elapsed_time = 0.0

    while True:
        world.tick()
        refresh_weather(weather, elapsed_time, update_freq)

except KeyboardInterrupt:
    pass
finally:

    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')