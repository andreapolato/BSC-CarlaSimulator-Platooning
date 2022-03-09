from concurrent.futures import process
import glob
import math
from msilib.schema import Directory
import os
import re
import sys
from PIL import Image

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
import matplotlib.pyplot as plt
import lane_detector as lane
import csv


#-------------------------------------
#****** CAMERA IMAGE DIMENSIONS ******
#-------------------------------------
IMG_WIDTH = 1280
IMG_HEIGHT = 720

actor_list = []

def convert_time(seconds):
    seconds = seconds%(24*3600)
    hrs = (seconds//3600)
    seconds %= 3600
    mins = seconds//60
    seconds %= 60
    mill = (seconds*1000)%1000
    return "%d:%02d:%02d:%04d"%(hrs,mins,seconds,mill)

def extract_data(snap,vehicle,f):
    vehicle_snap=snap.find(vehicle.id)
    transform = vehicle_snap.get_transform()
    frame = str(snap.frame)
    time = convert_time(snap.timestamp.elapsed_seconds)
    id = str(vehicle.id)
    type = str(vehicle.type_id)
    x = str("{0:10.3f}".format(transform.location.x))
    y = str("{0:10.3f}".format(transform.location.y))
    z = str("{0:10.3f}".format(transform.location.z))
    vel = vehicle_snap.get_velocity()
    speed = str('%15.2f'%(3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)))
    gear = str(vehicle.get_control().gear)
    output = (frame + ',' + time + ',' + id + ',' + type + ',' + x + ',' + y + ',' + z + ',' + speed + ',' + gear)
    w = csv.writer(f)
    w.writerow(output)

def get_vehicle_data(snap,f):
    for vehicle in actor_list: #TROVARE UN FILTRO PER VEICOLI
        if isinstance(vehicle, carla.Vehicle):
            extract_data(snap,vehicle,f)

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
    spawn = random.choice(world.get_map().get_spawn_points())
    vehicle_a = world.spawn_actor(model3, spawn)

    #subject.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0)) #code for manual control
    vehicle.set_autopilot(True)
    vehicle_a.set_autopilot(True)
    
    #--------------------------------------------------------
    #****** SPAWN RGB CAMERA AND FIX IT TO THE VEHICLE ******
    #--------------------------------------------------------
    rgb_bp = blueprint_library.find('sensor.camera.rgb')
    rgb_bp.set_attribute('image_size_x', f'{IMG_WIDTH}')
    rgb_bp.set_attribute('image_size_y', f'{IMG_HEIGHT}')
    rgb_bp.set_attribute('fov', '110')
    rgb_bp.set_attribute('fstop', '1.0')
    rgb_bp.set_attribute('sensor_tick', '0.03')

    spawn = carla.Transform(carla.Location(x=3.5, z=1.2), carla.Rotation(pitch=-7))
    rgb_cam = world.spawn_actor(rgb_bp, spawn, attach_to=vehicle_a)
    rgb_cam_a = world.spawn_actor(rgb_bp, spawn, attach_to=vehicle)

    #---------------------------------------------------
    #****** SPAWN LIDAR AND FIX IT TO THE VEHICLE ******
    #---------------------------------------------------
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar = world.spawn_actor(lidar_bp, spawn, attach_to=vehicle)
    lidar_a = world.spawn_actor(lidar_bp, spawn, attach_to=vehicle_a)

    actor_list.append(vehicle)
    actor_list.append(rgb_cam)
    actor_list.append(lidar)

    dir = 'recs/' + time.strftime("%Y%m%d-%H%M%S")
    if not os.path.exists(dir):
        os.makedirs(dir)

    with open(dir + '/vehicle_data.csv', 'w') as f:
        w = csv.writer(f)
        print('CSV file created.')
        w.writerow(['snap','time', 'id', 'type', 'x', 'y', 'z', 'Km/h', 'Gear'])
    f = open(dir + '/vehicle_data.csv','a+')
    
    rgb_cam.listen(lambda image: image.save_to_disk(dir + '/camera/%d'%image.frame))
    lidar.listen(lambda point_cloud: point_cloud.save_to_disk(dir + '/lidar/%.6d.ply' % point_cloud.frame))
    world.on_tick(lambda snap: get_vehicle_data(snap,f))

    while True:
        world.tick()

except KeyboardInterrupt:
    pass
finally:
    f.close()
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')