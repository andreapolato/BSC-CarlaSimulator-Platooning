from concurrent.futures import process
from dis import dis
import glob
from multiprocessing.connection import wait
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
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import lane_detector as lane
from time import sleep
from visualize_sensors import DisplayManager, SensorManager
import curved_lane_detection as det
import csv

IMG_WIDTH = 1280
IMG_HEIGHT = 720

actor_list = []
snap_list = []
big_dist = True

def convert_time(seconds):
    seconds = seconds%(24*3600)
    hrs = (seconds//3600)
    seconds %= 3600
    mins = seconds//60
    seconds %= 60
    mill = (seconds*1000)%1000
    return "%d:%02d:%02d:%04d"%(hrs,mins,seconds,mill)

def extract_data(snap,vehicle):
    vehicle_snap = snap.find(vehicle.id)
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
    throttle = str(vehicle.get_control().throttle)
    steer = str(vehicle.get_control().steer)
    brake = str(vehicle.get_control().brake)
    with open(dir + '/vehicle_data_%s.csv'%('leader' if vehicle==PlatooningLeader else 'follower'), 'a+', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fn)
        output = {'Snap':frame,'Time':time, 'ID':id, 'Type':type, 'X':x, 'Y':y, 'Z':z, 'Km/h':speed, 'Throttle':throttle, 'Steer':steer, 'Brake':brake}
        w.writerow(output)

def record_vehicle_data(snap):
    snap_list.append(snap)
    for vehicle in actor_list:
        if isinstance(vehicle, carla.Vehicle):
            extract_data(snap,vehicle)
    #t,s,b = manage_follower(snap)
    
def set_steer(fx,fy):
    with open(dir + '/vehicle_data_leader.csv', newline='') as f:
        r = csv.DictReader(f, fieldnames=fn)
        n=0
        s=0.0
        for row in r:
            if n!=0:
                lx = float(row['X'])
                ly = float(row['Y'])
                if abs(lx-fx)<=2.0 and abs(ly-fy)<=2.0:
                    s=float(row['Steer'])
                    return s
            else:
                n+=1
        return s

def manage_follower(snap):
    with open(dir + '/vehicle_data_leader.csv', newline='') as f:
        r = csv.DictReader(f, fieldnames=fn)
        vehicle_snap = snap.find(PlatooningFollower.id)
        transform = vehicle_snap.get_transform()
        vel = vehicle_snap.get_velocity()
        fspeed = float('%15.2f'%(3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)))
        fx = float("{0:10.3f}".format(transform.location.x))
        fy = float("{0:10.3f}".format(transform.location.y))
        n=0
        t=s=b=0
        for row in r:
            if n!=0:
                if row['Snap'] == str(snap.frame):
                    lspeed = float(row['Km/h'])
                    delta = lspeed-fspeed
                    global big_dist
                    if (delta>=0):
                        if big_dist:
                            t = 1.0
                            b = 0.0
                            s = set_steer(fx,fy)
                        else:
                            t = delta/10+0.1 if delta/10+0.1 <= 1.0 else 1.0
                            b = 0.0
                            s = set_steer(fx,fy)
                        #PlatooningFollower.apply_control(carla.VehicleControl(throttle=t, steer=s, brake=b))
                    else:
                        t = 0.0
                        b = -delta/10 if -delta/10 <= 1.0 else 1.0
                        s = set_steer(fx,fy)
                    PlatooningFollower.apply_control(carla.VehicleControl(throttle=t, steer=s, brake=b))
            else:
                n+=1

def get_points(points):
    min = 500
    for p in points:
        if p.point.x<min:
            min = p.point.x
    global big_dist
    print(min)
    big_dist = True if min>=4 else False
            


        
try:
    #----------------------------------------------
    #****** CONNECT TO THE SIMULATION SERVER ******
    #----------------------------------------------
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    
    world = client.get_world()
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.01
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()

    for v in world.get_actors():
        if isinstance(v, carla.Vehicle):
            v.destroy()

    #------------------------------------
    #****** SPAWN TEST SUBJECT CAR ******
    #------------------------------------
    model3 = blueprint_library.filter('model3')[0]
    audiTT = blueprint_library.filter('tt')[0]

    spawn = random.choice(world.get_map().get_spawn_points())

    PlatooningLeader = world.spawn_actor(model3, spawn)
    PlatooningLeader.set_autopilot(True)

    time.sleep(2)
    PlatooningFollower = world.spawn_actor(audiTT, spawn)
    PlatooningFollower.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0))
    actor_list.append(PlatooningLeader)
    actor_list.append(PlatooningFollower)

    spawn = carla.Transform(carla.Location(x=2.5, z=0.8))
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('horizontal_fov','90')
    lidar_bp.set_attribute('upper_fov','5')
    lidar_bp.set_attribute('lower_fov','-5')
    LidarFollower = world.spawn_actor(lidar_bp, spawn, attach_to=PlatooningFollower)
    actor_list.append(LidarFollower)

    dir = 'recs/' + time.strftime("%Y%m%d-%H%M%S")
    if not os.path.exists(dir):
        os.makedirs(dir)

    with open(dir + '/vehicle_data_leader.csv', 'w', newline='') as f:
        fn = ['Snap','Time', 'ID', 'Type', 'X', 'Y', 'Z', 'Km/h', 'Throttle', 'Steer', 'Brake']
        w = csv.DictWriter(f, fieldnames=fn)
        print('Leader CSV file created.')
        w.writeheader()

    with open(dir + '/vehicle_data_follower.csv', 'w', newline='') as f:
        fn = ['Snap','Time', 'ID', 'Type', 'X', 'Y', 'Z', 'Km/h', 'Throttle', 'Steer', 'Brake']
        w = csv.DictWriter(f, fieldnames=fn)
        print('Follower CSV file created.')
        w.writeheader()

    with open(dir + '/lidar_data_follower.csv', 'w', newline='') as f:
        ln = ['Location']
        w = csv.DictWriter(f, fieldnames=ln)
        print('Follower CSV file created.')
        w.writeheader()

    last_snap=0
    world.on_tick(lambda snap: record_vehicle_data(snap))
    LidarFollower.listen(lambda points: get_points(points))
    time.sleep(1)
    while True:
        if(snap_list):
            manage_follower(snap_list[-1])
        
except KeyboardInterrupt:
    pass

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')