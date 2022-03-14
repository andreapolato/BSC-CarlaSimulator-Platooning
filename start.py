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
N=1

actor_list = []
snap_list = []

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
    

def manage_follower(snap):
    with open(dir + '/vehicle_data_leader.csv', newline='') as f:
        r = csv.DictReader(f, fieldnames=fn)
        t = s = b = v = 0.0
        for row in r:
            if row['Snap'] == str(snap.frame-70):             
                vehicle_snap=snap.find(PlatooningFollower.id)
                transform = vehicle_snap.get_transform()
                #fx = float("{0:10.3f}".format(transform.location.x))
                #fy = float("{0:10.3f}".format(transform.location.y))
                #lx = float(row['X'])
                #ly = float(row['Y'])
                #curr_fit = abs(fx-lx) + abs(fy-ly)
                #print(fx,fy,lx,ly,curr_fit)
                #if curr_fit<best_fit:
                t = float(row['Throttle'])
                s = float(row['Steer'])
                b = float(row['Brake'])
                v = float(row['Km/h'])
                PlatooningFollower.apply_control(carla.VehicleControl(throttle=t, steer=s, brake=b))
#    return t,s,b
        
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
    PlatooningFollower.apply_control(carla.VehicleControl(
        throttle=1.0, steer=0.0, brake=0.0))
    actor_list.append(PlatooningLeader)
    actor_list.append(PlatooningFollower)

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

    last_snap=0
    world.on_tick(lambda snap: record_vehicle_data(snap))
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