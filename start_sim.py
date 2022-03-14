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

#-------------------------------------
#****** CAMERA IMAGE DIMENSIONS ******
#-------------------------------------
IMG_WIDTH = 1280
IMG_HEIGHT = 720

actor_list = []

def process_img(image):
    image.save_to_disk('recs/%d.png'%image.frame)
    i = np.array(image.raw_data)
    i2 = i.reshape((IMG_HEIGHT, IMG_WIDTH, 4))
    i3 = i2[:, :, :3]  # remove the alpha (basically, remove the 4th index  of every pixel. Converting RGBA to RGB)
    cv2.imshow("", i3)  # show it.
    cv2.waitKey(1)
    return i2/255.0  # normalize

def manage_vehicle(image, dir, vehicle):
    image.save_to_disk(dir+'/raw_imgs/%d.png'%image.frame)
    i = np.array(image.raw_data)
    i2 = i.reshape((IMG_HEIGHT, IMG_WIDTH, 4))
    #res, dist = det.detect_steering(i2)
    res, dist = lane.detect(i2)
    cv2.imwrite(dir+'/%d.png'%image.frame, res)
    vehicle.apply_control(carla.VehicleControl(throttle=0.4, steer=dist/960))
    #cv2.imshow("lanes",res)
    #cv2.waitKey(1)

def steer(dist):
    if abs(dist>16):
        return dist/640
    return 0.0


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
    #gear = str(vehicle.get_control().gear)
    throttle = str(vehicle.get_control().throttle)
    steer = str(vehicle.get_control().steer)
    brake = str(vehicle.get_control().brake)
    with open(dir + '/vehicle_data_%s.csv'%('leader' if vehicle==PlatooningLeader else 'follower'), 'a+', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fn)
        output = {'Snap':frame,'Time':time, 'ID':id, 'Type':type, 'X':x, 'Y':y, 'Z':z, 'Km/h':speed, 'Throttle':throttle, 'Steer':steer, 'Brake':brake}
        w.writerow(output)
    return throttle, steer

def get_vehicle_data(snap,f):
    for vehicle in actor_list: #TROVARE UN FILTRO PER VEICOLI
        if isinstance(vehicle, carla.Vehicle):
            extract_data(snap,vehicle,f)
            if vehicle==PlatooningFollower:
                with open(dir + '/vehicle_data_leader.csv', newline='') as f:
                    r = csv.DictReader(f, fieldnames=fn)
                    for row in r:
                        vehicle_snap=snap.find(vehicle.id)
                        transform = vehicle_snap.get_transform()
                        x = str("{0:10.3f}".format(transform.location.x))
                        y = str("{0:10.3f}".format(transform.location.y))
                        if row['Type']=='vehicle.tesla.model3':
                            print(x,y,row['X'],row['Y'])
                            if abs(float(row['X'])-float(x))<0.1 and abs(float(row['Y'])-float(y))<0.1:
                                PlatooningFollower.apply_control(carla.VehicleControl(throttle=float(row['Throttle']), steer=float(row['Steer']), brake=float(row['Brake'])))
                            else:
                                PlatooningFollower.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))
try:
    #----------------------------------------------
    #****** CONNECT TO THE SIMULATION SERVER ******
    #----------------------------------------------
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    
    world = client.get_world()

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
    
    #--------------------------------------------------------
    #****** SPAWN RGB CAMERA AND FIX IT TO THE VEHICLE ******
    #--------------------------------------------------------
    rgb_bp = blueprint_library.find('sensor.camera.rgb')
    rgb_bp.set_attribute('image_size_x', f'{IMG_WIDTH}')
    rgb_bp.set_attribute('image_size_y', f'{IMG_HEIGHT}')
    rgb_bp.set_attribute('fov', '110')
    rgb_bp.set_attribute('fstop', '1.0')
    rgb_bp.set_attribute('sensor_tick', '0.02')

    spawn = carla.Transform(carla.Location(x=3.5, z=1.2), carla.Rotation(pitch=-7))
    rgbLeader = world.spawn_actor(rgb_bp, spawn, attach_to=PlatooningLeader)
    #rgbFollower = world.spawn_actor(rgb_bp, spawn, attach_to=PlatooningFollower)

    #---------------------------------------------------
    #****** SPAWN LIDAR AND FIX IT TO THE VEHICLE ******
    #---------------------------------------------------
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    LidarLeader = world.spawn_actor(lidar_bp, spawn, attach_to=PlatooningLeader)

    actor_list.append(PlatooningLeader)
    actor_list.append(PlatooningFollower)
    actor_list.append(rgbLeader)
    actor_list.append(LidarLeader)

    dir = 'recs/' + time.strftime("%Y%m%d-%H%M%S")
    if not os.path.exists(dir):
        os.makedirs(dir)

    with open(dir + '/vehicle_data_leader.csv', 'w', newline='') as f:
        fn = ['Snap','Time', 'ID', 'Type', 'X', 'Y', 'Z', 'Km/h', 'Throttle', 'Steer', 'Brake']
        w = csv.DictWriter(f, fieldnames=fn)
        print('CSV file created.')
        w.writeheader()

    with open(dir + '/vehicle_data_follower.csv', 'w', newline='') as f:
        fn = ['Snap','Time', 'ID', 'Type', 'X', 'Y', 'Z', 'Km/h', 'Throttle', 'Steer', 'Brake']
        w = csv.DictWriter(f, fieldnames=fn)
        print('CSV file created.')
        w.writeheader()
    #f = open(dir + '/vehicle_data.csv','a+', newline='')

    rgbLeader.listen(lambda data: manage_vehicle(data, dir, PlatooningLeader))
    #rgbFollower.listen(lambda data: testLaneDet(data))
    #lidar.listen(lambda point_cloud: point_cloud.save_to_disk('recs/%.6d.ply' % point_cloud.frame))
    world.on_tick(lambda snap: get_vehicle_data(snap, f))

    while True:
        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:

    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')