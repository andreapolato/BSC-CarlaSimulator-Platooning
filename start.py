from concurrent.futures import process
from dis import dis
from distutils.log import log
import glob
from multiprocessing.connection import wait
from ntpath import join
import os
from pyexpat import model
import re
import sys
from tabnanny import check
import threading
from turtle import speed
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
platoon_members = []
snap_list = []
F2Spawned = False
leader_pos_list = [[0,0],[0,0],[0,0],[0,0],[0,0]]
follower_pos_list = [[0,0],[0,0],[0,0],[0,0],[0,0]]
big_dist = []

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
    ya = str(transform.rotation.yaw)
    vel = vehicle_snap.get_velocity()
    speed = str('%15.2f'%(3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)))
    throttle = str(vehicle.get_control().throttle)
    steer = str(vehicle.get_control().steer)
    brake = str(vehicle.get_control().brake)
    if vehicle==PlatooningLeader:
        leader_pos_list.append([float(x),float(y)])
        leader_pos_list.pop(0)
    with open(dir + '/vehicle_data_%s.csv'%vehicle.id, 'a+', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fn)
        output = {'Snap':frame,'Time':time, 'ID':id, 'Type':type, 'X':x, 'Y':y, 'Z':z, 'Yaw':ya, 'Km/h':speed, 'Throttle':throttle, 'Steer':steer, 'Brake':brake}
        w.writerow(output)

def record_vehicle_data(snap):
    snap_list.append(snap)
    if len(snap_list)>1:
        snap_list.pop(0)
    for vehicle in actor_list:
        if isinstance(vehicle, carla.Vehicle):
            extract_data(snap,vehicle)
    
def leader_going_straight():
    res = True
    for i in range (4):
        if abs(leader_pos_list[i][0]-leader_pos_list[i+1][0])>0.01 or abs(leader_pos_list[i][1]-leader_pos_list[i+1][1])>0.01:
            res = False
    return res

def follower_going_straight():
    res = True
    for i in range (4):
        if abs(follower_pos_list[i][0]-follower_pos_list[i+1][0])>0.01 or abs(follower_pos_list[i][1]-follower_pos_list[i+1][1])>0.01:
            res = False
    return res

def sign(number):
  if number>=0: return 1
  else: return -1

def set_steer(fx,fy,yaw,fspeed):
 
    if fspeed<=0.03: return 0.0

    with open(dir + '/vehicle_data_%d.csv'%PlatooningLeader.id, newline='') as f:
        r = csv.DictReader(f, fieldnames=fn)
        rl = list(r)
        n=0
        limit=1000
        s=0.0
        delta_x = delta_y = 360
        best_x = best_y = rel_x = rel_y = 0
        for row in reversed(rl):
            if row!=rl[0]:
                lx = float(row['X'])
                ly = float(row['Y'])
                lyaw = float(row['Yaw'])
                toll = 0.01 if follower_going_straight() else 0.5
                if abs(lx-fx)<= toll and abs(ly-fy)<= toll:
                    s=float(row['Steer'])
                    if lyaw<-90 and yaw>90:
                        corr=(lyaw-yaw+360)/90
                    elif lyaw>90 and yaw<-90:
                        corr=(lyaw-yaw-360)/90
                    else:
                        corr=(lyaw-yaw)/90
                    if corr > 1.0: corr = 1.0
                    elif corr < -1.0: corr = -1.0
                    s+=corr
                    if s>1.0:
                        s=1.0
                    elif s<-1.0:
                        s=-1.0
                    return s
                
                else:
                    if abs(lx-fx) < delta_x:
                        rel_y = ly
                        delta_x = abs(lx-fx)
                        yaw_x = lyaw
                    if abs(ly-fy) < delta_y:
                        rel_x = lx
                        delta_y = abs(ly-fy)
                        yaw_y=lyaw
            
            n+=1
            if n>=limit: break
        
        lyaw = yaw_x if delta_x<delta_y else yaw_y
        #yaw_condition = sign(lyaw)==sign(yaw) or (abs(lyaw)>90 and abs(yaw)>90)
        #same_yaw = abs(lyaw-yaw)<3 if yaw_condition else abs(lyaw+yaw)<3
        
        corr = (lyaw-yaw)/90

        if abs(yaw)<=5: #going east
            s=(rel_y-fy)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
        elif abs(yaw)>=175: #going ovest
            if lyaw>90 and yaw<-90:
                corr-=4
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            elif lyaw<-90 and yaw>90:
                corr+=4
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            else:
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr

        elif yaw<95 and yaw>85: #going south
            s=(fx-rel_x)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
            
        elif yaw>-95 and yaw<-85: #going north
            s=(rel_x-fx)/10 
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr

        else:
            if yaw>-180 and yaw<=-90:
                l_pos = -rel_y if delta_x<delta_y else rel_x
                f_pos = fy if delta_x<delta_y else -fx
                
            elif yaw>-90 and yaw<=0:
                l_pos = rel_y if delta_x<delta_y else rel_x
                f_pos = -fy if delta_x<delta_y else -fx
            
            elif yaw>0 and yaw<=90:
                l_pos = rel_y if delta_x<delta_y else -rel_x
                f_pos = -fy if delta_x<delta_y else fx
            
            elif yaw>90 and yaw<=180:
                l_pos = -rel_y if delta_x<delta_y else -rel_x
                f_pos = fy if delta_x<delta_y else fx
            
            if sign(l_pos)==sign(f_pos): l_pos = -l_pos

            s = (l_pos+f_pos)/40
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0

            if lyaw>90 and yaw<-90:
                corr=(lyaw-yaw-360)/90
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            
            elif lyaw<-90 and yaw>90:
                corr=(lyaw-yaw+360)/90
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            
            else:
                corr=(lyaw-yaw)/90
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
        
        return s

def manage_follower(snap,vehicle, i):
    with open(dir + '/vehicle_data_%d.csv'%PlatooningLeader.id, newline='') as f:
        r = csv.DictReader(f, fieldnames=fn)
        vehicle_snap = snap.find(vehicle.id)
        transform = vehicle_snap.get_transform()
        yaw = float(transform.rotation.yaw)
        vel = vehicle_snap.get_velocity()
        fspeed = float('%15.2f'%(3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)))
        fx = float("{0:10.3f}".format(transform.location.x))
        fy = float("{0:10.3f}".format(transform.location.y))
        follower_pos_list.append([fx,fy])
        follower_pos_list.pop(0)
        t=s=b=0
        rl = list(r)
        for row in reversed(rl):
            if row!=rl[0]:
                if row['Snap'] == str(snap.frame):
                    lspeed = float(row['Km/h'])
                    delta = lspeed-fspeed
                    global big_dist
                    if lspeed>0.04:
                        s = set_steer(fx,fy,yaw,fspeed)
                        if (delta>0.03):
                            boost = 0.4 if big_dist[i] else 0.0
                            t = delta/10 + boost if delta/10 + boost <= 1.0 else 1.0
                        else:
                            t += 0.3
                            s = set_steer(fx,fy,yaw,fspeed)
                            if not big_dist[i]:
                                t = 0.0
                                b = -delta/10 if -delta/10 <= 1.0 else 1.0
                    else:
                        s = set_steer(fx,fy,yaw,fspeed)
                        if big_dist[i]:
                            t = 0.2
                        else:
                            b = 0.8
                    if s>0.005: t = t/2
                    vehicle.apply_control(carla.VehicleControl(throttle=t, steer=s, brake=b))

def check_distance():
    global big_dist
    for i in range(len(platoon_members)):
        if platoon_members[i]!=PlatooningLeader:
            follower_pos = platoon_members[i].get_transform()
            leader_pos = platoon_members[i-1].get_transform()
            dist = abs(((follower_pos.location.x-leader_pos.location.x)**2 + (follower_pos.location.y-leader_pos.location.y)**2)**(1/2))
            big_dist[i-1] = dist>12.0

def get_points(points):
    min = 500
    for p in points:
        if p.point.x<min:
            min = p.point.x
    global big_dist
    big_dist = True if min>=2 else False
            
def create_file(vehicle):
    with open(dir + '/vehicle_data_%d.csv'%vehicle.id, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fn)
        print('Leader CSV file created.')
        w.writeheader()
        
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    
    world = client.get_world()
    #world = client.load_world('Town06')
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.01
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()

    for v in world.get_actors():
        if isinstance(v, carla.Vehicle):
            v.destroy()

    dir = 'recs/' + time.strftime("%Y%m%d-%H%M%S")
    if not os.path.exists(dir):
        os.makedirs(dir)
    fn = ['Snap','Time', 'ID', 'Type', 'X', 'Y', 'Z', 'Yaw', 'Km/h', 'Throttle', 'Steer', 'Brake']
    
    world.on_tick(lambda snap: record_vehicle_data(snap))
    model3 = blueprint_library.filter('model3')[0]

    spawn = carla.Transform(carla.Location(x=-15.407496, y=133.728470, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.647827, roll=0.000000))
    
    PlatooningLeader = world.spawn_actor(model3, spawn)
    create_file(PlatooningLeader)
    PlatooningLeader.set_autopilot(True)
    actor_list.append(PlatooningLeader)
    platoon_members.append(PlatooningLeader)

    spawn.location.x += 12
    model3.set_attribute('color','255,0,0')
    PlatooningFollower = world.spawn_actor(model3, spawn)
    create_file(PlatooningFollower)
    big_dist.append(False)
    actor_list.append(PlatooningFollower)
    platoon_members.append(PlatooningFollower)
    
    spawn.location.x += 12
    model3.set_attribute('color','0,255,0')
    PlatooningFollower2 = world.spawn_actor(model3, spawn)
    create_file(PlatooningFollower2)
    big_dist.append(False)
    actor_list.append(PlatooningFollower2)
    platoon_members.append(PlatooningFollower2)

    sensorspawn = carla.Transform(carla.Location(x=2.5, z=0.8))
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('horizontal_fov','90')
    lidar_bp.set_attribute('upper_fov','5')
    lidar_bp.set_attribute('lower_fov','-5')
    LidarFollower = world.spawn_actor(lidar_bp, sensorspawn, attach_to=PlatooningFollower)
    actor_list.append(LidarFollower)

    lanecrossing_bp = blueprint_library.find('sensor.other.lane_invasion')
    LaneSensorFollower = world.spawn_actor(lanecrossing_bp, sensorspawn, attach_to=PlatooningFollower)
    actor_list.append(LaneSensorFollower)

    
    last_snap=0
    LidarFollower.listen(lambda points: get_points(points))
    LaneSensorFollower.listen(lambda event: print(event.transform))
    
    trans = PlatooningFollower.get_transform()
    trans.location.z = 100
    trans.rotation.pitch=-90
    trans.rotation.yaw=0
    trans.rotation.roll=0
    world.get_spectator().set_transform(trans)

    

    while True:
        dist_thread = threading.Thread(target=check_distance())
        dist_thread.start()

        if(snap_list):
            threading.Thread(target=manage_follower(snap_list[-1], PlatooningFollower, 0)).start()
            threading.Thread(target=manage_follower(snap_list[-1], PlatooningFollower2, 1)).start()
       
        world.wait_for_tick()
        
except KeyboardInterrupt:
    pass

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')