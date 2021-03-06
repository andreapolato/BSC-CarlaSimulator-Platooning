import glob
import os
import sys

from threading import Thread
from time import sleep

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from platooning import Follower, Leader
from cloud import SafeCloud

actor_list = []
platoon_members = []
traffic = [None]*10

try:
    #***********************
    #---CONNECT TO SERVER---
    #***********************
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    
    world = client.get_world()
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.01
    settings.synchronous_mode = True
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()

    cloud = SafeCloud()
    
    #*******************
    #---TRASH CLEANUP---
    #*******************
    for v in world.get_actors():
        if isinstance(v, carla.Vehicle):
            v.destroy()

    
    #****************
    #---CARS SETUP---
    #****************
    spawn = carla.Transform(carla.Location(x=-25.407496, y=133.728470, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.647827, roll=0.000000))
    model3 = blueprint_library.filter('model3')[0]
   
   
    #*****************
    #---LIDAR SETUP---
    #*****************
    sensor_spawn = carla.Transform(carla.Location(x=2.5, z=0.8))
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('rotation_frequency','100')
    lidar_bp.set_attribute('horizontal_fov','45')
    lidar_bp.set_attribute('upper_fov','5')
    lidar_bp.set_attribute('lower_fov','-5')
    lidar_bp.set_attribute('range','20.0')

    
    #******************
    #---SPAWN LEADER---
    #******************
    PlatooningLeader = world.spawn_actor(model3, spawn)
    PlatooningLeader.set_autopilot(True)
    actor_list.append(PlatooningLeader)

    leader = Leader(PlatooningLeader)
    leader.connect_to_cloud(cloud)

    world.on_tick(lambda snap: Thread(leader.move()).start())


    #*********************************
    #---UNCOMMENT TO SPAWN OBSTACLE---
    #*********************************
    #spawn.location.x += 12
    #model3.set_attribute('color','0,0,0')
    #obstacle = world.spawn_actor(model3, spawn)
    #actor_list.append(obstacle)
    

    #********************
    #---SPAWN FOLLOWER---
    #********************
    spawn.location.x += 12
    model3.set_attribute('color','255,0,0')
    PlatooningFollower = world.spawn_actor(model3, spawn)
    actor_list.append(PlatooningFollower)
    LidarFollower = world.spawn_actor(lidar_bp, sensor_spawn, attach_to=PlatooningFollower)
    actor_list.append(LidarFollower)
    follower = Follower(PlatooningFollower)
    follower.connect_to_cloud(cloud)
    platoon_members.append(follower)
    world.on_tick(lambda snap: Thread(follower.move()).start())
    LidarFollower.listen(lambda points: follower.check_lidar(points))

    
    #*********************
    #---SPAWN FOLLOWER2---
    #*********************
    spawn.location.x += 12
    model3.set_attribute('color','0,255,0')
    PlatooningFollower2 = world.spawn_actor(model3, spawn)
    actor_list.append(PlatooningFollower2)
    LidarFollower2 = world.spawn_actor(lidar_bp, sensor_spawn, attach_to=PlatooningFollower2)
    actor_list.append(LidarFollower2)
    follower2 = Follower(PlatooningFollower2)
    follower2.connect_to_cloud(cloud)
    platoon_members.append(follower2)
    world.on_tick(lambda snap: Thread(follower2.move()).start())
    LidarFollower2.listen(lambda points: follower2.check_lidar(points))

    
    #*********************************
    #---POINT CAMERA TO SPAWN POINT---
    #*********************************
    trans = spawn
    trans.location.x -= 12
    trans.location.z = 70
    trans.rotation.pitch=-90
    trans.rotation.yaw=0
    trans.rotation.roll=0
    world.get_spectator().set_transform(trans)


    while True:
        world.tick()
        #**************************************
        #---UNCOMMENT TO SIMULATE THE ATTACK---
        #**************************************
        #print("ATTACK")
        #target = follower
        #target.x = target.y = target.z = target.yaw = random.randint(-100,100)
        #target.control()

except KeyboardInterrupt:
    pass

finally:
    print('destroying actors')    
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    print('done.')
