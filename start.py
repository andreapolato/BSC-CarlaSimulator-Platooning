import random
from time import sleep
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
    #world = client.load_world('Town06')
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.01
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()

    
    #*******************
    #---TRASH CLEANUP---
    #*******************
    for v in world.get_actors():
        if isinstance(v, carla.Vehicle):
            v.destroy()

    
    #****************
    #---CARS SETUP---
    #****************
    #spawn = random.choice(world.get_map().get_spawn_points())
    spawn = carla.Transform(carla.Location(x=25.407496, y=133.728470, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.647827, roll=0.000000))
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

    cloud = SafeCloud()

    
    #******************
    #---SPAWN LEADER---
    #******************
    PlatooningLeader = world.spawn_actor(model3, spawn)
    PlatooningLeader.set_autopilot(True)
    actor_list.append(PlatooningLeader)

    leader = Leader(PlatooningLeader)
    leader.connect_to_cloud(cloud)

    world.on_tick(lambda snap: leader.move())


    #********************
    #---OBSTACLE SPAWN---
    #********************
    #model3.set_attribute('color','0,0,0')
    #obstacle = world.spawn_actor(model3, spawn)
    #actor_list.append(obstacle)
    #spawn.location.x += 50
    

    #********************
    #---SPAWN FOLLOWER---
    #********************
    spawn.location.x += 12
    model3.set_attribute('color','255,0,0')
    PlatooningFollower = world.spawn_actor(model3, spawn)
    actor_list.append(PlatooningFollower)

    LidarFollower = world.spawn_actor(lidar_bp, sensor_spawn, attach_to=PlatooningFollower)
    actor_list.append(LidarFollower)

    follower = Follower(PlatooningFollower, leader)
    follower.connect_to_cloud(cloud)
    platoon_members.append(follower)
    world.on_tick(lambda snap: follower.move())
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
    
    follower2 = Follower(PlatooningFollower2, follower)
    follower2.connect_to_cloud(cloud)
    platoon_members.append(follower2)
    world.on_tick(lambda snap: follower2.move())
    LidarFollower2.listen(lambda points: follower2.check_lidar(points))


    #*********************************
    #---POINT CAMERA TO SPAWN POINT---
    #*********************************
    trans = spawn
    trans.location.z = 100
    trans.rotation.pitch=-90
    trans.rotation.yaw=0
    trans.rotation.roll=0
    world.get_spectator().set_transform(trans)


    #**********************
    #---GENERATE TRAFFIC---
    #**********************
    #for i in range (10):
    #    traffic_spawn = random.choice(world.get_map().get_spawn_points())
    #    model3.set_attribute('color','255,255,255')
    #    try:
    #        traffic[i] = world.spawn_actor(model3, traffic_spawn).set_autopilot(True)
    #        actor_list.append(traffic[i])
    #    except:
    #        i -= 1

    while True:
        world.wait_for_tick()
        if random.randint(0,99) == 0:
            print("ATTACK")
            target = random.choice(platoon_members)
            target.x = target.y = target.z = target.yaw = 0.0
            target.control()

except KeyboardInterrupt:
    pass

finally:
    print('destroying actors')    
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    print('done.')
