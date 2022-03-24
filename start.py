import random
from time import sleep
import carla
from tester import Follower, Leader
#from platooning import Follower, Leader

actor_list = []

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

    model3 = blueprint_library.filter('model3')[0]

    #spawn = random.choice(world.get_map().get_spawn_points())
    spawn = carla.Transform(carla.Location(x=5.407496, y=133.728470, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.647827, roll=0.000000))

    sensor_spawn = carla.Transform(carla.Location(x=2.5, z=0.8))
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('horizontal_fov','45')
    lidar_bp.set_attribute('upper_fov','5')
    lidar_bp.set_attribute('lower_fov','-5')
    lidar_bp.set_attribute('range','20.0')


    PlatooningLeader = world.spawn_actor(model3, spawn)
    PlatooningLeader.set_autopilot(True)
    actor_list.append(PlatooningLeader)
    PlatooningLeader.set_autopilot()

    leader = Leader(PlatooningLeader)

    world.on_tick(lambda snap: leader.move())

    spawn.location.x += 12
    spawn.location.y -= 6
    #sleep(2)
    model3.set_attribute('color','255,0,0')
    #PlatooningFollower = world.spawn_actor(model3, spawn)
    #actor_list.append(PlatooningFollower)
    
    #LidarFollower = world.spawn_actor(lidar_bp, sensor_spawn, attach_to=PlatooningFollower)
    #actor_list.append(LidarFollower)

    #follower = Follower(PlatooningFollower, leader)
    #leader.addFollower(follower)
    #world.on_tick(lambda snap: follower.move())
    #LidarFollower.listen(lambda points: follower.checkLidar(points))

    spawn.location.x += 12
    spawn.location.y += 12
    #sleep(2)
    model3.set_attribute('color','0,255,0')
    PlatooningFollower2 = world.spawn_actor(model3, spawn)
    actor_list.append(PlatooningFollower2)
    
    LidarFollower2 = world.spawn_actor(lidar_bp, sensor_spawn, attach_to=PlatooningFollower2)
    actor_list.append(LidarFollower2)
    
    follower2 = Follower(PlatooningFollower2, leader)
    leader.addFollower(follower2)
    world.on_tick(lambda snap: follower2.move())
    #LidarFollower2.listen(lambda points: follower2.checkLidar(points))


    trans = spawn
    trans.location.z = 100
    trans.rotation.pitch=-90
    trans.rotation.yaw=0
    trans.rotation.roll=0
    world.get_spectator().set_transform(trans)

    while True:
        world.wait_for_tick()

except KeyboardInterrupt:
    pass

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
