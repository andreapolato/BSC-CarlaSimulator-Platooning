from time import sleep
import carla
import random
from leader import Follower, Leader

actor_list = []

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    
    world = client.get_world()
    #world = client.load_world('Town01')
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.01
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()

    for v in world.get_actors():
        if isinstance(v, carla.Vehicle):
            v.destroy()

    model3 = blueprint_library.filter('model3')[0]

    #spawn = random.choice(world.get_map().get_spawn_points())
    spawn = carla.Transform(carla.Location(x=-15.407496, y=133.728470, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.647827, roll=0.000000))


    PlatooningLeader = world.spawn_actor(model3, spawn)
    PlatooningLeader.set_autopilot(True)
    actor_list.append(PlatooningLeader)
    PlatooningLeader.set_autopilot()

    leader = Leader(PlatooningLeader)

    world.on_tick(lambda snap: leader.move())

    #spawn.location.x += 12
    sleep(2)
    model3.set_attribute('color','255,0,0')
    PlatooningFollower = world.spawn_actor(model3, spawn)
    actor_list.append(PlatooningFollower)
    follower = Follower(PlatooningFollower, leader)
    leader.addFollower(follower)
    world.on_tick(lambda snap: follower.move())

    #sleep(2)
    #spawn.location.x += 12
    #model3.set_attribute('color','255,0,0')
    #PlatooningFollower2 = world.spawn_actor(model3, spawn)
    #actor_list.append(PlatooningFollower2)
    #follower2 = Follower(PlatooningFollower2, follower)
    #leader.addFollower(follower2)
    #world.on_tick(lambda snap: follower2.move())

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
