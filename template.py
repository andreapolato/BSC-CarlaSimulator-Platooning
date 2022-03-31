import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    while True:
        world.wait_for_tick()

except KeyboardInterrupt:
    pass

finally:
    print('destroying actors')    
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    print('done.')
