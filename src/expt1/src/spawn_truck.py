#!/usr/bin/env python

# Stopped vehicle script

# Author: Viranjan Bhattacharyya
# EMC2 Lab, Clemson University

##################################### Imports #######################################                                   

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

import rospy
from geometry_msgs.msg import Pose
######################################################################################

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set up the simulator in synchronous mode
# settings = world.get_settings()
# settings.synchronous_mode = True # Enables synchronous mode
# settings.fixed_delta_seconds = 0.05 # => 20 fps
# world.apply_settings(settings)

# The world contains the list blueprints that we can use for adding new
# actors into the simulation.
blueprint_library = world.get_blueprint_library()
stopped_veh_bp = blueprint_library.filter('firetruck')[0]

# stopped_veh color
if stopped_veh_bp.has_attribute('color'):
    stopped_veh_bp.set_attribute('color', '255,0,0')

# Choose a transform from the list of recommended spawn points of the map
recommended_spawn_points = world.get_map().get_spawn_points()

truck_transform = recommended_spawn_points[45]
truck_transform.location += carla.Location(x=-20)
print('Truck spawned at %s' % truck_transform)

# Spawn the stopped_veh
stopped_veh = world.spawn_actor(stopped_veh_bp, truck_transform)
print('Created %s' % stopped_veh.type_id)