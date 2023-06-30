#!/usr/bin/env python

# Stopped vehicle script

# Author: Viranjan Bhattacharyya
# EMC2 Lab, Clemson University

#--------------------------------------- Imports -------------------------------------                                     

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
#-------------------------------------------------------------------------------------

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05 # => 20 fps
world.apply_settings(settings)

# The world contains the list blueprints that we can use for adding new
# actors into the simulation.
blueprint_library = world.get_blueprint_library()

vehicle_bp = blueprint_library.filter('firetruck')[0]

# Vehicle color
if vehicle_bp.has_attribute('color'):
    vehicle_bp.set_attribute('color', '255,0,0')

# Choose a transform from the list of recommended spawn points of the map
recommended_spawn_points = world.get_map().get_spawn_points()

transform = recommended_spawn_points[45]
transform.location += carla.Location(x=-20)
print('Spawned at %s' % transform)

# Spawn the vehicle
vehicle = world.spawn_actor(vehicle_bp, transform)
print('Created %s' % vehicle.type_id)