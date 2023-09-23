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

######################################################################################
def obstacle_position_publisher(stopped_veh_position):    

    obs_pos_pub = rospy.Publisher("/obs_pos_topic", Pose, queue_size=10)

    rospy.init_node("obs_node")

    rate = rospy.Rate(20)

    # looped in main()    
    obs_pos = Pose()
    obs_pos.position.x = stopped_veh_position.x
    obs_pos.position.y = stopped_veh_position.y

    obs_pos_pub.publish(obs_pos)
    rospy.loginfo(obs_pos)

    rate.sleep()

def main():
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client('localhost', 2000)
        world = client.get_world()

        # Set up the simulator in synchronous mode
        # settings = world.get_settings()
        # settings.synchronous_mode = True # Enables synchronous mode
        # settings.fixed_delta_seconds = 0.05 # => 20 fps
        # world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        stopped_veh_bp = blueprint_library.filter('firetruck')[0]

        # stopped vehicle color
        if stopped_veh_bp.has_attribute('color'):
            stopped_veh_bp.set_attribute('color', '255,0,0')

        # choose a transform from the list of recommended spawn points of the map
        recommended_spawn_points = world.get_map().get_spawn_points()

        transform = recommended_spawn_points[45]
        transform.location += carla.Location(x=-30, y=-1.8)
        # transform.location += carla.Location(x=-40, y=-1.8)
        # transform.location += carla.Location(x=-50, y=-1.8)
        # transform.location += carla.Location(x=-60, y=-1.8)
        print('Spawned at %s' % transform)

        # Spawn the stopped_veh
        stopped_veh = world.spawn_actor(stopped_veh_bp, transform)
        print('Created %s' % stopped_veh.type_id)

        while not rospy.is_shutdown():
            # world.tick() # Ticked by Ego vehicle
            world.wait_for_tick()
            try:
                stopped_veh_position = stopped_veh.get_location()
                obstacle_position_publisher(stopped_veh_position)
            except rospy.ROSInterruptException:
                pass
    
    finally:
        print("destroying obstacle...")
        stopped_veh.destroy()
        print("done.")
######################################################################################

if __name__ == '__main__':
    
   main()