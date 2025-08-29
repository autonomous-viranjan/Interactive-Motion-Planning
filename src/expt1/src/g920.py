#!/usr/bin/python3
""" Viranjan Bhattacharyya """

import pygame
import rospy
from geometry_msgs.msg import Pose

class G920():
    def __init__(self):        
        
        self.u_steer = 0
        self.u_throttle = 0
        self.brake = 0

        # Wheel control
        pygame.init()
        pygame.joystick.init()
        self._wheel = pygame.joystick.Joystick(0)
        self._wheel.init()

        self.g920_pub = rospy.Publisher("/g920_topic", Pose, queue_size=10)
    
    def get_g920(self):
        # Get input from Logitech G920 Wheel
        self.u_steer = self._wheel.get_axis(0)
        self.u_throttle = -self._wheel.get_axis(1)
        self.brake = abs(1 - self._wheel.get_axis(2))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                playing = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    playing = False

    def run(self):
        # looped in main
        rospy.init_node("g920_node")
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            self.get_g920()            

            g920 = Pose()

            g920.position.x = self.u_steer
            g920.position.y = self.u_throttle
            g920.position.z = self.brake

            self.g920_pub.publish(g920)

            rospy.loginfo("steer: %f, throttle: %f, brake: %f", self.u_steer, self.u_throttle, self.brake)

            rate.sleep()

if __name__ == '__main__':

    g920 = G920()    
    g920.run()
