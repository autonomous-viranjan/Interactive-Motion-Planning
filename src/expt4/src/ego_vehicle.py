#!/usr/bin/env python

# Ego vehicle script

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

import pygame
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
#---------------------------------------------------------------------------------------

#=======================================================================================
#                                Low-level Controller                         
#=======================================================================================

class LowLevel:

    def __init__(self):

        self.lane_width = 3.6 # m
        self.dt = 0.05 # simulator delta time
        self.x0 = 85.235
        self.y0 = 13.415
        self.l0 = 1.0 # spawn lane

        self.e_a = 0.0 # for throttle PID
        self.e_l = 0.0 # for steering PID
        
        # Default vehicle state (sim frame)
        self.x = 0.01
        self.y = 0.01
        self.vx = 0.01
        self.vy = 0.01
        self.ax = 0.01
        self.ay = 0.01
        self.theta = np.arctan2(0.439, 112.71) # constant for straight road
        # self.theta_dot = 0.01 # not needed for straight road
        # Default highlevel states
        self.s = 0.1
        self.v = 0.1
        self.a = 0.1
        self.l = self.l0
        self.rl = 0.1
        # Default highlevel reference
        self.s_ref = 0
        self.v_ref = 2.5
        self.a_ref = 0
        self.l_ref = self.l0
        self.rl_ref = 0.1
        self.ua_ref = 0
        self.ul_ref = self.l0 

        # default NV loc
        self.nv_s = 150
        self.nv_l = 1

        self.s_obs = 150

        self.pos_pub = rospy.Publisher("/ego_pos_topic", Pose, queue_size=10)
        self.vel_pub = rospy.Publisher("/ego_vel_topic", Twist, queue_size=10)
        self.acc_pub = rospy.Publisher("/ego_acc_topic", Accel, queue_size=10)

        self.pos_sub = rospy.Subscriber("/plan_pos_topic", Pose, self.pos_callback)
        self.vel_sub = rospy.Subscriber("plan_vel_topic", Twist, self.vel_callback)
        self.acc_sub = rospy.Subscriber("/plan_acc_topic", Accel, self.acc_callback)
        self.ua_sub = rospy.Subscriber("/plan_ua_topic", Accel, self.ua_callback)
        self.ul_sub = rospy.Subscriber("/plan_ul_topic", Pose, self.ul_callback)

        self.NV_pos_sub = rospy.Subscriber("/NV_pos_topic", Pose, self.nv_pos_callback)
        self.obs_pos_sub = rospy.Subscriber("/obs_pos_topic", Pose, self.obs_pos_callback)
        
    def pos_callback(self, pos_msg):
        self.s_ref = pos_msg.position.x
        self.l_ref = pos_msg.position.y

    def vel_callback(self, vel_msg):
        self.v_ref = vel_msg.linear.x
        self.rl_ref = vel_msg.linear.y
        
    def acc_callback(self, acc_msg):
        self.a_ref = acc_msg.linear.x

    def ua_callback(self, ua_msg):
        self.ua_ref = ua_msg.linear.x

    def ul_callback(self, ul_msg):
        self.ul_ref = ul_msg.position.y

    def nv_pos_callback(self, nv_pos_msg):
        x_NV = nv_pos_msg.position.x
        y_NV = nv_pos_msg.position.y
        self.nv_s = -((x_NV - 85.235) * np.cos(0.003895) + (y_NV - 13.415) * np.sin(0.003895))
        self.nv_l = (-(x_NV - 85.235) * np.sin(0.003895) + (y_NV - 13.415) * np.cos(0.003895)) / self.lane_width + 1

    def obs_pos_callback(self, obs_pos_msg):
        x_obs = obs_pos_msg.position.x
        y_obs = obs_pos_msg.position.y
        self.s_obs =  -((x_obs - 85.235) * np.cos(0.003895) + (y_obs - 13.415) * np.sin(0.003895))     
    
    def sim2hl(self):
        """ 
            This function transforms *current* states from 
            CARLA (sim) frame to High-level (hl) frame 
        """ 
        self.s = -((self.x - self.x0) * np.cos(self.theta) + (self.y - self.y0) * np.sin(self.theta))
        self.l = (-(self.x - self.x0) * np.sin(self.theta) + (self.y - self.y0) * np.cos(self.theta)) / self.lane_width + 1
        self.v = -(self.vx * np.cos(self.theta) + self.vy * np.sin(self.theta))
        self.a = -(self.ax * np.cos(self.theta) + self.ay * np.sin(self.theta))
        self.rl = self.vx * np.sin(self.theta) - self.vy * np.sin(self.theta)

    def controller(self, vehicle):
        """
            Input: vehicle states (sim frame)
            Output: vehicle throttle and steer
        """
        # from simulator frame
        vehicle_transform = vehicle.get_transform()
        vehicle_velocity = vehicle.get_velocity()
        vehicle_accleration = vehicle.get_acceleration()            
        vehicle_angular_vel = vehicle.get_angular_velocity()
        
        # Set state of vehicle from simualtor data
        self.x = vehicle_transform.location.x
        self.y = vehicle_transform.location.y
        self.vx = vehicle_velocity.x
        self.vy = vehicle_velocity.y
        self.ax = vehicle_accleration.x
        self.ay = vehicle_accleration.y
        # self.theta = vehicle_transform.rotation.yaw * 2 * np.pi / 360 # convert to  radians
        self.theta_dot = vehicle_angular_vel.z * 2 * np.pi / 360      

        # Convert to high-level frame
        self.sim2hl()

        # Controller logic
        ## when planner not running cruise at const speed
        if self.ua_ref == 0:
            u_a = (self.v_ref - self.v)
        else:
            u_a = (self.ua_ref - self.a) + 0.02 * (self.e_a + (self.ua_ref - self.a) * self.dt) + 0.1 * ((self.ua_ref - self.a) - self.e_a) / self.dt
        
        u_steer = 0.25 * (self.l - self.ul_ref) + 0.25 * (self.e_l + (self.l - self.ul_ref) * self.dt) + 1.0 * ((self.l - self.ul_ref) - self.e_l) / self.dt

        self.e_a = (self.ua_ref - self.a)
        self.e_l = (self.l - self.ul_ref)

        return u_a, u_steer

    def ego_data(self):        
        return self.s, self.l, self.s_ref, self.l_ref
    
    def nv_loc(self):
        return self.nv_s, self.nv_l
    
    def obs_loc(self):
        return self.s_obs
    
    def run(self):
        # looped in game_loop()
        rospy.init_node("Low_level_node")
        rate = rospy.Rate(20) # Hz
        
        pos = Pose()
        vel = Twist()
        acc = Accel()

        pos.position.x = self.s
        pos.position.y = self.l
        vel.linear.x = self.v
        vel.linear.y = self.rl           
        acc.linear.x = self.a        

        self.pos_pub.publish(pos)
        self.vel_pub.publish(vel)
        self.acc_pub.publish(acc)

        rospy.loginfo("s: %f, v: %f, a: %f, l: %f", self.s, self.v, self.a, self.l)
        rospy.loginfo("x: %f, y: %f, vx: %f, vy: %f", self.x,  self.y, self.vx, self.vy)
        rospy.loginfo("s_ref: %f, v_ref: %f, a_ref: %f, l_ref: %f", self.s_ref, self.v_ref, self.a_ref, self.l_ref)
        # rospy.loginfo("Theta: %f", self.theta)
        rospy.loginfo("ua_ref: %f, ul_ref: %f", self.ua_ref, self.ul_ref)        

        rate.sleep()
    

#==================================================================================================
#                                      Render object
#==================================================================================================

# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

#===================================================================================================
#                                    Ego vehicle setup 
#===================================================================================================

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05 # => 20 fps
world.apply_settings(settings)

# Also set up the spectator so we can see what we do
spectator = world.get_spectator()

# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))

# The world contains the list blueprints that we can use for adding new
# actors into the simulation.
blueprint_library = world.get_blueprint_library()

vehicle_bp = blueprint_library.filter('model3')[0]

# Blue Ego Vehicle
if vehicle_bp.has_attribute('color'):
    vehicle_bp.set_attribute('color', '0,0,255')

# Choose a transform from the list of recommended spawn points of the map
recommended_spawn_points = world.get_map().get_spawn_points()

transform = recommended_spawn_points[45]
# transform.location += carla.Location(x=15, y=3.7)
# transform.location += carla.Location(x=20, y=0)
transform.location += carla.Location(x=23, y=0)
print('Spawned at %s' % transform)

# Spawn the vehicle
vehicle = world.spawn_actor(vehicle_bp, transform)
print('Created %s' % vehicle.type_id)
# left indicator
vehicle.set_light_state(carla.VehicleLightState.LeftBlinker)

# Add some physics
front_left_wheel  = carla.WheelPhysicsControl(tire_friction=2.0, damping_rate=1.5, max_steer_angle=40.0, long_stiff_value=1000)
front_right_wheel = carla.WheelPhysicsControl(tire_friction=2.0, damping_rate=1.5, max_steer_angle=40.0, long_stiff_value=1000)
rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=3.0, damping_rate=1.5, max_steer_angle=0.0,  long_stiff_value=1000)
rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=3.0, damping_rate=1.5, max_steer_angle=0.0,  long_stiff_value=1000)
wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
try:
    physics_control = vehicle.get_physics_control()
    physics_control = vehicle.get_physics_control()
    physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
    physics_control.max_rpm = 3000
    physics_control.moi = 1.0
    physics_control.damping_rate_full_throttle = 1.0
    physics_control.mass = 1800
    physics_control.drag_coefficient = 0.1
    physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
    physics_control.use_sweep_wheel_collision = True
    physics_control.wheels = wheels
    vehicle.apply_physics_control(physics_control)
    vehicle.apply_physics_control(physics_control)
except Exception:
    pass

# Initialize the camera floating behind the vehicle
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Start camera with PyGame callback
camera.listen(lambda image: pygame_callback(image, render_object))

# Get camera dimensions
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Instantiate objects for render
render_object = RenderObject(image_w, image_h)

# CARLA vehicle control object
vehicle_control = carla.VehicleControl()

# Low-level controller object
low_level = LowLevel()
#====================================================================================================
#                                             Game loop
#====================================================================================================
def game_loop():
    try:
        # Initialize the display
        pygame.init()
        gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
        # Draw black to the display
        gameDisplay.fill((0,0,0))
        gameDisplay.blit(render_object.surface, (0,0))
        pygame.display.flip()

        # To run plot GUI event loop
        plt.ion()

        # Data to plot
        s, l, s_ref, l_ref = low_level.ego_data()
        s_nv, l_nv = low_level.nv_loc()
        s_obs = low_level.obs_loc()

        # Plot settings
        figure = plt.figure(figsize=(2, 10))
        plt.xlim(2.5, 0.5)
        # plt.ylim(-10, 120)
        ln0, = plt.plot([1.5, 1.5], [0, 140], '--k')
        ego_point, = plt.plot(l, s, 'sb')
        ref_point, = plt.plot(l_ref, s_ref, 'Pg')
        nv_point, = plt.plot(l_nv, s_nv, 'sr')
        obs_point, = plt.plot(1, s_obs, 'Xr')
        plt.title("Tracking")
        plt.xlabel("Lane")
        plt.ylabel("Road length [m]")
        plt.legend([ego_point, ref_point, nv_point], ['Ego', 'Ref', 'NV'])        

        end = False
        t = 0
        u_a_prev = 0
        u_throttle_prev = 0
        u_brake_prev = 0
        dt_sim = 0.05
        while not end:
            # Advance the simulation time
            world.tick()
            t += 1
            print(t)
            # Update the display
            gameDisplay.blit(render_object.surface, (0,0))
            pygame.display.flip() 
            
            # Collect key press events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    # If user presses Esc, break the while loop
                    if event.key == pygame.K_ESCAPE:
                        end = True

            if t == 100:
                # Set traffic light to green
                for tl in world.get_actors().filter('traffic.*'):
                    if isinstance(tl, carla.TrafficLight):
                        tl.set_state(carla.TrafficLightState.Green)
                        tl.freeze(True)                      
            # if t == 650:
            #     # set traffic light to red            
            #     for tl in world.get_actors().filter('traffic.*'):
            #         if isinstance(tl, carla.TrafficLight):
            #             tl.set_state(carla.TrafficLightState.Red)
            #             tl.freeze(True)  
            
            """ ------- Vehicle control ---------- """

            u_a, u_steer = low_level.controller(vehicle)            
            # print("throttle: ", u_a)
            # print("steer: ", u_steer)

            if u_a >= 0:
                # vehicle_control.throttle = u_a
                u_throttle = u_throttle_prev + ((-1/0.1)*u_throttle_prev + (1/0.1)*(u_a + (u_a - u_a_prev)/dt_sim))*dt_sim
                vehicle_control.throttle = u_throttle
                u_a_prev = u_a
                u_throttle_prev = u_throttle
            else:
                vehicle_control.throttle = 0.5
                # vehicle_control.throttle = 0
            
            if u_a < 0:
                vehicle_control.brake = u_a
                # u_brake = u_brake_prev + ((-1/0.1)*u_brake_prev + (1/0.1)*(u_a))*dt_sim
                # vehicle_control.brake = u_brake
                # u_brake_prev = u_a

            else:
                vehicle_control.brake = 0
            
            vehicle_control.steer = u_steer

            # Stop vehicle after this many time steps           
            if t > 900:
                vehicle_control.throttle = 0
                vehicle_control.brake = 1

             # Apply the control to the car
            vehicle.apply_control(vehicle_control)       

            low_level.run()            
            """ ----------------------------------- """

            # Updating data values
            s, l, s_ref, l_ref = low_level.ego_data()
            s_nv, l_nv = low_level.nv_loc()
            s_obs = low_level.obs_loc()
            ego_point.set_data(l, s)
            ref_point.set_data(l_ref, s_ref)
            nv_point.set_data(l_nv, s_nv)
            obs_point.set_data(1, s_obs)

            figure.gca().relim()
            figure.gca().autoscale_view(False, True) # rescale only y-axis         
            # Drawing updated values
            figure.canvas.draw()        
            # This will run the GUI event
            figure.canvas.flush_events()           

    finally:
        # Stop camera and quit PyGame after exiting game loop
        camera.stop()
        print('destroying automated vehicle...')
        vehicle.destroy() 
        camera.destroy()      
        print('Done.')
        pygame.quit()


if __name__ == '__main__':    
    
    game_loop()