#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal             
from geometry_msgs.msg  import Pose2D
from geometry_msgs.msg import Wrench

# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        self.goal = {}
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force

        self.message = Pose2D()
        pose_subscriber = self.create_subscription(Pose2D,'/detected_aruco',self.store_msg,10)
        pose_subscriber  # prevent unused variable warning

        self.wheel_left = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.wheel_right = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.wheel_rear = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        

    def send_request(self, index):

        """
        Sends the request to the "next_goal" service

        Parameters:
            index: Index of the goal pose in the list of goal poses

        Returns:
            None

        """

        self.req.request_goal = index
        self.future = self.cli.call_async(self.req)
        # self.get_logger().info(f"Request sent: {self.req.request_goal}")
        rclpy.spin_until_future_complete(self, self.future)
        # self.get_logger().info(f"Futrue complete")

        return self.future.result()
    
    def store_msg(self,msg):
        self.message = msg

    def find_velocity(self):
        x = (self.message.x - 250)/25
        y = (self.message.y - 250)/-25
        theta = -(self.message.theta - np.pi/4)

        goal_x = (self.goal['x_goal'])/25
        goal_y = (self.goal['y_goal'])/25
        goal_theta = self.goal['theta_goal']

        self.kP_linear = 45
        self.kP_angular = 30

        self.global_pos_error_x = goal_x - x
        self.global_pos_error_y = goal_y - y
        self.theta_error = goal_theta - theta

        self.velocity_x = self.kP_linear*(self.global_pos_error_x*np.cos(theta) + self.global_pos_error_y*np.sin(theta))
        self.velocity_y = self.kP_linear*(-self.global_pos_error_x*np.sin(theta) + self.global_pos_error_y*np.cos(theta))
        self.velocity_theta = self.kP_angular*(self.theta_error)

        self.alpha_1 = 30/180*np.pi
        self.alpha_2 = 150/180*np.pi
        self.alpha_3 = 270/180*np.pi

        self.global_velocity = np.array([self.velocity_x, self.velocity_y, self.velocity_theta])

        self.inverse_kinematics()

    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        

    def inverse_kinematics(self):

        component_matrix = [[np.cos(self.alpha_1 + np.pi/2), np.cos(self.alpha_2 + np.pi/2), np.cos(self.alpha_3 + np.pi/2)],
                            [np.sin(self.alpha_1 + np.pi/2), np.sin(self.alpha_2 + np.pi/2), np.sin(self.alpha_3 + np.pi/2)],
                            [1, 1, 1]]
        
        component_matrix = np.array(component_matrix)

        component_inverse = np.linalg.inv(component_matrix)

        wheel_velocity = np.matmul(component_inverse, self.global_velocity)

        wheel_1 = Wrench()
        wheel_1.force.y = wheel_velocity[0]
        wheel_2 = Wrench()
        wheel_2.force.y = wheel_velocity[1]
        wheel_3 = Wrench()
        wheel_3.force.y = wheel_velocity[2]

        self.wheel_right.publish(wheel_1)
        self.wheel_left.publish(wheel_2)
        self.wheel_rear.publish(wheel_3)




def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from ebot_controller.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                hb_controller.goal['x_goal']      = response.x_goal
                hb_controller.goal['y_goal']      = response.y_goal
                hb_controller.goal['theta_goal']  = response.theta_goal
                hb_controller.flag = response.end_of_list

                hb_controller.find_velocity()
                ####################################################
                
                # Calculate Error from feedback

                # Change the frame by using Rotation Matrix (If you find it required)

                # Calculate the required velocity of bot for the next iteration(s)
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)

                # Apply appropriate force vectors

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                
                ############     DO NOT MODIFY THIS       #########
                if np.linalg.norm(np.array([hb_controller.global_pos_error_x, hb_controller.global_pos_error_y])) < 0.2 and np.abs(hb_controller.theta_error) < 0.1:
                    hb_controller.index += 1
                    if hb_controller.flag == 1 :
                        hb_controller.index = 0
                    hb_controller.send_request(hb_controller.index)
                ###################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
