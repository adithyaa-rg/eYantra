#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
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
<<<<<<< HEAD
# import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal        
import numpy as np
from geometry_msgs.msg  import Pose2D
from geometry_msgs.msg import Wrench     
=======
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal             
>>>>>>> 41ed2e3e8d352b2aecc18c68bdfef0df411555b4


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
<<<<<<< HEAD
        self.goal = {'x_goal': [0.0, -5.0*25, 7.0*25], 'y_goal': [3.0*25, 5.0*25, 7.0*25], 'theta_goal': [0.0, 0.0, 0.0]}
        # Initialise the required variables

        self.bot_x = [[],[],[]]
        self.bot_y = [[],[],[]]
        self.bot_theta = [0.0, 0.0, 0.0]

        self.messages = [Pose2D(), Pose2D(), Pose2D()]

        self.subscription_1 = self.create_subscription(
            Goal,  
            'hb_bot_1/goal',  
            self.goalCallBack_1,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.subscription_1  # Prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Goal,  
            'hb_bot_2/goal',  
            lambda : self.goalCallBack_2,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.subscription_2  # Prevent unused variable warning

        self.subscription_3 = self.create_subscription(
            Goal,  
            'hb_bot_3/goal',  
            self.goalCallBack_3,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.subscription_3  # Prevent unused variable warning

        pose_subscriber_1 = self.create_subscription(Pose2D,'/detected_aruco_1',self.store_msg_1,10)
        pose_subscriber_1  # prevent unused variable warning

        pose_subscriber_2 = self.create_subscription(Pose2D,'/detected_aruco_2',self.store_msg_2,10)
        pose_subscriber_2  # prevent unused variable warning

        pose_subscriber_3 = self.create_subscription(Pose2D,'/detected_aruco_3',self.store_msg_3,10)
        pose_subscriber_3  # prevent unused variable warning


        wheel_name = ['left', 'right', 'rear']
        self.wheel_publishers = [[self.create_publisher(Wrench, f'/hb_bot_{i}/{wheel_name[j]}_wheel_force', 10) for j in range(3)] for i in range(3)]

=======

        # Initialise the required variables
        self.bot_1_x = []
        self.bot_1_y = []
        self.bot_1_theta = 0.0

        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.subscription = self.create_subscription(
            Goal,  
            'hb_bot_1/goal',  
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.subscription  # Prevent unused variable warning
>>>>>>> 41ed2e3e8d352b2aecc18c68bdfef0df411555b4

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

<<<<<<< HEAD
    def find_velocity(self):
        x = (np.array([i.x for i in self.messages]) - 250)/25
        y = (np.array([i.y for i in self.messages]) - 250)/-25
        theta = -(np.array([i.theta for i in self.messages]) - np.pi/4)

        print(f"x: {x}")
        print(f"y: {y}")
        print(f"theta: {theta}")

        goal_x = (np.array(self.goal['x_goal']))/25
        goal_y = (np.array(self.goal['y_goal']))/25
        goal_theta = np.array(self.goal['theta_goal'])

        self.kP_linear = 10
        self.kP_angular = 13

        self.global_pos_error_x = goal_x - x
        self.global_pos_error_y = goal_y - y
        self.theta_error = - (theta - goal_theta)

        self.velocity_x = self.kP_linear*(self.global_pos_error_x*np.cos(theta) + self.global_pos_error_y*np.sin(theta))
        self.velocity_y = self.kP_linear*(-self.global_pos_error_x*np.sin(theta) + self.global_pos_error_y*np.cos(theta))
        self.velocity_theta = self.kP_angular*(self.theta_error)

        self.alpha_1 = 30/180*np.pi
        self.alpha_2 = 150/180*np.pi
        self.alpha_3 = 270/180*np.pi

        self.global_velocity = np.array([self.velocity_x, self.velocity_y, self.velocity_theta])

        print(f'Global Velocity : {self.global_velocity}')

        self.inverse_kinematics()


    def inverse_kinematics(self):
=======
    def inverse_kinematics():
>>>>>>> 41ed2e3e8d352b2aecc18c68bdfef0df411555b4
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
<<<<<<< HEAD
        component_matrix = [[np.cos(self.alpha_1 + np.pi/2), np.cos(self.alpha_2 + np.pi/2), np.cos(self.alpha_3 + np.pi/2)],
                            [np.sin(self.alpha_1 + np.pi/2), np.sin(self.alpha_2 + np.pi/2), np.sin(self.alpha_3 + np.pi/2)],
                            [1, 1, 1]]
        
        component_matrix = np.array(component_matrix)

        component_inverse = np.linalg.inv(component_matrix)
        print(f"Component Inverse: {component_inverse}")

        wheel_velocity = np.matmul(component_inverse, self.global_velocity)
        print(f"Wheel Velocities: {wheel_velocity}")

        wheel = np.array([[Wrench() for i in range(3)] for j in range(3)])
        for i in range(3):
            for j in range(3):
                wheel[i][j].force.y = wheel_velocity.T[i][j]




    def goalCallBack_1(self, msg):
        self.bot_x[0] = np.array(msg.x)
        self.bot_y[0] = np.array(msg.y)
        self.bot_theta[0] = msg.theta

    def goalCallBack_2(self, msg):
        self.bot_x[1] = np.array(msg.x)
        self.bot_y[1] = np.array(msg.y)
        self.bot_theta[1] = msg.theta

    def goalCallBack_3(self, msg):
        self.bot_x[2] = np.array(msg.x)
        self.bot_y[2] = np.array(msg.y)
        self.bot_theta[2] = msg.theta

    def store_msg_1(self, msg):
        self.messages[0] = msg

    def store_msg_2(self, msg):
        self.messages[1] = msg

    def store_msg_3(self, msg):
        self.messages[2] = msg
=======
        pass

    def goalCallBack(self, msg):
        self.bot_1_x = msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta = msg.theta
>>>>>>> 41ed2e3e8d352b2aecc18c68bdfef0df411555b4

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
       
    # Main loop
    while rclpy.ok():

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
<<<<<<< HEAD
        hb_controller.find_velocity()
=======
>>>>>>> 41ed2e3e8d352b2aecc18c68bdfef0df411555b4
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
