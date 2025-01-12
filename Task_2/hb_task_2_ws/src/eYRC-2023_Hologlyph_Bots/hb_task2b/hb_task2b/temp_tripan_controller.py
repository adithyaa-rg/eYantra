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
# import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal        
import numpy as np
from geometry_msgs.msg  import Pose2D
from geometry_msgs.msg import Wrench     


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
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

        print(self.wheel_publishers)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    def find_velocity(self):
        x = (np.array([i.x for i in self.messages]) - 250)/25
        y = (np.array([i.y for i in self.messages]) - 250)/-25
        theta = -(np.array([i.theta for i in self.messages]) - np.pi/4)


        goal_x = (self.goal['x_goal'])/25
        goal_y = (self.goal['y_goal'])/25
        goal_theta = self.goal['theta_goal']

        self.kP_linear = 100
        self.kP_angular = 130

        self.global_pos_error_x = goal_x - x
        self.global_pos_error_y = goal_y - y
        self.theta_error = - (theta - goal_theta)

        self.velocity_x = self.kP_linear*(self.global_pos_error_x*np.cos(theta) + self.global_pos_error_y*np.sin(theta))
        self.velocity_y = self.kP_linear*(-self.global_pos_error_x*np.sin(theta) + self.global_pos_error_y*np.cos(theta))
        self.velocity_theta = self.kP_angular*(self.theta_error)

        collision_time = 1 #collision time of 1 second assumed
        collision_line = np.array([[x[i],x[i]+self.velocity_x[i]*collision_time, y[i],y[i]+self.velocity_y[i]*collision_time] for i in range(3)])
        for i in range(3):
            for j in range(i+1,3):
                line1 = ((collision_line[i][0],collision_line[i][3]),
                         (collision_line[i][1],collision_line[i][4]))
                line2 = ((collision_line[j][0],collision_line[j][3]),
                         (collision_line[j][1],collision_line[j][4]))
                if line_intersect(line1,line2):
                    self.velocity_x[j] = 0
                    self.velocity_y[j] = 0

        self.alpha_1 = 30/180*np.pi
        self.alpha_2 = 150/180*np.pi
        self.alpha_3 = 270/180*np.pi

        self.global_velocity = np.array([self.velocity_x, self.velocity_y, self.velocity_theta])

        self.inverse_kinematics()


    def inverse_kinematics(self):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        component_matrix = [[np.cos(self.alpha_1 + np.pi/2), np.cos(self.alpha_2 + np.pi/2), np.cos(self.alpha_3 + np.pi/2)],
                            [np.sin(self.alpha_1 + np.pi/2), np.sin(self.alpha_2 + np.pi/2), np.sin(self.alpha_3 + np.pi/2)],
                            [1, 1, 1]]
        
        component_matrix = np.array(component_matrix)

        component_inverse = np.linalg.inv(component_matrix)

        wheel_velocity = np.matmul(component_inverse, self.global_velocity)

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

def check_sign(line_start, line_end, point):
    slope = (line_end[1] - line_start[1])/(line_end[0] - line_start[0])
    # y - y1 - m(x-x1)
    pt1_eqn = point[1] - line_start[1] - slope*(point[0])
    return pt1_eqn

def line_intersect(seg1, seg2):
    if check_sign(seg1[0],seg1[1],seg2[0])*check_sign(seg1[0],seg1[1],seg2[1]):
        opp_sides_line1 = 1
    else:
        opp_sides_line1 = 0
    if check_sign(seg2[0],seg2[1],seg1[0])*check_sign(seg2[0],seg2[1],seg1[1]):
        opp_sides_line2 = 1
    else:
        opp_sides_line2 = 0
    return opp_sides_line2*opp_sides_line1

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    
    # Main loop
    while rclpy.ok():

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()