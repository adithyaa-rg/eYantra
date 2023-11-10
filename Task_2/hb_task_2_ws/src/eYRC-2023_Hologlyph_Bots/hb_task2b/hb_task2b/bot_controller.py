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


# Team ID:		hb#2008
# Author List:		Adithyaa, Haricharan, Tripan, Leon
# Filename:		bot_controller.py
# Functions:	HBController, find_velocity, inverse_kinematics, goalCallBack_1, goalCallBack_2, goalCallBack_3, store_msg_1, store_msg_2, store_msg_3, goal_update, check_sign, line_intersect, check
# Nodes:		subscription_1, subscription_2, subscription_3, pose_subscriber_1, pose_subscriber_2, pose_subscriber_3, wheel_publishers

################### IMPORT MODULES #######################
import time
import rclpy
from rclpy.node import Node
# import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal        
import numpy as np
from geometry_msgs.msg  import Pose2D
from geometry_msgs.msg import Wrench     
import logging
import threading
import time


n = 3
flag = 0

class HBController(Node):
    def __init__(self):
        """
        Class constructor to set up the node

        Parameters:
            None

        Return:
            None
        """
        super().__init__('hb_controller')
        self.loopcount = 0
        self.lastindices = [0, 0, 0]
        self.flag_subscriber = 1
        # Initialise the required variables
        self.bot_x = [[], [], []]
        self.bot_y = [[], [], []]
        self.bot_theta = [[], [], []]
        self.index = np.array([0, 0, 0])

        self.goal = {
            'x_goal' : [250, 250, 250],
            'y_goal' : [250, 250, 250],
            'theta_goal' : [0, 0, 0]
        }


        self.messages = [Pose2D(), Pose2D(), Pose2D()]

        self.subscription_1 = self.create_subscription(
            Goal,
            'hb_bot_1/goal',  
            self.goalCallBack_1,  # Callback function to handle received messages
            1  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.subscription_1  # Prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Goal,  
            'hb_bot_2/goal',  
            self.goalCallBack_2,  # Callback function to handle received messages
            1  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.subscription_2  # Prevent unused variajble warning

        self.subscription_3 = self.create_subscription(
            Goal,  
            'hb_bot_3/goal',  
            self.goalCallBack_3,  # Callback function to handle received messages
            1  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.subscription_3  # Prevent unused v ariable warning


        pose_subscriber_1 = self.create_subscription(Pose2D,'/detected_aruco_1',self.store_msg_1,1)
        pose_subscriber_1  # prevent unused variable warning

        pose_subscriber_2 = self.create_subscription(Pose2D,'/detected_aruco_2',self.store_msg_2,1)
        pose_subscriber_2  # prevent unused variable warningcheck()

        pose_subscriber_3 = self.create_subscription(Pose2D,'/detected_aruco_3',self.store_msg_3,1)
        pose_subscriber_3  # prevent unused variable warning


        wheel_name = ['right', 'left', 'rear']
        self.wheel_publishers = [[self.create_publisher(Wrench, f'/hb_bot_{j+1}/{wheel_name[i]}_wheel_force', 10) for j in range(n)] for i in range(n)]

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        time.sleep(2)

    def find_velocity(self):
        """
        Function to find the velocity of the bots

        Parameters:
            None

        Returns:
            None
        """

        if (self.loopcount > 500):
            self.goal_update()
        
        x = (np.array([i.x for i in self.messages]) - 250)/25
        y = (np.array([i.y for i in self.messages]) - 250)/-25
        theta = -(np.array([i.theta for i in self.messages]) - np.pi/4)

        goal_x = (np.array(self.goal['x_goal']) - 250)/25
        goal_y = (np.array(self.goal['y_goal']) - 250)/-25
        goal_theta = np.array(self.goal['theta_goal'])

        # print(self.goal)
        self.kP_linear_x = 75
        self.kP_angular = 0
        self.kP_linear_y = 75

        self.global_pos_error_x = goal_x - x
        self.global_pos_error_y = goal_y - y
        self.theta_error = - (theta - goal_theta)

        # print(f"{self.global_pos_error_x=}")
        # print(f"{self.global_pos_error_y=}")
        # print(f"{self.theta_error=}")
        

        self.velocity_x = self.kP_linear_x*(self.global_pos_error_x*np.cos(theta) + self.global_pos_error_y*np.sin(theta))
        self.velocity_y = self.kP_linear_y*(-self.global_pos_error_x*np.sin(theta) + self.global_pos_error_y*np.cos(theta))
        self.velocity_theta = self.kP_angular*(self.theta_error)


        collision_time = 2e0 #collision time of 1 second assumed
        collision_line = np.array([[x[i],x[i]+self.velocity_x[i]*collision_time, y[i],y[i]+self.velocity_y[i]*collision_time] for i in range(3)])

        for i in range(3):
            for j in range(i+1,3):
                line1 = ((collision_line[i][0],collision_line[i][2]),
                         (collision_line[i][1],collision_line[i][3]))
                line2 = ((collision_line[j][0],collision_line[j][2]),
                         (collision_line[j][1],collision_line[j][3]))

                if self.line_intersect(line1, line2):
                    self.velocity_x[j] = 0
                    self.velocity_y[j] = 0

        self.check()

        self.alpha_1 = 30/180*np.pi
        self.alpha_2 = 150/180*np.pi
        self.alpha_3 = 270/180*np.pi

        self.global_velocity = np.array([self.velocity_x, self.velocity_y, self.velocity_theta])

        self.inverse_kinematics()


    def inverse_kinematics(self):
        """
        Function to find the velocity of the wheels using an inverse matrix

        Parameters:
            None

        Returns:
            None
        """
        component_matrix = [[np.cos(self.alpha_1 + np.pi/2), np.cos(self.alpha_2 + np.pi/2), np.cos(self.alpha_3 + np.pi/2)],
                            [np.sin(self.alpha_1 + np.pi/2), np.sin(self.alpha_2 + np.pi/2), np.sin(self.alpha_3 + np.pi/2)],
                            [1, 1, 1]]
        
        component_matrix = np.array(component_matrix)

        component_inverse = np.linalg.inv(component_matrix)

        wheel_velocity = np.matmul(component_inverse, self.global_velocity)


        wheel = ([[Wrench() for i in range(n)] for j in range(n)])
        for i in range(n):
            for j in range(n):
                wheel[i][j].force.y = wheel_velocity[i][j]


        for i in range(n):
            for j in range(n):
                self.wheel_publishers[i][j].publish(wheel[i][j])

    def goalCallBack_1(self, msg):
        """
        Callback function for subscriber to store goal

        Parameters:
            msg: Message received from the publisher

        Returns:
            None
        """
        if self.flag_subscriber:
            self.bot_x[0] = np.array(msg.x)
            self.bot_y[0] = np.array(msg.y)
            self.bot_theta[0] = msg.theta
            

    def goalCallBack_2(self, msg):
        """
        Callback function for subscriber to store goal

        Parameters:
            msg: Message received from the publisher

        Returns:
            None
        """
        if self.flag_subscriber:
            self.bot_x[1] = np.array(msg.x)
            self.bot_y[1] = np.array(msg.y)
            self.bot_theta[1] = msg.theta

    def goalCallBack_3(self, msg):
        """
        Callback function for subscriber to store goal

        Parameters:
            msg: Message received from the publisher

        Returns:
            None
        """
        if self.flag_subscriber:
            self.bot_x[2] = np.array(msg.x)
            self.bot_y[2] = np.array(msg.y)
            self.bot_theta[2] = msg.theta

        self.goal_update()

    def store_msg_1(self, msg):
        """
        Callback function for subscriber to store message

        Parameters:
            msg: Message received from the publisher

        Returns:
            None
        """
        self.messages[0] = msg

    def store_msg_2(self, msg):
        """
        Callback function for subscriber to store message

        Parameters:
            msg: Message received from the publisher

        Returns:
            None
        """
        self.messages[1] = msg

    def store_msg_3(self, msg):
        """
        Callback function for subscriber to store message

        Parameters:
            msg: Message received from the publisher

        Returns:
            None
        """
        self.messages[2] = msg

    def goal_update(self):
        """
        Function to update the goal

        Parameters:
            None

        Returns:
            None
        """
        self.flag_subscriber = 0    
        for i in range(0, n):

            self.goal["x_goal"][i] = self.bot_x[i][self.index[i]]
            self.goal["y_goal"][i] = self.bot_y[i][self.index[i]]
            self.goal["theta_goal"][i] = self.bot_theta[i]

    def check_sign(self, line_start, line_end, point):
        """
        Function to check the sign of the point

        Parameters:
            line_start: Starting point of the line
            line_end: Ending point of the line
            point: Point to check

        Returns:
            None
        """
        slope = (line_end[1] - line_start[1])/(line_end[0] - line_start[0])
        pt1_eqn = point[1] - line_start[1] - slope*(point[0])
        return pt1_eqn

    def line_intersect(self, seg1, seg2):
        """
        Function to check if the lines intersect

        Parameters:
            seg1: First segment
            seg2: Second segment

        Returns:
            None
        """
        if self.check_sign(seg1[0],seg1[1],seg2[0])*self.check_sign(seg1[0],seg1[1],seg2[1]) < 0:
            opp_sides_line1 = 1
        else:
            opp_sides_line1 = 0
        if self.check_sign(seg2[0],seg2[1],seg1[0])*self.check_sign(seg2[0],seg2[1],seg1[1]) < 0:
            opp_sides_line2 = 1
        else:
            opp_sides_line2 = 0
        return opp_sides_line2*opp_sides_line1

        
    def check (self):
        """
        Function to check if the bots have reached the goal

        Parameters:
            None

        Returns:
            None
        """
        self.l = len(self.bot_x[0])

        self.loopcount += 1        

        # print(self.loopcount, self.lastindices)

        for i in range(0, n):
            if np.linalg.norm(np.array([self.global_pos_error_x[i], self.global_pos_error_y[i]])) < 0.7: # and np.abs(self.theta_error[i]) < 0.2:

                if (self.loopcount > 50 + self.lastindices[i]):

                    # print(self.lastindices)
                    # print(self.loopcount)

                    self.index[i] += 1
                    self.index[i] = min(self.index[i], self.l - 1)

                    self.lastindices[i] = self.loopcount


                        


def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    rclpy.spin_once(hb_controller)
    # Main loop
    while rclpy.ok():
        # print(f'{hb_controller.index=}')

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
        hb_controller.find_velocity()
        # print(hb_controller.goal)

    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()