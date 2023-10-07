import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
import numpy as np

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')
        
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)

        # x, y, theta
        self.cur_pose = np.array([0.0, 0.0, 0.0])
        self.goal_pose = np.array([2.0, 2.0, np.pi/2])

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.Kp_pos = 2.0
        self.Kp_angle = 4.0
        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        # and also Kp values for the P Controller


        # client for the "next_goal" service

        # TODO 
        # self.cli = self.create_client(NextGoal, 'next_goal')      
        # self.req = NextGoal.Request() 
        # self.index = 0

    def odom_callback(self,msg = Odometry()):
        pose1 = Odometry()
        pose1.pose.pose.orientation
    
        orientation = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        #self.get_logger().info(f"{orientation}")
        self.cur_pose[0] = msg.pose.pose.position.x
        self.cur_pose[1] = msg.pose.pose.position.y
        self.cur_pose[2] = orientation[2]
        self.get_logger().info(f'x = {self.cur_pose[0]}, y = {self.cur_pose[1]}, theta = {self.cur_pose[2]}')

    def controller(self):
        err = self.goal_pose - self.cur_pose
        pos_err = np.linalg.norm(err[0:2])
        angle_err = err[-1] 
        self.w = self.Kp_angle * angle_err 
        # Angular velocity used to reach goal orientation only. Indipendent of position/linear velocity

        v_global = self.Kp_pos * pos_err #magnitude of velocity based on error in position (euler distance)
        theta = np.arctan2(err[1],err[0]) #angle of vector from cur_position to goal_position
        rot_angle = self.cur_pose[-1] - theta # angle to rotate the global vector to the local frame
        self.get_logger().info(f"rot_angle = {rot_angle}")

        if rot_angle > np.pi:           # checks to make sure angle lies in (-pi, pi)
            rot_angle = np.pi - rot_angle
        elif rot_angle < -np.pi:
            rot_angle = -np.pi - rot_angle
        
        self.get_logger().info(f"v global = {v_global}, rot angle = {rot_angle}")
        
        v_local_resolved = v_global * np.array([np.cos(-rot_angle), np.sin(-rot_angle)])
        self.v_x = v_local_resolved[0]
        self.v_y = v_local_resolved[1]

        self.get_logger().info(f"v_x = {self.v_x}, v_y = {self.v_y}, w = {self.w}")

        publish_msg = Twist()
        publish_msg.linear.x = self.v_x
        publish_msg.linear.y = self.v_y
        publish_msg.angular.z = self.w
        self.publisher.publish(publish_msg)
        
        
def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()

    # Send an initial request with the index from ebot_controller.index
    # ebot_controller.send_request(ebot_controller.index)

    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        # if ebot_controller.future.done():
        try:
            # response from the service call
            # response = ebot_controller.future.result()
            pass
        except Exception as e:
            ebot_controller.get_logger().infselfo(
                'Service call failed %r' % (e,))
        else:
            #########           GOAL POSE             #########
            # x_goal      = response.x_goal
            # y_goal      = response.y_goal
            # theta_goal  = response.theta_goal
            # ebot_controller.flag = response.end_of_list

            ebot_controller.controller()
                ####################################################

                # Find error (in x, y and theta) in global frame
                # the /odom topic is giving pose of the robot in global frame
                # the desired pose is declared above and defined by you in global frame
                # therefore calculate error in global frame

                # (Calculate error in body frame)
                # But for Controller outputs robot velocity in robot_body frame, 
                # i.e. velocity are define is in x, y of the robot frame, 
                # Notice: the direction of z axis says the same in global and body frame
                # therefore the errors will have have to be calculated in body frame.
                # 
                # This is probably the crux of Task 1, figure this out and rest should be fine.

                # Finally implement a P controller 
                # to react to the error with velocities in x, y and theta.

                # Safety Check
                # make sure the velocities are within a range.
                # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
                # we may get away with skipping this step. But it will be very necessary in the long run.


                #If Condition is up to you
                
                ############     DO NOT MODIFY THIS       #########
                # ebot_controller.index += 1
                # if ebot_controller.flag == 1 :
                #     ebot_controller.index = 0
                # ebot_controller.send_request(ebot_controller.index)
                ####################################################

        #Spin once to process callbacks
        rclpy.spin_once(ebot_controller)

    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
