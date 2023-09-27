import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

# start = 0

class TurtleCircle(Node):

    def __init__(self):
        super().__init__('turtleCircle')
        self.radius = 1.0  # Adjust this radius to change the size of the circle.
        self.radius_change = False
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        self.publish_msg = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_velocity = Twist()
        timer_period = 0.5  # seconds
        self.start = 0
        self.timer = self.create_timer(timer_period, self.publish_new_velocity)
        


        

    def publish_new_velocity(self):
        # To make the turtle move in a circle, set the linear velocity and angular velocity appropriately.
        
        if self.start < 5:

            linear_speed = 0.6  # Adjust the linear speed as needed.
            angular_speed = linear_speed / self.radius
            self.current_velocity.linear.x = linear_speed
            self.current_velocity.linear.y = 0
            self.current_velocity.linear.z = 0
            self.current_velocity.angular.x = 0
            self.current_velocity.angular.y = 0
            self.current_velocity.angular.z = angular_speed
            self.publish_msg.publish(self.current_velocity)
            self.get_logger().info(f'{self.curPose.theta}')
        
    
    def listener_callback(self, msg):
        try:
            self.prevPose = self.curPose   
        except:
            self.prevPose = Pose()
            self.curPose = Pose()
        self.curPose = msg
        if (self.curPose.theta * self.prevPose.theta) < 0:
            self.start += 1

        if self.start == 2 and not self.radius_change :
            self.radius = -2*self.radius
            self.radius_change = True
            self.get_logger().info(f'I am changing radius')

        
        if self.start >= 5:
            self.stop_turtle()

    def stop_turtle(self):
        self.publish_msg.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    turtleCircle = TurtleCircle()
    rclpy.spin(turtleCircle)
    turtleCircle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()