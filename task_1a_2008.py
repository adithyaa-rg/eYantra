import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np

class TurtleControlNode(Node):
    def __init__(self):
        super().__init__()
        self.turtlename = self.declare_parameter('turtlename', 'turtle').get_parameter_value().string_value
        self.publisher = self.create_publisher(Twist, f'{self.turtlename}/cmd_vel',10)
        self.subscription = self.create_subscription(Pose, f'{self.turtlename}/pose', self.handle_turtle_pose,10)
        self.subscription
        self.initial_pose_recieved = False
        self.current_pose = Pose()

    def handle_turtle_pose(self,msg):
        self.current_pose = msg
        self.initial_pose_recieved = True
    
    def draw_circle(self, radius, speed):
        if self.initial_pose_recieved:
            self.get_logger().info("Drawing Circle 1")
            angular_speed = speed/radius
            theta_0 = self.current_pose.theta
            theta = 0.0
            rate = self.create_rate(10) #rate at which the controller runs

            while theta < 2*np.pi:
                twist = Twist()
                twist.linear.x = speed
                twist.angular.z = angular_speed
                self.publisher.publish(twist)
                theta = self.current_pose.theta - theta_0
                rate.sleep()
            
            self.get_logger().info("Circle 1 Drawn!")
            self.publisher.publish(Twist()) #publish empty Twist message to stop the turtle
        else:
            self.get_logger().info("Waiting for initial pose...")
            rclpy.spin_once(self)


def main(args = None):
    rclpy.init(args=args)
    turtle_control = TurtleControlNode()
    turtle_control.draw_circle(radius = 1.0, speed = 2.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
