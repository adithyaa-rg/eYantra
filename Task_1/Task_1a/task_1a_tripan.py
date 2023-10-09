# Adding necessary imports
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np

# Creating a class called TurtleControlNode which is a subclass of Node
class TurtleControlNode(Node):
    
    """
    This class is a subclass of Node class and is used to control the turtle's motion.
    It has a constructor which initializes the node, creates a publisher and a subscriber.
    It also has a method called draw_circle which makes the turtle draw a circle of given radius and speed.

    Attributes:
        turtlename: Name of the turtle
        publisher: Publisher object which publishes the Twist message
        subscription: Subscription object which subscribes to the Pose message
        initial_pose_recieved: Boolean variable which is set to True when the turtle's initial pose is recieved
        current_pose: Pose message which stores the turtle's current pose

    Methods:
        handle_turtle_pose: Callback function for the subscriber
        draw_circle: Makes the turtle draw a circle of given radius and speed

    """

    def __init__(self):

        """
        The constructor for TurtleControlNode class.
        It initializes the node, creates a publisher and a subscriber.
        
        Parameters:
            None   
        
        Returns:
            None

        """

        # Initializing attributes
        super().__init__()
        self.turtlename = self.declare_parameter('turtlename', 'turtle').get_parameter_value().string_value
        self.publisher = self.create_publisher(Twist, f'{self.turtlename}/cmd_vel',10)
        self.subscription = self.create_subscription(Pose, f'{self.turtlename}/pose', self.handle_turtle_pose,10)
        self.initial_pose_recieved = False
        self.current_pose = Pose()

    def handle_turtle_pose(self,msg):
            
        """
        Callback function for the subscriber.
        It is called whenever a Pose message is recieved.

        Parameters:
            msg: Pose message recieved from the subscriber

        Returns:
            None

        """

        self.current_pose = msg
        self.initial_pose_recieved = True
    
    def draw_circle(self, radius, speed):

        """
        Makes the turtle draw a circle of given radius and speed.

        Parameters:
            radius: Radius of the circle
            speed: Speed of the turtle

        Returns:
            None

        """

        if self.initial_pose_recieved:
            # Calculating the angular speed
            self.get_logger().info("Drawing Circle 1")
            angular_speed = speed/radius

            # Initializing the current angle and the angle at the start of the circle
            theta_0 = self.current_pose.theta
            theta = 0.0

            # Rate at which the controller runs
            rate = self.create_rate(10)

            # Looping until the turtle completes one circle
            while theta < 2*np.pi:

                # Creating a Twist message and publishing it
                twist = Twist()
                twist.linear.x = speed
                twist.angular.z = angular_speed
                self.publisher.publish(twist)
                theta = self.current_pose.theta - theta_0
                rate.sleep()

            # Stopping the turtle            
            self.get_logger().info("Circle 1 Drawn!")
            self.publisher.publish(Twist()) #publish empty Twist message to stop the turtle
        else:
            
            # Waiting for the turtle's initial pose
            self.get_logger().info("Waiting for initial pose...")
            rclpy.spin_once(self)

# Definition the main function
def main(args = None):

    rclpy.init(args=args)

    # Creating an instance of TurtleControlNode class
    turtle_control = TurtleControlNode()
    turtle_control.draw_circle(radius = 1.0, speed = 2.0)
    rclpy.shutdown()

# Call the main function
if __name__ == '__main__':
    main()
