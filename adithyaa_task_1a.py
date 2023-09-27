import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from turtlesim.srv import Spawn

class TurtleCircle(Node):

    def __init__(self, radius, name = '1'):
        super().__init__(f'turtleCircle{name}')
        self.name = f'turtle{name}'
        self.get_logger().info(f'{self.name}')
        self.radius = radius  # Adjust this radius to change the size of the circle.
        self.radius_change = False
        self.subscription = self.create_subscription(Pose, f'/{self.name}/pose', self.listener_callback, 10)
        self.publish_msg = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.current_velocity = Twist()
        timer_period = 0.5  # seconds
        self.start = 0
        self.timer = self.create_timer(timer_period, self.publish_new_velocity)
        self.initial_pose = Pose()
        self.counter = 0


    def publish_new_velocity(self):
        # To make the turtle move in a circle, set the linear velocity and angular velocity appropriately.
        
        if self.start < 2:

            linear_speed = 0.6  # Adjust the linear speed as needed.
            angular_speed = linear_speed / self.radius
            self.current_velocity.linear.x = linear_speed
            self.current_velocity.linear.y = 0
            self.current_velocity.linear.z = 0
            self.current_velocity.angular.x = 0
            self.current_velocity.angular.y = 0
            self.current_velocity.angular.z = angular_speed
            self.publish_msg.publish(self.current_velocity)
        
    
    def listener_callback(self, msg):
        try:
            self.prevPose = self.curPose   
        except:
            self.prevPose = Pose()
            self.curPose = Pose()
        self.curPose = msg

        if self.counter == 0:
            self.initial_pose = self.curPose
            self.counter += 1

        if (self.curPose.theta * self.prevPose.theta) < 0:
            self.start += 1

        if self.start == 2 and not self.radius_change :
            self.stop_turtle()
            self.radius_change = True
            raise SystemExit

    def stop_turtle(self):
        self.publish_msg.publish(Twist())


def spawn_turtle(current_pose):
    node = rclpy.create_node('spawn_turtle_node')

    # Create a client to call the spawn service
    spawn_client = node.create_client(Spawn, 'spawn')

    # Wait for the service to become available
    while not spawn_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service "spawn" not available, waiting...')

    # Create a request to spawn the turtle
    request = Spawn.Request()
    request.x = current_pose.x  # Set the X coordinate of the turtle's spawn point
    request.y = current_pose.y  # Set the Y coordinate of the turtle's spawn point
    request.name = 'turtle2'  # Set the name of the turtle

    # Call the spawn service to create the turtle
    future = spawn_client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Spawned turtle with name: {request.name}')
    else:
        node.get_logger().error('Failed to spawn turtle')



def main(args=None):
    rclpy.init(args=args)
    radius = 1
    turtleCircle = TurtleCircle(radius, '1')
    try:
        rclpy.spin(turtleCircle)
    except SystemExit:
        pass
    turtleCircle.get_logger().error('Outside first turtle')

    spawn_turtle(turtleCircle.initial_pose)

    new_radius = radius * -1.5
    turtleCircle.get_logger().info("Outside the Function")
    turtleCircle = TurtleCircle(new_radius, '2')
    try:
        rclpy.spin(turtleCircle)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()