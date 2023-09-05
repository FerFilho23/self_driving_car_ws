import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')
        
        # Create a publisher to publish Twist messages to /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)

        # Create a timer with a 0.5-second interval for publishing
        self.timer = self.create_timer(0.5, self.send_cmd_vel)

    def send_cmd_vel(self):
        # Create a Twist message and set linear and angular velocities
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.0  # Adjust this value for linear velocity
        cmd_vel_msg.angular.z = 1.0  # Adjust this value for angular velocity

        # Publish the Twist message
        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info("Published cmd_vel: linear=%.2f, angular=%.2f" % (cmd_vel_msg.linear.x, cmd_vel_msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
