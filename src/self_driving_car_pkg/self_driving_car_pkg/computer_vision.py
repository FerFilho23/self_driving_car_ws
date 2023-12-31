import cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy
from .self_driving import Car

class ComputerVisionNode(Node):
    def __init__(self):

        super().__init__('computer_vision_node')
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)

        self.velocity = Twist()
        self.bridge   = CvBridge() # converting ros images to opencv data
        self.Car = Car()

    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)
        
    def process_data(self, data): 

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
        
        self.Car.drive_car(frame)
        
        Angle,Speed,img = 0.0, 0.0, frame

        self.velocity.angular.z = Angle
        self.velocity.linear.x = Speed      

        cv2.imshow("Frame",img)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ComputerVisionNode()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()