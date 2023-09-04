import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoRecorderNode(Node):

    def __init__(self):
        super().__init__('video_recorder_node') #Node Name

        # Create a subscriber
        self.subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.process_data, 10)
        
        # Write the frames into a video
        self.out = cv2.VideoWriter('/home/ferfilho/ros2/self_driving_car_ws/src/prius_sdc_pkg/prius_sdc_pkg/video_recorder_out/output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (1280,720))

        # Converts between OpenCV Images and ROS Image messages
        self.bridge = CvBridge()

    def process_data(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.out.write(cv_image)    #Write the frames into a video
            cv2.imshow("Video Recorder", cv_image) # Display the image
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
