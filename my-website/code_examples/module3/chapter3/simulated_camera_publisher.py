import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimulatedCameraPublisher(Node):
    def __init__(self):
        super().__init__('simulated_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.br = CvBridge()
        self.get_logger().info("Simulated Camera Publisher node started.")

        # Create a dummy image
        self.image_width = 640
        self.image_height = 480
        self.dummy_image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        self.dummy_image[100:200, 100:300] = [255, 0, 0] # Red rectangle
        self.dummy_image[250:350, 350:550] = [0, 255, 0] # Green rectangle

    def timer_callback(self):
        # Convert OpenCV image to ROS Image message
        ros_image = self.br.cv2_to_imgmsg(self.dummy_image, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'
        
        self.publisher_.publish(ros_image)
        self.get_logger().info('Published simulated camera image.')

def main(args=None):
    rclpy.init(args=args)
    simulated_camera_publisher = SimulatedCameraPublisher()
    rclpy.spin(simulated_camera_publisher)
    simulated_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
