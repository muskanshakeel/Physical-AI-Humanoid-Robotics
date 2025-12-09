import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Run every second
        self.state = 0 # 0: move forward, 1: turn
        self.get_logger().info("Simple Navigator node started.")

    def timer_callback(self):
        twist_msg = Twist()
        if self.state == 0: # Move forward
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving forward...')
            self.state = 1
        elif self.state == 1: # Turn
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5
            self.get_logger().info('Turning...')
            self.state = 0
        
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    simple_navigator = SimpleNavigator()
    rclpy.spin(simple_navigator)
    simple_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
