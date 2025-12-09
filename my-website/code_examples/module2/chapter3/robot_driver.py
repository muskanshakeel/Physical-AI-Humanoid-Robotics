import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Run every second
        self.get_logger().info("Robot Driver node started.")
        self.forward_duration = 3  # seconds to move forward
        self.turn_duration = 2     # seconds to turn
        self.start_time = self.get_clock().now()
        self.state = 'forward'

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9 # convert to seconds

        twist_msg = Twist()

        if self.state == 'forward':
            twist_msg.linear.x = 0.5 # Move forward at 0.5 m/s
            twist_msg.angular.z = 0.0
            if elapsed_time > self.forward_duration:
                self.state = 'turn'
                self.start_time = current_time
                self.get_logger().info('Switching to turn state.')
        elif self.state == 'turn':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.8 # Turn at 0.8 rad/s
            if elapsed_time > self.turn_duration:
                self.state = 'forward'
                self.start_time = current_time
                self.get_logger().info('Switching to forward state.')
        
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f"Published Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)
    robot_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
