import rclpy
from rclpy.node import Node

class IsaacRosDummyNode(Node):
    def __init__(self):
        super().__init__('isaac_ros_dummy_node')
        self.get_logger().info('Isaac ROS Dummy Node started. This simulates a basic Isaac ROS component.')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Isaac ROS Dummy Node is alive.')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacRosDummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
