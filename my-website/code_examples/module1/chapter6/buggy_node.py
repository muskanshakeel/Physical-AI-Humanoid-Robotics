import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 # Expecting Int32, but will send String in error

class BuggyNode(Node):
    def __init__(self):
        super().__init__('buggy_node')
        # Intentionally create a type mismatch by declaring publisher for Int32
        # but later trying to send String.
        self.publisher_ = self.create_publisher(Int32, 'buggy_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info("Buggy node started, attempting to publish to 'buggy_topic'...")

    def timer_callback(self):
        # This will cause a serialization error because msg.data is a string, not an int
        msg = Int32()
        # msg.data = self.i # Correct way
        msg.data = "This is a string, not an integer!" # Intentional error
        
        try:
            self.publisher_.publish(msg)
            self.get_logger().info('Published (should fail): "%s"' % msg.data)
        except Exception as e:
            self.get_logger().error(f"Error publishing message: {e}")
            self.get_logger().error("Hint: Check message type consistency between publisher and subscriber/topic definition.")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    buggy_node = BuggyNode()
    rclpy.spin(buggy_node)
    buggy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
