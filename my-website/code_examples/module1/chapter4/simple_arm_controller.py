import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class SimpleArmController(Node):
    def __init__(self):
        super().__init__('simple_arm_controller')
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.command_subscriber = self.create_subscription(
            Float64MultiArray,
            'arm_commands',
            self.command_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        
        self.joint_positions = [0.0, 0.0] # Two joints
        self.target_joint_positions = [0.0, 0.0]
        self.joint_names = ['joint1', 'joint2']
        self.get_logger().info("Simple Arm Controller node started.")

    def command_callback(self, msg):
        if len(msg.data) == len(self.joint_positions):
            self.target_joint_positions = list(msg.data)
            self.get_logger().info(f"Received arm commands: {self.target_joint_positions}")
        else:
            self.get_logger().warn("Received invalid number of joint commands.")

    def timer_callback(self):
        # Simulate moving towards target positions
        for i in range(len(self.joint_positions)):
            diff = self.target_joint_positions[i] - self.joint_positions[i]
            if abs(diff) > 0.01: # Move if difference is significant
                self.joint_positions[i] += math.copysign(0.01, diff)
            
        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f"Published joint states: {self.joint_positions}")

def main(args=None):
    rclpy.init(args=args)
    arm_controller = SimpleArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
