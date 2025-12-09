import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class SimpleManipulatorTarget(Node):
    def __init__(self):
        super().__init__('simple_manipulator_target')
        self.publisher_ = self.create_publisher(PoseStamped, 'manipulator_target_pose', 10)
        self.timer = self.create_timer(5.0, self.timer_callback) # Publish target every 5 seconds
        self.get_logger().info("Simple Manipulator Target node started.")

        self.current_target_index = 0
        self.targets = [
            [0.5, 0.5, 0.5],  # Target 1
            [0.5, -0.5, 0.7], # Target 2
            [0.3, 0.0, 0.4]   # Target 3
        ]

    def timer_callback(self):
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'world' # Or base_link of the robot
        
        target_coords = self.targets[self.current_target_index]
        target_pose.pose.position.x = target_coords[0]
        target_pose.pose.position.y = target_coords[1]
        target_pose.pose.position.z = target_coords[2]
        
        # Orientation (identity quaternion for now)
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        self.publisher_.publish(target_pose)
        self.get_logger().info(f"Published new target pose: x={target_coords[0]}, y={target_coords[1]}, z={target_coords[2]}")

        self.current_target_index = (self.current_target_index + 1) % len(self.targets)

def main(args=None):
    rclpy.init(args=args)
    manipulator_target_node = SimpleManipulatorTarget()
    rclpy.spin(manipulator_target_node)
    manipulator_target_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
