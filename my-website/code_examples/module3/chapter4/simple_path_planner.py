import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')
        self.publisher_ = self.create_publisher(Path, 'plan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish path every second
        self.get_logger().info("Simple Path Planner node started.")

        self.current_time = self.get_clock().now().to_msg()
        self.path_points = []
        # Generate a simple square path
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            self.path_points.append(pose)
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 4.0
            pose.pose.position.y = float(i)
            pose.pose.position.z = 0.0
            self.path_points.append(pose)
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 4.0 - float(i)
            pose.pose.position.y = 4.0
            pose.pose.position.z = 0.0
            self.path_points.append(pose)
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 0.0
            pose.pose.position.y = 4.0 - float(i)
            pose.pose.position.z = 0.0
            self.path_points.append(pose)

    def timer_callback(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.poses = self.path_points
        
        self.publisher_.publish(path_msg)
        self.get_logger().info('Published simulated path.')

def main(args=None):
    rclpy.init(args=args)
    simple_path_planner = SimplePathPlanner()
    rclpy.spin(simple_path_planner)
    simple_path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
