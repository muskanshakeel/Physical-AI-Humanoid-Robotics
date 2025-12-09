import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class SimulatedLidar(Node):
    def __init__(self):
        super().__init__('simulated_lidar')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info("Simulated LiDAR node started.")

        self.angle_min = -math.pi / 2  # -90 degrees
        self.angle_max = math.pi / 2   # +90 degrees
        self.angle_increment = math.pi / 360 # 0.5 degrees
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame' # Standard frame for lidar

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0 # Not used for simulated static scan
        scan.scan_time = 0.1 # 10 Hz scan

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Simulate some obstacles: a wall at 5m
        ranges = []
        intensities = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            # Simple wall simulation:
            if -math.pi/4 < angle < math.pi/4: # Obstacle in front
                ranges.append(5.0 + np.random.uniform(-0.1, 0.1)) # Wall at 5m with some noise
            else:
                ranges.append(float('inf')) # Clear
            intensities.append(0.0) # No intensity data

        scan.ranges = ranges
        scan.intensities = intensities
        
        self.publisher_.publish(scan)
        self.get_logger().info('Published simulated LaserScan message.')

def main(args=None):
    rclpy.init(args=args)
    simulated_lidar = SimulatedLidar()
    rclpy.spin(simulated_lidar)
    simulated_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
