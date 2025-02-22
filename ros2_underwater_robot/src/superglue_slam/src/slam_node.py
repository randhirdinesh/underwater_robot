# SLAM Node integrating SuperPoint & SuperGlue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class SuperGlueSLAM(Node):
    def __init__(self):
        super().__init__('superglue_slam')

        # Subscribe to Sensors
        self.create_subscription(Image, "/keypoint_matches", self.keypoints_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publisher for Filtered Pose
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)

    def keypoints_callback(self, msg):
        """Process matched keypoints from perception node"""
        self.get_logger().info("Received Matched Keypoints")

    def imu_callback(self, msg):
        """Process IMU Data"""
        self.get_logger().info(f"IMU Orientation: {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}")

    def lidar_callback(self, msg):
        """Process Lidar Data"""
        self.get_logger().info(f"Lidar Scan Received: {len(msg.ranges)} points")

    def odom_callback(self, msg):
        """Process Odometry Data"""
        self.get_logger().info(f"Odometry Position: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)
    node = SuperGlueSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
