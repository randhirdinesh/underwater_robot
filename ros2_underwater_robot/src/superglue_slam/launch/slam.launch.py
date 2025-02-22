# ROS2 launch file for SuperGlue SLAM
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define package paths
    config_dir = os.path.join(os.path.expanduser("~"), "ros2_ws", "src", "superglue_slam", "config")
    
    return LaunchDescription([
        # Launch EKF (Sensor Fusion)
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter",
            output="screen",
            parameters=[os.path.join(config_dir, "ekf_3d_lidar_optional_gps.yaml")]
        ),

        # Launch Perception Node (SuperPoint & SuperGlue)
        Node(
            package="superglue_perception",
            executable="perception_node",
            name="superglue_perception",
            output="screen"
        ),

        # Launch SLAM Node (Feature Matching & Pose Estimation)
        Node(
            package="superglue_slam",
            executable="slam_node",
            name="superglue_slam",
            output="screen"
        ),

        # Launch NavSat Transform (GPS Fusion, Optional)
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[os.path.join(config_dir, "navsat_transform_optional.yaml")]
        ),

        # Launch RViz for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["-d", os.path.join(config_dir, "superglue_slam.rviz")]
        ),
    ])
