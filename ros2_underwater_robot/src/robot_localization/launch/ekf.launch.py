# Launch file for EKF sensor fusion
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "src", "robot_localization", "config", "ekf_3d_lidar_sonar_imu_camera_imu.yaml"
    )

    return LaunchDescription([
        # **Start EKF Node for Sensor Fusion**
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter",
            output="screen",
            parameters=[config_path],
        ),

        # **Start NavSat Transform (GPS) - Only if available**
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[os.path.join(os.path.dirname(config_path), "navsat_transform_optional.yaml")],
        ),
    ])
