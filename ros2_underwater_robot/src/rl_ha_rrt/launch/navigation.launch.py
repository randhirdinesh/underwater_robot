# Launch file for Navigation Stack
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "src", "rl_ha_rrt", "config"
    )

    return LaunchDescription([
        # **Launch SLAM System (SuperPoint & SuperGlue)**
        Node(
            package="superglue_slam",
            executable="slam_node",
            name="superglue_slam",
            output="screen"
        ),

        # **Launch AI-Based Sonar Obstacle Detector**
        Node(
            package="uuv_simulation",
            executable="sonar_ai_detector_trt",
            name="sonar_ai_detector",
            output="screen"
        ),

        # **Launch AI-Based Obstacle Avoidance with TensorRT**
        Node(
            package="uuv_simulation",
            executable="ai_obstacle_avoidance",
            name="ai_obstacle_avoidance",
            output="screen"
        ),

        # **Launch Navigation Stack**
        Node(
            package="nav2_bringup",
            executable="bringup_launch.py",
            name="nav2_bringup",
            output="screen",
            parameters=[os.path.join(config_dir, "navigation_underwater.yaml")]
        ),

        # **Launch MPPI-Based Obstacle Avoidance**
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[os.path.join(config_dir, "controller_mppi.yaml")]
        ),

        # **Launch RViz for Visualization**
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["-d", os.path.join(config_dir, "superglue_slam.rviz")]
        ),
    ])
