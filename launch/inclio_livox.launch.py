"""
inclio_livox.launch.py
Launch the IncLIO ROS2 node with a Livox Mid-360.

Livox publishes on two topics simultaneously:
  /livox/lidar  → sensor_msgs/PointCloud2   (PointCloud2 mode)
  /livox/lidar  → livox_ros_driver2/CustomMsg  (native mode, preferred)

The wrapper subscribes to both at the same time; whichever arrives is
converted.  If you only want one, comment out the other remapping below.

Usage:
    ros2 launch inclio_ros2 inclio_livox.launch.py \\
        config_file:=/path/to/livox_mid360.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("config_file",      default_value="",
                              description="Path to IncLIO YAML config file"),
        DeclareLaunchArgument("imu_topic",         default_value="/livox/imu",
                              description="IMU topic"),
        DeclareLaunchArgument("lidar_topic",       default_value="/livox/lidar",
                              description="LiDAR PointCloud2 topic"),
        DeclareLaunchArgument("lidar_type",        default_value="1",
                              description="1=Livox"),
        DeclareLaunchArgument("num_scans",         default_value="4",
                              description="Mid-360 has 4 scan lines"),
        DeclareLaunchArgument("time_scale",        default_value="1e-6",
                              description="Livox offset_time is in nanoseconds"),
        DeclareLaunchArgument("point_filter_num",  default_value="4"),
        DeclareLaunchArgument("world_frame",       default_value="world"),
        DeclareLaunchArgument("body_frame",        default_value="body"),
    ]

    node = Node(
        package="inclio_ros2",
        executable="inclio_ros2_node",
        name="inclio",
        output="screen",
        parameters=[{
            "config_file":      LaunchConfiguration("config_file"),
            "lidar_type":       LaunchConfiguration("lidar_type"),
            "num_scans":        LaunchConfiguration("num_scans"),
            "time_scale":       LaunchConfiguration("time_scale"),
            "point_filter_num": LaunchConfiguration("point_filter_num"),
            "world_frame":      LaunchConfiguration("world_frame"),
            "body_frame":       LaunchConfiguration("body_frame"),
        }],
        remappings=[
            ("inclio/imu",          LaunchConfiguration("imu_topic")),
            # PointCloud2 fallback
            ("inclio/points",       LaunchConfiguration("lidar_topic")),
            # Livox native (only active when built with HAVE_LIVOX_ROS_DRIVER2)
            ("inclio/points_livox", "/livox/lidar"),
        ],
    )

    return LaunchDescription(args + [node])
