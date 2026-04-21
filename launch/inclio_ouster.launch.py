"""
inclio_velodyne.launch.py
Launch the IncLIO ROS2 node with a Velodyne (or any PointCloud2) LiDAR.

Usage:
    ros2 launch inclio_ros2 inclio_velodyne.launch.py \\
        config_file:=/path/to/velodyne_test.yaml \\
        imu_topic:=/imu/data \\
        lidar_topic:=/velodyne_points
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("config_file",      default_value="config/osdome128.yaml",
                              description="Path to IncLIO YAML config file"),
        DeclareLaunchArgument("imu_topic",         default_value="/ouster2/imu",
                              description="IMU topic"),
        DeclareLaunchArgument("lidar_topic",       default_value="/ouster2/points",
                              description="LiDAR PointCloud2 topic"),
        DeclareLaunchArgument("map_voxel_size",     default_value="0.05"),
        DeclareLaunchArgument("local_map_scans",     default_value="500"), # number of scans in local map
        DeclareLaunchArgument("world_frame",       default_value="world"),
        DeclareLaunchArgument("body_frame",        default_value="body"),
        DeclareLaunchArgument("publish_tf",        default_value="true"),
        DeclareLaunchArgument("publish_path",      default_value="true"),
        DeclareLaunchArgument("publish_cloud",     default_value="true")
        
        ]

    # ── Node ──────────────────────────────────────────────────────────────────
    node = Node(
        package="inclio_ros2",
        executable="inclio_ros2_node",
        name="inclio",
        output="screen",
        parameters=[{
            "config_file":      LaunchConfiguration("config_file"),
            "map_voxel_size":   LaunchConfiguration("map_voxel_size"),
            "local_map_scans":  LaunchConfiguration("local_map_scans"),
            "world_frame":      LaunchConfiguration("world_frame"),
            "body_frame":       LaunchConfiguration("body_frame"),
            "publish_tf":       LaunchConfiguration("publish_tf"),
            "publish_path":     LaunchConfiguration("publish_path"),
            "publish_cloud":    LaunchConfiguration("publish_cloud"),
            "imu_topic":       LaunchConfiguration("imu_topic"),
            "lidar_topic":     LaunchConfiguration("lidar_topic"),
        }],
        remappings=[
            # Map the node's ~/imu and ~/points to the actual sensor topics
            ("inclio/imu",    LaunchConfiguration("imu_topic")),
            ("inclio/points", LaunchConfiguration("lidar_topic")),
        ],
        # prefix=["gdbserver localhost:3000"]    
        )

    return LaunchDescription(args + [node])
