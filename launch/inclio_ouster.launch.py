"""
inclio_ouster.launch.py
Launch the IncLIO ROS2 node with an Ouster lidar.

Usage:
    ros2 launch inclio_ros2 inclio_ouster.launch.py \\
        config_file:=/path/to/osdome128.yaml \\
        imu_topic:=/ouster/imu \\
        lidar_topic:=/ouster/points
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("config_file",        default_value="/home/soula/Desktop/PROJECTS/iheb_slam/src/IncLIO/config/osdome128.yaml",
                              description="Path to IncLIO YAML config file"),
        DeclareLaunchArgument("imu_topic",          default_value="/os1_cloud_node/imu",
                              description="IMU topic"),
        DeclareLaunchArgument("lidar_topic",        default_value="/os1_cloud_node/points",
                              description="LiDAR PointCloud2 topic"),
        DeclareLaunchArgument("map_voxel_size",     default_value="0.05",
                              description="Voxel leaf size applied to full_map_ at save time (m)"),
        DeclareLaunchArgument("body_crop_radius",   default_value="1.0",
                              description="Remove points closer than this to the sensor — strips vehicle body (m)"),
        DeclareLaunchArgument("publish_voxel_size", default_value="0.1",
                              description="Voxel leaf size for ~/cloud_world (m); smaller = denser cloud"),
        DeclareLaunchArgument("local_map_scans",    default_value="50",
                              description="Number of recent scans in the sliding window for ~/cloud_world"),
        DeclareLaunchArgument("publish_radius",     default_value="500.0",
                              description="Radius around current pose included in ~/cloud_world (m)"),
        DeclareLaunchArgument("publish_rate_hz",    default_value="10.0",
                              description="Publish rate of ~/cloud_world (Hz)"),
        DeclareLaunchArgument("world_frame",        default_value="world"),
        DeclareLaunchArgument("body_frame",         default_value="body"),
        DeclareLaunchArgument("publish_tf",         default_value="true"),
        DeclareLaunchArgument("publish_path",       default_value="true"),
        DeclareLaunchArgument("publish_cloud",      default_value="true"),
    ]

    # ── Node ──────────────────────────────────────────────────────────────────
    node = Node(
        package="inclio_ros2",
        executable="inclio_ros2_node",
        name="inclio",
        output="screen",
        parameters=[{
            "config_file":        LaunchConfiguration("config_file"),
            "map_voxel_size":     LaunchConfiguration("map_voxel_size"),
            "body_crop_radius":   LaunchConfiguration("body_crop_radius"),
            "publish_voxel_size": LaunchConfiguration("publish_voxel_size"),
            "local_map_scans":    LaunchConfiguration("local_map_scans"),
            "publish_radius":     LaunchConfiguration("publish_radius"),
            "publish_rate_hz":    LaunchConfiguration("publish_rate_hz"),
            "world_frame":        LaunchConfiguration("world_frame"),
            "body_frame":         LaunchConfiguration("body_frame"),
            "publish_tf":         LaunchConfiguration("publish_tf"),
            "publish_path":       LaunchConfiguration("publish_path"),
            "publish_cloud":      LaunchConfiguration("publish_cloud"),
            "imu_topic":          LaunchConfiguration("imu_topic"),
            "lidar_topic":        LaunchConfiguration("lidar_topic"),
        }],
        remappings=[
            ("inclio/imu",    LaunchConfiguration("imu_topic")),
            ("inclio/points", LaunchConfiguration("lidar_topic")),
        ],
        # prefix=["gdbserver localhost:3000"]
    )

    return LaunchDescription(args + [node])
