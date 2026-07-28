"""
inclio_hesai.launch.py
Launch the IncLIO ROS2 node with a Hesai Pandar128.

Usage:
    ros2 launch inclio_ros2 inclio_hesai.launch.py \\
        config_file:=/path/to/hesai128.yaml \\
        imu_topic:=/sensor/sbg/imu/data \\
        lidar_topic:=/astra_lidar/data_filtered
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("config_file",
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare("inclio_ros2"), "config", "hesai128_sbg.yaml"]),
                              description="Path to IncLIO YAML config file"),
        DeclareLaunchArgument("imu_topic",          default_value="/alphasense_driver_ros/imu",
                              description="IMU topic"),
        DeclareLaunchArgument("lidar_topic",        default_value="/hesai/pandar",
                              description="LiDAR PointCloud2 topic"),
        DeclareLaunchArgument("wheel_odom_topic",   default_value="",
                              description="Wheel-odometry nav_msgs/Odometry topic (empty = disabled)"),
        DeclareLaunchArgument("map_voxel_size",     default_value="0.01",
                              description="Voxel leaf size applied to full_map_ at save time (m)"),
        DeclareLaunchArgument("body_crop_radius",   default_value="1.5",
                              description="Remove points closer than this to the sensor — strips vehicle body (m)"),
        DeclareLaunchArgument("local_map_scans",    default_value="20",
                              description="Number of maximum recent scans in the sliding window for ~/cloud_world"), 
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
            "local_map_scans":    LaunchConfiguration("local_map_scans"),
            "publish_rate_hz":    LaunchConfiguration("publish_rate_hz"),
            "world_frame":        LaunchConfiguration("world_frame"),
            "body_frame":         LaunchConfiguration("body_frame"),
            "publish_tf":         LaunchConfiguration("publish_tf"),
            "publish_path":       LaunchConfiguration("publish_path"),
            "publish_cloud":      LaunchConfiguration("publish_cloud"),
            "imu_topic":          LaunchConfiguration("imu_topic"),
            "lidar_topic":        LaunchConfiguration("lidar_topic"),
            "wheel_odom_topic":   LaunchConfiguration("wheel_odom_topic"),
        }],
        remappings=[
            ("inclio/imu",    LaunchConfiguration("imu_topic")),
            ("inclio/points", LaunchConfiguration("lidar_topic")),
        ],
        # prefix=["gdbserver localhost:3000"]
    )

    return LaunchDescription(args + [node])
