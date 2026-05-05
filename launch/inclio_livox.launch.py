"""
inclio_livox.launch.py
Launch the IncLIO ROS2 node with a Livox Mid-360.

Usage:
    ros2 launch inclio_ros2 inclio_livox.launch.py \\
        config_file:=/path/to/livox_mid360.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("config_file",        default_value="/home/soula/Desktop/PROJECTS/iheb_slam/src/IncLIO/config/avia.yaml",
                              description="Path to IncLIO YAML config file"),
        DeclareLaunchArgument("imu_topic",          default_value="/livox/imu",
                              description="IMU topic"),
        DeclareLaunchArgument("lidar_topic",        default_value="/livox/lidar",
                              description="LiDAR topic (CustomMsg or PointCloud2)"),
        DeclareLaunchArgument("map_voxel_size",     default_value="0.05",
                              description="Voxel leaf size applied to full_map_ at save time (m)"),
        DeclareLaunchArgument("body_crop_radius",   default_value="0.5",
                              description="Remove points closer than this to the sensor — strips vehicle body (m)"),
        DeclareLaunchArgument("local_map_scans",    default_value="30",
                              description="Number of recent scans in the sliding window for ~/cloud_world"),
        DeclareLaunchArgument("publish_rate_hz",    default_value="10.0",
                              description="Publish rate of ~/cloud_world (Hz)"),
        DeclareLaunchArgument("world_frame",        default_value="world"),
        DeclareLaunchArgument("body_frame",         default_value="body"),
        DeclareLaunchArgument("publish_tf",         default_value="true"),
        DeclareLaunchArgument("publish_path",       default_value="true"),
        DeclareLaunchArgument("publish_cloud",      default_value="true"),

        # ── Loop closure (Phase A: detection only — see doc/loop_closure_steps.txt)
        DeclareLaunchArgument("loop_closure_enable",              default_value="true",
                              description="Enable NDTMC loop candidate detection (logs to stdout, no map correction)"),
        DeclareLaunchArgument("loop_closure_pc_max_radius",       default_value="80.0",
                              description="Descriptor cylinder radius (m)"),
        DeclareLaunchArgument("loop_closure_lidar_height",        default_value="0.5",
                              description="Sensor height above ground (m) — z-shift for NDTMC convention"),
        DeclareLaunchArgument("loop_closure_pc_max_z",            default_value="6.0",
                              description="Max height above ground used by descriptor (m)"),
        DeclareLaunchArgument("loop_closure_min_keyframe_gap",    default_value="20",
                              description="Min keyframe-id gap before a loop candidate is considered"),
        DeclareLaunchArgument("loop_closure_sc_dist_thres",       default_value="0.5",
                              description="Descriptor distance threshold (lower = stricter; tune on bags)"),
        DeclareLaunchArgument("loop_closure_max_spatial_dist",    default_value="30.0",
                              description="Max ||t_cur - t_cand|| (m) — gates out spatially-far false positives"),
        DeclareLaunchArgument("loop_closure_search_frequency_hz", default_value="1.0",
                              description="Throttle for the candidate search loop (Hz)"),
        DeclareLaunchArgument("loop_closure_pc_num_ring",         default_value="60"),
        DeclareLaunchArgument("loop_closure_pc_num_sector",       default_value="80"),
        DeclareLaunchArgument("loop_closure_pc_num_z",            default_value="6"),
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

            "loop_closure.enable":              LaunchConfiguration("loop_closure_enable"),
            "loop_closure.pc_max_radius":       LaunchConfiguration("loop_closure_pc_max_radius"),
            "loop_closure.lidar_height":        LaunchConfiguration("loop_closure_lidar_height"),
            "loop_closure.pc_max_z":            LaunchConfiguration("loop_closure_pc_max_z"),
            "loop_closure.min_keyframe_gap":    LaunchConfiguration("loop_closure_min_keyframe_gap"),
            "loop_closure.sc_dist_thres":       LaunchConfiguration("loop_closure_sc_dist_thres"),
            "loop_closure.max_spatial_dist":    LaunchConfiguration("loop_closure_max_spatial_dist"),
            "loop_closure.search_frequency_hz": LaunchConfiguration("loop_closure_search_frequency_hz"),
            "loop_closure.pc_num_ring":         LaunchConfiguration("loop_closure_pc_num_ring"),
            "loop_closure.pc_num_sector":       LaunchConfiguration("loop_closure_pc_num_sector"),
            "loop_closure.pc_num_z":            LaunchConfiguration("loop_closure_pc_num_z"),
        }],
        remappings=[
            ("inclio/imu",    LaunchConfiguration("imu_topic")),
            ("inclio/points", LaunchConfiguration("lidar_topic")),
        ],
        # prefix=["gdbserver localhost:3000"]
    )

    return LaunchDescription(args + [node])
