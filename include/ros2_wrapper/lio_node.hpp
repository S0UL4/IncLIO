#include <omp.h>
#pragma once

// lio_node.hpp
//
// ROS2 composable node that wraps IncLIO::LIO.
//
// Subscriptions:
//   ~/imu           sensor_msgs/msg/Imu
//   ~/points        sensor_msgs/msg/PointCloud2
//   ~/points_livox  livox_ros_driver2/msg/CustomMsg  (only if built with Livox support)
//
// Publications:
//   ~/odometry      nav_msgs/msg/Odometry          (pose + twist in world frame)
//   ~/path          nav_msgs/msg/Path              (trajectory history)
//   ~/cloud_body    sensor_msgs/msg/PointCloud2    (current scan in body/IMU frame)
//   ~/cloud_world   sensor_msgs/msg/PointCloud2    (current scan in world frame)
//
// TF:
//   world → body  (configurable frame names)
//
// Threading: uses two MutuallyExclusiveCallbackGroups so IMU buffering
// runs concurrently with heavy LiDAR processing on a MultiThreadedExecutor.
//
// Parameters (all have sensible defaults):
//   config_file       : path to IncLIO YAML config
//   imu_topic         : default "imu"
//   lidar_topic       : default "points"
//   lidar_type        : 1=Livox, 2=Velodyne/generic PC2, 3=Ouster
//   num_scans         : scan lines
//   time_scale        : per-point time scale
//   point_filter_num  : point decimation
//   world_frame       : default "world"
//   body_frame        : default "body"
//   publish_tf        : default true
//   publish_path      : default true
//   publish_cloud     : default true

#include "ros2_wrapper/cloud_convert.hpp"
#include "ros2_wrapper/imu_convert.hpp"

#include "inclio/inclio.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#ifdef HAVE_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <mutex>
#include <deque>
#include <unordered_map>


using namespace std::chrono_literals;

namespace inclio_ros2 {

class LioNode : public rclcpp::Node {
public:
    explicit LioNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LioNode() override;

private:
    // ── Initialisation ────────────────────────────────────────────────────────
    void DeclareParameters();
    bool InitLIO();
    void CreateSubscriptions();
    void CreatePublishers();

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void ImuCallback(sensor_msgs::msg::Imu::UniquePtr msg);
    void CloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void ui_callback();

#ifdef HAVE_LIVOX_ROS_DRIVER2
    void LivoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
#endif

    // ── Publishing helpers ────────────────────────────────────────────────────
    void PublishOdometry(const rclcpp::Time& stamp, const IncLIO::SE3& pose,
                         const IncLIO::Stated& state);
    void PublishPath(const rclcpp::Time& stamp, const IncLIO::SE3& pose);
    void PublishCloud(const rclcpp::Time& stamp, const IncLIO::SE3& pose);
    void PublishTF(const rclcpp::Time& stamp, const IncLIO::SE3& pose);

    // ── Service callbacks ────────────────────────────────────────────────────
    void SaveMapCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                         std_srvs::srv::Trigger::Response::SharedPtr res);

    // ── Shared LiDAR processing (called by CloudCallback / LivoxCallback) ────
    void ProcessCloud(IncLIO::FullCloudPtr cloud, double ts, const rclcpp::Time& stamp);

    // ── IncLIO pipeline ───────────────────────────────────────────────────────
    std::unique_ptr<IncLIO::LIO> lio_;
    CloudConverter converter_;
    IMUConverter imu_converter_;

    // IMU buffer — filled by ImuCallback, drained by ProcessCloud
    std::deque<IMUPtr> imu_buffer_;
    std::mutex imu_buf_mutex_;

    // ── Callback groups (enable parallel IMU + LiDAR on MultiThreadedExecutor)
    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;

    // ── ROS2 I/O ──────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::TimerBase::SharedPtr timer_;


#ifdef HAVE_LIVOX_ROS_DRIVER2
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
#endif

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_fast_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_world_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

    // Path history
    nav_msgs::msg::Path path_msg_;

    // ── Parameters ────────────────────────────────────────────────────────────
    std::string config_file_;
    std::string world_frame_;
    std::string body_frame_;
    bool publish_tf_    = true;
    bool publish_path_  = true;
    bool publish_cloud_ = true;

    // ── Voxelized full map ───────────────────────────────────────────────────
    // One point per voxel cell — clean, uniform-density, grows over time.
    // Transform + insert happens on the timer thread, not the lidar callback.
    struct ScanEntry {
        IncLIO::CloudPtr cloud;
        IncLIO::SE3 pose;
    };
    std::deque<ScanEntry> scan_queue_;
    std::mutex scan_queue_mutex_;

    using VoxelMap = std::unordered_map<IncLIO::Vec3i, IncLIO::PointType,
                                        IncLIO::hash_vec<3>>;
    VoxelMap full_map_;
    std::mutex map_mutex_;
    double map_voxel_size_ = 0.2;

    // Sliding window of recent world-frame scans for local map visualization.
    // Scans are pre-voxelized at publish_voxel_size_ before insertion so the
    // window can hold more history without exploding the publish payload.
    // Only accessed from ui_callback (timer_group_, MutuallyExclusive) — no mutex needed.
    std::deque<IncLIO::CloudPtr> local_scan_window_;
    int local_map_max_scans_ = 20;

    // Visualization publish config — decoupled from LIO update rate.
    double publish_voxel_size_ = 0.3;   // per-scan voxel size for the local window
    double publish_radius_     = 80.0;  // crop around current pose at publish
    double publish_rate_hz_    = 5.0;   // ~/cloud_world publish rate

    // Latest corrected pose from the lidar thread — used as the crop center.
    IncLIO::SE3 current_pose_;

    // CloudConvertConfig 
    CloudConvertConfig cc;
};

} // namespace inclio_ros2
