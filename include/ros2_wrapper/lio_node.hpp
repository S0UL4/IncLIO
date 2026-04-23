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

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

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
#include <thread>
#include <condition_variable>
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

    // ── Viz worker (downsample + crop + publish, outside executor pool) ─────
    void VizWorkerLoop();

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

    // ── Transformed-scan queue (lidar thread → timer thread) ────────────────
    struct ScanEntry {
        IncLIO::CloudPtr cloud;
        IncLIO::SE3 pose;
    };
    std::deque<ScanEntry> scan_queue_;
    std::mutex scan_queue_mutex_;

    // ── World-frame map storage ─────────────────────────────────────────────
    // full_map_        : raw accumulated world-frame points; voxel-filtered on save.
    // viz_scan_window_ : sliding window of the last viz_max_scans_ world-frame scans;
    //                    concatenated + voxel-filtered + radius-cropped for publish.
    IncLIO::CloudPtr full_map_;
    std::deque<IncLIO::CloudPtr> viz_scan_window_;
    int viz_max_scans_ = 20;

    // full_map_mutex_ : protects full_map_ (ui_callback append + SaveMapCallback copy).
    // viz_map_mutex_  : protects viz_scan_window_ (ui_callback push + VizWorkerLoop read).
    //                   ui_callback uses try_lock so it never blocks on the worker.
    std::mutex full_map_mutex_;
    std::mutex viz_map_mutex_;

    double map_voxel_size_     = 0.2;
    double body_crop_radius_   = 1.0;   // points closer than this to the sensor are removed
    double publish_voxel_size_ = 0.3;

    pcl::CropBox<IncLIO::PointType> scan_crop_;   // near-field body removal, configured once in InitLIO
    double publish_radius_     = 80.0;
    double publish_rate_hz_    = 5.0;

    // Latest corrected pose from the lidar thread — used as the crop center.
    IncLIO::SE3 current_pose_;

    // ── Viz worker state ────────────────────────────────────────────────────
    // The worker runs voxel downsample + radius crop + publish independently of
    // the executor thread pool so that ui_callback always returns in < 5 ms.
    std::thread             viz_worker_;
    std::mutex              viz_cv_mutex_;
    std::condition_variable viz_cv_;
    bool                    viz_work_ready_  = false;
    bool                    viz_worker_stop_ = false;
    IncLIO::SE3             viz_crop_center_;  // crop centre passed to the worker

    // CloudConvertConfig 
    CloudConvertConfig cc;
};

} // namespace inclio_ros2
