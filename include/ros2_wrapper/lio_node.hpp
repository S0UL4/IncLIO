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
//   ~/cloud_body    sensor_msgs/msg/PointCloud2    (deskewed current scan in base_link,
//                                                   or IMU frame if no base<-imu TF)
//   ~/cloud_world   sensor_msgs/msg/PointCloud2    (current scan in world frame)
//
// TF:
//   world → body  (configurable frame names)
//
// Threading: uses two MutuallyExclusiveCallbackGroups so IMU buffering
// runs concurrently with heavy LiDAR processing on a MultiThreadedExecutor.
//
// Sensor setup is discovered, not configured: the message type published on
// lidar_topic decides between the PointCloud2 and the Livox CustomMsg path, the
// PointCloud2 field table gives the point layout and the per-point time
// encoding (see cloud_convert.hpp), and the accelerometer unit follows from the
// magnitude of the first IMU samples (see imu_convert.hpp).
//
// Parameters (all have sensible defaults):
//   config_file       : path to IncLIO YAML config
//   imu_topic         : default "imu"
//   lidar_topic       : default "points"
//   point_filter_num  : point decimation (YAML)
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
#include <string>
#include <cstdio>

// TF listener libraries (for reading lidar->imu extrinsic from TF if configured)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/console/print.h>


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

    // Asks the ROS graph which message type is published on `topic`, waiting up
    // to `timeout_s` for a publisher to appear. Returns "" if none shows up.
    std::string DiscoverTopicType(const std::string& topic, double timeout_s);

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void ImuCallback(sensor_msgs::msg::Imu::UniquePtr msg);
    void CloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void OdomCallback(nav_msgs::msg::Odometry::UniquePtr msg);
    void ui_callback();

    // ── Viz worker (downsample + crop + publish, outside executor pool) ─────
    void VizWorkerLoop();

    // Controlling terminal (/dev/tty) for the in-place init progress bar, or nullptr.
    FILE* ProgressTTY();

#ifdef HAVE_LIVOX_ROS_DRIVER2
    void LivoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
#endif

    // ── Publishing helpers ────────────────────────────────────────────────────
    void PublishOdometry(const rclcpp::Time& stamp, const IncLIO::SE3& pose,
                         const IncLIO::Stated& state);
    void PublishPath(const rclcpp::Time& stamp, const IncLIO::SE3& pose);
    void PublishCloud(const rclcpp::Time& stamp, const IncLIO::SE3& pose);
    void PublishBodyCloud(const rclcpp::Time& stamp);
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       wheel_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;


#ifdef HAVE_LIVOX_ROS_DRIVER2
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
#endif

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_fast_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_world_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_body_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

    // Path history
    nav_msgs::msg::Path path_msg_;

    // ── Parameters ────────────────────────────────────────────────────────────
    std::string config_file_;
    std::string world_frame_;
    std::string body_frame_;
    bool publish_path_  = true;
    bool publish_cloud_ = true;
    bool publish_body_cloud_ = true;   // ~/cloud_body — deskewed scan in body frame
    bool init_announced_ = false;   // logs the "tracking started" message once
    bool init_bar_shown_ = false;   // an in-place init progress bar was drawn (needs a closing newline)

    // Controlling terminal for the in-place init progress bar (see ProgressTTY()).
    std::FILE* progress_tty_ = nullptr;
    bool       progress_tty_attempted_ = false;

    // ── Transformed-scan queue (lidar thread → timer thread) ────────────────
    struct ScanEntry {
        IncLIO::CloudPtr cloud;
        IncLIO::SE3 pose;
    };
    std::deque<ScanEntry> scan_queue_;
    std::mutex scan_queue_mutex_;

    // ── World-frame map storage ─────────────────────────────────────────────
    // full_map_        : raw accumulated world-frame points; voxel-filtered on save.
    // viz_scan_window_ : sliding window of the last viz_max_scans_ world-frame scans
    //                    (each pre-filtered per scan); concatenated for publish.
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
    double publish_rate_hz_    = 5.0;

    // Latest corrected pose from the lidar thread.
    IncLIO::SE3 current_pose_;

    // ── Viz worker state ────────────────────────────────────────────────────
    // The worker concatenates the sliding window and publishes independently of
    // the executor thread pool so that ui_callback always returns in < 5 ms.
    std::thread             viz_worker_;
    std::mutex              viz_cv_mutex_;
    std::condition_variable viz_cv_;
    bool                    viz_work_ready_  = false;
    bool                    viz_worker_stop_ = false;

    // CloudConvertConfig 
    CloudConvertConfig cc;


    // to project odom in base_link
    std::string world_frame_id = "camera_init";
    std::string base_frame_id  = "body";
    std::string imu_frame_id   = "body";
    std::string lidar_frame_id = "lidar";
    bool publish_tf_ = true;
    bool use_tf_extrinsic = false;
    double tf_lookup_timeout = 5.0;

    IncLIO::Vec3d t_imu_base{IncLIO::Zero3d};
    IncLIO::Mat3d R_imu_base{IncLIO::Eye3d};

    // base_link ← IMU, used to bring the deskewed scan out of the IMU frame.
    // Identity (and have_base_offset_ = false) when no base<-imu TF is available,
    // in which case ~/cloud_body is published in imu_frame_id as-is.
    IncLIO::SE3 T_base_imu_;
    bool        have_base_offset_ = false;
    std::string body_cloud_frame_id_;

    IncLIO::Vec3d Lidar_T_wrt_IMU{IncLIO::Zero3d};
    IncLIO::Mat3d Lidar_R_wrt_IMU{IncLIO::Eye3d};

    // Re-anchor the published odom frame to base_link's first pose so base_link
    // starts at the origin. T_base0_world_ = (first T_world_base)^-1, captured once.
    IncLIO::SE3 T_base0_world_;
    bool        have_base_origin_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace inclio_ros2
