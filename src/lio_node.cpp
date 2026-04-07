// lio_node.cpp
//
// ROS2 composable node that wraps IncLIO::LIO.
// See lio_node.hpp for the full interface description.

#include "ros2_wrapper/lio_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <chrono>

namespace inclio_ros2 {

// ─────────────────────────────────────────────────────────────────────────────
// Construction / Destruction
// ─────────────────────────────────────────────────────────────────────────────
LioNode::LioNode(const rclcpp::NodeOptions& options)
    : Node("inclio_ros2_node", options)
{
    DeclareParameters();

    if (!InitLIO()) {
        RCLCPP_FATAL(get_logger(), "IncLIO initialisation failed — shutting down");
        throw std::runtime_error("IncLIO init failed");
    }

    // Callback groups: two MutuallyExclusive groups so that IMU and LiDAR
    // callbacks can run in parallel on a MultiThreadedExecutor, while each
    // stream remains internally sequential.
    imu_cb_group_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lidar_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    CreatePublishers();
    CreateSubscriptions();

    RCLCPP_INFO(get_logger(), "IncLIO ROS2 node ready (multi-threaded)");
}


LioNode::~LioNode() {
    if (lio_) {
        lio_->Finish();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DeclareParameters
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::DeclareParameters() {
    // Config file path (required — no default makes sense here)
    declare_parameter<std::string>("config_file", "");

    // Topic remappings (the node uses ~/imu and ~/points by default;
    // these params allow overriding the suffix without ROS2 topic remapping)
    declare_parameter<std::string>("imu_topic",   "imu");
    declare_parameter<std::string>("lidar_topic", "points");

    // Point cloud conversion (mirrors run_bag.cc CLI / YAML options)
    declare_parameter<int>   ("lidar_type",        4);     // 1=Livox, 2=Velodyne, 3=Ouster
    declare_parameter<int>   ("num_scans",         128);
    declare_parameter<double>("time_scale",        1e-3);
    declare_parameter<int>   ("point_filter_num",  1);

    // Frame ids
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("body_frame",  "body");

    // Feature toggles
    declare_parameter<bool>("publish_tf",    true);
    declare_parameter<bool>("publish_path",  true);
    declare_parameter<bool>("publish_cloud", true);
}

// ─────────────────────────────────────────────────────────────────────────────
// InitLIO
// ─────────────────────────────────────────────────────────────────────────────
bool LioNode::InitLIO() {
    // ── Read parameters ───────────────────────────────────────────────────────
    config_file_   = get_parameter("config_file").as_string();
    world_frame_   = get_parameter("world_frame").as_string();
    body_frame_    = get_parameter("body_frame").as_string();
    publish_tf_    = get_parameter("publish_tf").as_bool();
    publish_path_  = get_parameter("publish_path").as_bool();
    publish_cloud_ = get_parameter("publish_cloud").as_bool();

    if (config_file_.empty()) {
        RCLCPP_ERROR(get_logger(),
            "Parameter 'config_file' is empty. "
            "Set it with: --ros-args -p config_file:=/path/to/config.yaml");
        return false;
    }

    // ── Configure point cloud converter ──────────────────────────────────────
    converter_.LoadFromYAML(config_file_);  // override with YAML values if present
    CloudConvertConfig cc = converter_.Config();
    // converter_.SetConfig(cc);
    imu_converter_.SetIMUCoeff(1.0);  // default (set to 9.81 if using raw accel in m/s²)

    RCLCPP_INFO(get_logger(), "Loading IncLIO config: %s", config_file_.c_str());

    // ── Parse YAML to build LIOConfig ─────────────────────────────────────────
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_file_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load config: %s", e.what());
        return false;
    }

    IncLIO::LIOConfig lio_config;

    // IMU init params
    if (yaml["imu_init_time"])
        lio_config.imu_config.init_time_seconds = yaml["imu_init_time"].as<double>();
    if (yaml["max_static_gyro_var"])
        lio_config.imu_config.max_static_gyro_var = yaml["max_static_gyro_var"].as<double>();
    if (yaml["max_static_acce_var"])
        lio_config.imu_config.max_static_acce_var = yaml["max_static_acce_var"].as<double>();

    // IMU-LiDAR extrinsics
    if (yaml["mapping"]) {
        auto ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        auto ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
        IncLIO::Vec3d lidar_T = IncLIO::math::VecFromArray(ext_t);
        IncLIO::Mat3d lidar_R = IncLIO::math::MatFromArray(ext_r);
        lio_config.T_imu_lidar = IncLIO::SE3(lidar_R, lidar_T);
    } else {
        RCLCPP_WARN(get_logger(),
            "No 'mapping' section in config — assuming identity extrinsics");
        lio_config.T_imu_lidar = IncLIO::SE3();
    }

    // ── Construct and initialise LIO ──────────────────────────────────────────
    lio_ = std::make_unique<IncLIO::LIO>(lio_config);
    if (!lio_->Init(config_file_)) {
        RCLCPP_ERROR(get_logger(), "LIO::Init() failed");
        return false;
    }

    // ── Prepare local map (for visualization) ─────────────────────────────────
    local_map_.reset(new IncLIO::PointCloudType);
     RCLCPP_INFO(get_logger(), "IncLIO initialized successfully");

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// CreateSubscriptions
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::CreateSubscriptions() {
    // Large IMU queue: at 200 Hz, 1000 entries = 5 s of headroom.
    auto imu_qos = rclcpp::SensorDataQoS();
    imu_qos.keep_last(1000);

    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_cb_group_;

    rclcpp::SubscriptionOptions lidar_opts;
    lidar_opts.callback_group = lidar_cb_group_;

    // IMU
    std::string imu_topic = get_parameter("imu_topic").as_string();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos,
        [this](sensor_msgs::msg::Imu::UniquePtr msg) { ImuCallback(std::move(msg)); },
        imu_opts);

    RCLCPP_INFO(get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());

    // LiDAR — PointCloud2
    const auto cloud_qos = rclcpp::SensorDataQoS();
    std::string lidar_topic = get_parameter("lidar_topic").as_string();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, cloud_qos,
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { CloudCallback(std::move(msg)); },
        lidar_opts);

    RCLCPP_INFO(get_logger(), "Subscribing to PointCloud2: %s", lidar_topic.c_str());

#ifdef HAVE_LIVOX_ROS_DRIVER2
    livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "~/points_livox", cloud_qos,
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { LivoxCallback(msg); },
        lidar_opts);

    RCLCPP_INFO(get_logger(), "Subscribing to Livox CustomMsg: ~/points_livox");
#endif

      timer_ = this->create_wall_timer(
      50ms, std::bind(&LioNode::ui_callback, this), timer_group_);

}

// ─────────────────────────────────────────────────────────────────────────────
// CreatePublishers
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::CreatePublishers() {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
    path_pub_  = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    if (publish_cloud_) {
        cloud_world_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud_world", 5);
    }

    path_msg_.header.frame_id = world_frame_;
}

// ─────────────────────────────────────────────────────────────────────────────
// ImuCallback  (runs on imu_cb_group_ — never blocked by LiDAR processing)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::ImuCallback(sensor_msgs::msg::Imu::UniquePtr msg) {
    IMUPtr imu_out;
    imu_converter_.Process(*msg, imu_out);

    if (!imu_out) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Invalid IMU message after conversion");
        return;
    }

    // Buffer only — cloud callback drains this into LIO before each scan
    std::lock_guard<std::mutex> lock(imu_buf_mutex_);
    imu_buffer_.push_back(imu_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// CloudCallback  (sensor_msgs/PointCloud2)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::CloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    IncLIO::FullCloudPtr pcl_out;
    converter_.Process(*msg, pcl_out);

    if (!pcl_out || pcl_out->empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Empty or unconvertible PointCloud2 received");
        return;
    }

    const double ts = msg->header.stamp.sec +
                      msg->header.stamp.nanosec * 1e-9;

    ProcessCloud(pcl_out, ts, msg->header.stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
// LivoxCallback  (livox_ros_driver2/CustomMsg)
// ─────────────────────────────────────────────────────────────────────────────
#ifdef HAVE_LIVOX_ROS_DRIVER2
void LioNode::LivoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    auto cloud = converter_.Convert(*msg);
    if (!cloud || cloud->empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Empty Livox CustomMsg received");
        return;
    }

    double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    ProcessCloud(cloud, ts, msg->header.stamp);
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// ProcessCloud  (runs on lidar_cb_group_ — heavy work happens here)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::ProcessCloud(IncLIO::FullCloudPtr cloud, double ts,
                            const rclcpp::Time& stamp)
{
    // Drain all buffered IMU into LIO first (brief lock — doesn't block IMU
    // callbacks for long since we just swap-and-clear).
    {
        std::deque<IMUPtr> local_buf;
        {
            std::lock_guard<std::mutex> lock(imu_buf_mutex_);
            local_buf.swap(imu_buffer_);
        }
        for (auto& imu : local_buf) {
            lio_->AddIMU(imu);
        }
    }

    // Feed the cloud — triggers the full LIO pipeline
    lio_->AddCloud(cloud, ts);

    if (!lio_->IsInitialized()) return;

    auto pose  = lio_->GetCurrentPose();
    auto state = lio_->GetCurrentState();

    PublishOdometry(stamp, pose, state);
    if (publish_path_)  PublishPath(stamp, pose);
    if (publish_cloud_) PublishCloud(stamp, pose);
    if (publish_tf_)    PublishTF(stamp, pose);
}

// ─────────────────────────────────────────────────────────────────────────────
// PublishOdometry
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::PublishOdometry(const rclcpp::Time& stamp,
                               const IncLIO::SE3& pose,
                               const IncLIO::Stated& state)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id  = body_frame_;

    // Pose
    const auto& t = pose.translation();
    const auto  q = pose.unit_quaternion();
    odom.pose.pose.position.x    = t.x();
    odom.pose.pose.position.y    = t.y();
    odom.pose.pose.position.z    = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Linear velocity in body frame
    // state.v_ is in world frame; rotate into body frame for twist
    IncLIO::Vec3d v_body = pose.so3().inverse() * state.v_;
    odom.twist.twist.linear.x = v_body.x();
    odom.twist.twist.linear.y = v_body.y();
    odom.twist.twist.linear.z = v_body.z();

    odom_pub_->publish(odom);
}

// ─────────────────────────────────────────────────────────────────────────────
// PublishPath
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::PublishPath(const rclcpp::Time& stamp, const IncLIO::SE3& pose) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = stamp;
    ps.header.frame_id = world_frame_;

    const auto& t = pose.translation();
    const auto  q = pose.unit_quaternion();
    ps.pose.position.x    = t.x();
    ps.pose.position.y    = t.y();
    ps.pose.position.z    = t.z();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(ps);
    path_pub_->publish(path_msg_);
}

// ─────────────────────────────────────────────────────────────────────────────
// PublishCloud
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::PublishCloud(const rclcpp::Time& stamp, const IncLIO::SE3& pose) {
    auto scan = lio_->GetCurrentScan();
    if (!scan || scan->empty()) return;

    if(lio_->IsKeyframe(pose)) {
        std::lock_guard<std::mutex> lock(keyframe_mutex_);
        if(keyframe_queue_.size()>20) {
            keyframe_queue_.pop_front();
        }
        keyframe_queue_.push_back(Keyframe{pose, scan});
     }

}

// ─────────────────────────────────────────────────────────────────────────────
// PublishTF
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::PublishTF(const rclcpp::Time& stamp, const IncLIO::SE3& pose) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp;
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id  = body_frame_;

    const auto& t = pose.translation();
    const auto  q = pose.unit_quaternion();
    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = t.z();
    tf_msg.transform.rotation.x    = q.x();
    tf_msg.transform.rotation.y    = q.y();
    tf_msg.transform.rotation.z    = q.z();
    tf_msg.transform.rotation.w    = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// ui_callback  (runs on timer_group_ — doesn't interfere with IMU or LiDAR callbacks)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::ui_callback() {

    if(keyframe_queue_.empty()) return;

    local_map_->clear();

    std::deque<Keyframe> local_copy;

    {
        std::lock_guard<std::mutex> lock(keyframe_mutex_);
        local_copy = keyframe_queue_;
    }

    for(auto keyframe_queue_it = local_copy.begin(); keyframe_queue_it!=local_copy.end(); keyframe_queue_it++) {
        auto pose = (*keyframe_queue_it).pose;
        auto scan = (*keyframe_queue_it).cloud;

        IncLIO::CloudPtr scan_world(new IncLIO::PointCloudType());
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear()      = pose.rotationMatrix();
        T.translation() = pose.translation();
        pcl::transformPointCloud(*scan, *scan_world, T.matrix().cast<float>());
        *local_map_+=*scan_world;
    }

    sensor_msgs::msg::PointCloud2 world_msg;
    pcl::toROSMsg(*local_map_, world_msg);
    world_msg.header.stamp    = rclcpp::Clock().now();
    world_msg.header.frame_id = world_frame_;
    cloud_world_pub_->publish(world_msg);

}

} // namespace inclio_ros2

// ─────────────────────────────────────────────────────────────────────────────
// Register as a composable component
// ─────────────────────────────────────────────────────────────────────────────
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(inclio_ros2::LioNode)
