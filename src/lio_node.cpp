// lio_node.cpp
//
// ROS2 composable node that wraps IncLIO::LIO.
// See lio_node.hpp for the full interface description.

#include "ros2_wrapper/lio_node.hpp"

#include "ndt/ndt_map.hpp"
#include "ndt/ndt_registration.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <pcl/io/pcd_io.h>
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

    // Start the viz worker thread outside the executor pool.
    // It owns voxel downsample + crop + serialize + publish so ui_callback stays < 5 ms.
    viz_worker_ = std::thread(&LioNode::VizWorkerLoop, this);

    RCLCPP_INFO(get_logger(), "IncLIO ROS2 node ready (multi-threaded)");
}


LioNode::~LioNode() {
    // Stop the viz worker before destroying shared resources.
    {
        std::lock_guard<std::mutex> lk(viz_cv_mutex_);
        viz_worker_stop_ = true;
    }
    viz_cv_.notify_one();
    if (viz_worker_.joinable()) viz_worker_.join();

    // Stop the loop-closure worker (if started).
    {
        std::lock_guard<std::mutex> lk(loop_cv_mutex_);
        loop_worker_stop_ = true;
    }
    loop_cv_.notify_one();
    if (loop_worker_.joinable()) loop_worker_.join();

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

    // // Point cloud conversion (mirrors run_bag.cc CLI / YAML options)
    // declare_parameter<int>   ("lidar_type",        4);     // 1=Livox, 2=Velodyne, 3=Ouster
    // declare_parameter<int>   ("num_scans",         128);
    // declare_parameter<double>("time_scale",        1e-3);
    // declare_parameter<int>   ("point_filter_num",  1);

    // Frame ids
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("body_frame",  "body");

    // Feature toggles
    declare_parameter<bool>("publish_tf",    true);
    declare_parameter<bool>("publish_path",  true);
    declare_parameter<bool>("publish_cloud", true);

    // Map visualization
    declare_parameter<double>("map_voxel_size",    0.2);   // voxel leaf size for full_map_ and viz window
    declare_parameter<double>("body_crop_radius", 1.0);   // remove points closer than this to the sensor (m)
    declare_parameter<double>("publish_radius", 80.0);    // crop radius around current pose
    declare_parameter<double>("publish_rate_hz", 5.0);    // ~/cloud_world publish rate
    declare_parameter<int>("local_map_scans", 20);        // sliding window size for ~/cloud_world

    // Loop closure (Phase A: detection only — see doc/loop_closure_steps.txt)
    declare_parameter<bool>  ("loop_closure.enable",              false);
    declare_parameter<double>("loop_closure.pc_max_radius",       80.0);  // m
    declare_parameter<double>("loop_closure.lidar_height",        1.73);  // m above ground
    declare_parameter<double>("loop_closure.pc_max_z",            6.0);   // m above ground
    declare_parameter<int>   ("loop_closure.min_keyframe_gap",    30);
    declare_parameter<double>("loop_closure.sc_dist_thres",       0.6);
    declare_parameter<double>("loop_closure.max_spatial_dist",    25.0);
    declare_parameter<double>("loop_closure.search_frequency_hz", 1.0);
    declare_parameter<int>   ("loop_closure.pc_num_ring",         20);
    declare_parameter<int>   ("loop_closure.pc_num_sector",       60);
    declare_parameter<int>   ("loop_closure.pc_num_z",            6);
    declare_parameter<int>   ("loop_closure.kf_search_num",       25);   // submap size
    declare_parameter<double>("loop_closure.voxel_size",          1.0);  // m
    declare_parameter<int>   ("loop_closure.verify_max_iter",     25);
    declare_parameter<int>   ("loop_closure.verify_min_eff_pts",  200);
    declare_parameter<double>("loop_closure.verify_mean_res_thres", 1.0);
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
    map_voxel_size_   = get_parameter("map_voxel_size").as_double();
    body_crop_radius_ = get_parameter("body_crop_radius").as_double();
    publish_radius_ = get_parameter("publish_radius").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    full_map_      = std::make_shared<IncLIO::PointCloudType>();
    viz_max_scans_ = get_parameter("local_map_scans").as_int();

    loop_closure_enabled_   = get_parameter("loop_closure.enable").as_bool();
    lc_pc_max_radius_       = get_parameter("loop_closure.pc_max_radius").as_double();
    lc_lidar_height_        = get_parameter("loop_closure.lidar_height").as_double();
    lc_pc_max_z_            = get_parameter("loop_closure.pc_max_z").as_double();
    lc_min_keyframe_gap_    = get_parameter("loop_closure.min_keyframe_gap").as_int();
    lc_sc_dist_thres_       = get_parameter("loop_closure.sc_dist_thres").as_double();
    lc_max_spatial_dist_    = get_parameter("loop_closure.max_spatial_dist").as_double();
    lc_search_frequency_hz_ = get_parameter("loop_closure.search_frequency_hz").as_double();
    if (loop_closure_enabled_) {
        IncLIO::NdtDescriptorConfig dcfg;
        dcfg.pc_num_ring   = get_parameter("loop_closure.pc_num_ring").as_int();
        dcfg.pc_num_sector = get_parameter("loop_closure.pc_num_sector").as_int();
        dcfg.pc_num_z      = get_parameter("loop_closure.pc_num_z").as_int();
        dcfg.pc_max_radius = lc_pc_max_radius_;
        dcfg.pc_max_z      = lc_pc_max_z_;
        descriptor_builder_ = IncLIO::NdtDescriptor(dcfg);

        lc_kf_search_num_              = get_parameter("loop_closure.kf_search_num").as_int();
        kf_voxelizer_cfg_.voxel_size   = get_parameter("loop_closure.voxel_size").as_double();
        kf_voxelizer_cfg_.max_radius   = lc_pc_max_radius_;
        kf_voxelizer_cfg_.lidar_height = lc_lidar_height_;
        kf_voxelizer_cfg_.max_z        = lc_pc_max_z_;
        lc_verify_max_iter_            = get_parameter("loop_closure.verify_max_iter").as_int();
        lc_verify_min_eff_pts_         = get_parameter("loop_closure.verify_min_eff_pts").as_int();
        lc_verify_mean_res_thres_      = get_parameter("loop_closure.verify_mean_res_thres").as_double();

        loop_worker_ = std::thread(&LioNode::LoopWorkerLoop, this);
        RCLCPP_INFO(get_logger(),
            "Loop closure: enabled (radius=%.1fm, lidar_height=%.2fm, max_z=%.2fm)",
            lc_pc_max_radius_, lc_lidar_height_, lc_pc_max_z_);
        RCLCPP_INFO(get_logger(),
            "Loop closure: descriptor %dx%d (R=%d S=%d Z=%d, %d cells)",
            2 * dcfg.pc_num_ring, dcfg.pc_num_sector,
            dcfg.pc_num_ring, dcfg.pc_num_sector, dcfg.pc_num_z,
            2 * dcfg.pc_num_ring * dcfg.pc_num_sector);
        RCLCPP_INFO(get_logger(),
            "Loop closure: submap kf_search_num=%d voxel_size=%.2fm",
            lc_kf_search_num_, kf_voxelizer_cfg_.voxel_size);
        RCLCPP_INFO(get_logger(),
            "Loop closure: gates min_gap=%d sc_thres=%.2f spatial_thres=%.1fm search_hz=%.1f",
            lc_min_keyframe_gap_, lc_sc_dist_thres_,
            lc_max_spatial_dist_, lc_search_frequency_hz_);
    }

    // Body-frame CropBox: removes points within body_crop_radius_ of the sensor origin.
    // setNegative(true) inverts the filter — points INSIDE the box are discarded,
    // keeping only returns beyond the robot/vehicle body.
    const float r = static_cast<float>(body_crop_radius_);
    scan_crop_.setMin(Eigen::Vector4f(-r, -r, -r, 1.0f));
    scan_crop_.setMax(Eigen::Vector4f( r,  r,  r, 1.0f));
    scan_crop_.setNegative(true);


    if (config_file_.empty()) {
        RCLCPP_ERROR(get_logger(),
            "Parameter 'config_file' is empty. "
            "Set it with: --ros-args -p config_file:=/path/to/config.yaml");
        return false;
    }

    // ── Configure point cloud converter ──────────────────────────────────────
    converter_.LoadFromYAML(config_file_);  // override with YAML values if present
    cc = converter_.Config();
    // converter_.SetConfig(cc);
    imu_converter_.SetIMUCoeff(cc.imu_coeff);  // default (set to 9.81 if using raw accel in m/s²)

    RCLCPP_INFO(get_logger(), "Loading IncLIO config: %s", config_file_.c_str());

    // Print parameters for verification
    std::string lidar_type_str_;
    lidar_type_str_ = (cc.lidar_type == LidarType::LIVOX) ? "LIVOX" :
                      (cc.lidar_type == LidarType::VELO32) ? "VELO32" :
                      (cc.lidar_type == LidarType::OUST64) ? "OUST64" :
                      (cc.lidar_type == LidarType::HESAI) ? "HESAI" : "UNKNOWN";
    RCLCPP_INFO(get_logger(), "Config parameters:");
    RCLCPP_INFO(get_logger(), "  lidar_type: %s", lidar_type_str_.c_str());
    RCLCPP_INFO(get_logger(), "  num_scans: %d", cc.num_scans);
    RCLCPP_INFO(get_logger(), "  time_scale: %f", cc.time_scale);
    RCLCPP_INFO(get_logger(), "  point_filter_num: %d", cc.point_filter_num);
    RCLCPP_INFO(get_logger(), "  map_voxel_size: %f", map_voxel_size_);


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
    if (yaml["use_static_init"])
        lio_config.imu_config.use_static_init = yaml["use_static_init"].as<bool>();
    if (yaml["imu_init_time"])
        lio_config.imu_config.init_time_seconds = yaml["imu_init_time"].as<double>();
    if (yaml["max_static_gyro_var"])
        lio_config.imu_config.max_static_gyro_var = yaml["max_static_gyro_var"].as<double>();
    if (yaml["max_static_acce_var"])
        lio_config.imu_config.max_static_acce_var = yaml["max_static_acce_var"].as<double>();
    if (yaml["prior_bg"]) {
        auto v = yaml["prior_bg"].as<std::vector<double>>();
        lio_config.imu_config.prior_bg = IncLIO::Vec3d(v[0], v[1], v[2]);
    }
    if (yaml["prior_ba"]) {
        auto v = yaml["prior_ba"].as<std::vector<double>>();
        lio_config.imu_config.prior_ba = IncLIO::Vec3d(v[0], v[1], v[2]);
    }
    if (yaml["prior_gravity"]) {
        auto v = yaml["prior_gravity"].as<std::vector<double>>();
        lio_config.imu_config.prior_gravity = IncLIO::Vec3d(v[0], v[1], v[2]);
    }

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

    if (yaml["use_ct_undistort"])
        lio_config.use_ct_undistort = yaml["use_ct_undistort"].as<bool>();

    // ── Construct and initialise LIO ──────────────────────────────────────────
    lio_ = std::make_unique<IncLIO::LIO>(lio_config);
    if (!lio_->Init(config_file_)) {
        RCLCPP_ERROR(get_logger(), "LIO::Init() failed");
        return false;
    }

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
    if(cc.lidar_type != LidarType::LIVOX)
    {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, cloud_qos,
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { CloudCallback(std::move(msg)); },
        lidar_opts);

    RCLCPP_INFO(get_logger(), "Subscribing to PointCloud2: %s", lidar_topic.c_str());
    }
    else 
    {
    #ifdef HAVE_LIVOX_ROS_DRIVER2
        livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic, cloud_qos,
            [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { LivoxCallback(msg); },
            lidar_opts);

            RCLCPP_INFO(get_logger(), "Subscribing to custom msg: %s", lidar_topic.c_str());

    #endif
    }
    const double hz = std::max(0.5, publish_rate_hz_);
    const auto period_ns = std::chrono::nanoseconds(
        static_cast<int64_t>(1.0e9 / hz));
    timer_ = this->create_wall_timer(
        period_ns, std::bind(&LioNode::ui_callback, this), timer_group_);

}

// ─────────────────────────────────────────────────────────────────────────────
// CreatePublishers
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::CreatePublishers() {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
    odom_fast_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry_fast", 50);
    path_pub_  = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    if (publish_cloud_) {
        cloud_world_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud_world", 5);
    }

    if (loop_closure_enabled_) {
        loop_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/loop_closures", 10);
    }

    path_msg_.header.frame_id = world_frame_;

    save_map_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/save_map",
        std::bind(&LioNode::SaveMapCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

// ─────────────────────────────────────────────────────────────────────────────
// ImuCallback  (runs on imu_cb_group_ — never blocked by LiDAR processing)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::ImuCallback(sensor_msgs::msg::Imu::UniquePtr msg) {
    RCLCPP_DEBUG(get_logger(), "Received IMU at time %.3f", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    IMUPtr imu_out;
    imu_converter_.Process(*msg, imu_out);

    if (!imu_out) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Invalid IMU message after conversion");
        return;
    }

    if (lio_ && lio_->IsInitialized()) {
        // After init: feed IMU directly into LIO (for MessageSync + high-rate
        // propagation), then publish the propagated pose at IMU rate.
        lio_->AddIMU(imu_out);

        auto state = lio_->GetPropagatedState();
        auto pose = state.GetSE3();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = msg->header.stamp;
        odom.header.frame_id = world_frame_;
        odom.child_frame_id  = body_frame_;

        const auto& t = pose.translation();
        const auto  q = pose.unit_quaternion();
        odom.pose.pose.position.x    = t.x();
        odom.pose.pose.position.y    = t.y();
        odom.pose.pose.position.z    = t.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        IncLIO::Vec3d v_body = pose.so3().inverse() * state.v_;
        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();

        odom_fast_pub_->publish(odom);

        // TF at IMU rate — smooth transform for RViz and downstream nodes
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = msg->header.stamp;
            tf_msg.header.frame_id = world_frame_;
            tf_msg.child_frame_id  = body_frame_;
            tf_msg.transform.translation.x = t.x();
            tf_msg.transform.translation.y = t.y();
            tf_msg.transform.translation.z = t.z();
            tf_msg.transform.rotation.x    = q.x();
            tf_msg.transform.rotation.y    = q.y();
            tf_msg.transform.rotation.z    = q.z();
            tf_msg.transform.rotation.w    = q.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }
    } else {
        // Before init: buffer only — ProcessCloud drains these into LIO
        std::lock_guard<std::mutex> lock(imu_buf_mutex_);
        imu_buffer_.push_back(imu_out);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CloudCallback  (sensor_msgs/PointCloud2)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::CloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    RCLCPP_DEBUG(get_logger(), "Received PointCloud2 with %u points", msg->width * msg->height);
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
    IncLIO::FullCloudPtr pcl_out;
    converter_.Process(*msg,pcl_out);
    if (!pcl_out || pcl_out->empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Empty Livox CustomMsg received");
        return;
    }

    double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    ProcessCloud(pcl_out, ts, msg->header.stamp);
}

#endif

// ─────────────────────────────────────────────────────────────────────────────
// ProcessCloud  (runs on lidar_cb_group_ — heavy work happens here)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::ProcessCloud(IncLIO::FullCloudPtr cloud, double ts,
                            const rclcpp::Time& stamp)
{
    // Before initialization, ImuCallback only buffers (doesn't call AddIMU).
    // Drain any buffered IMUs into LIO here. After init, ImuCallback feeds
    // AddIMU directly, so the buffer stays empty and this is a no-op.
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
    if (publish_cloud_ && (lio_->WasKeyframe() || lio_->frame_num() % 10 == 0)) PublishCloud(stamp, pose);

    // ── Loop closure: capture a Keyframe on every keyframe event ─────────────
    // LIDAR thread only stashes the raw scan + pose. Voxelization and
    // descriptor build happen on the loop thread once the N-neighbor submap
    // can be assembled — keeps this thread off the critical path and makes
    // the descriptor inputs symmetric forward/reverse.
    if (loop_closure_enabled_ && lio_->WasKeyframe()) {
        auto kf = std::make_shared<IncLIO::Keyframe>();
        kf->id              = next_keyframe_id_++;
        kf->timestamp       = ts;
        kf->T_W_L           = pose;
        kf->raw_scan_lidar  = lio_->GetCurrentScan();
        {
            std::lock_guard<std::mutex> lock(keyframe_queue_mutex_);
            keyframe_queue_.push_back(std::move(kf));
        }
        loop_cv_.notify_one();
    }
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
void LioNode::PublishCloud(const rclcpp::Time& /*stamp*/, const IncLIO::SE3& pose) {
    auto scan = lio_->GetCurrentScan();
    if (!scan || scan->empty()) return;

    // Cheap enqueue — transform + voxel insert happens on timer thread
    std::lock_guard<std::mutex> lock(scan_queue_mutex_);
    scan_queue_.push_back({scan, pose});
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
// ui_callback  (runs on timer_group_)
// ─────────────────────────────────────────────────────────────────────────────
// Fast path: drain scan_queue_, transform to world frame, voxel filter each scan
// individually (small extent per scan → never overflows PCL int32), then accumulate
// into full_map_ and push into the viz sliding window.
// Following DLIO's pattern: filter-per-scan, not filter-the-whole-map.
void LioNode::ui_callback() {
    std::deque<ScanEntry> pending;
    {
        std::lock_guard<std::mutex> lock(scan_queue_mutex_);
        pending.swap(scan_queue_);
    }
    if (pending.empty()) return;

    // Transform to world frame, then voxel filter each scan individually.
    // Per-scan filtering is safe (one scan spans tens of meters, not the full session).
    std::vector<IncLIO::CloudPtr> scans_world;
    scans_world.reserve(pending.size());
    for (const auto& entry : pending) {
        // 1. Body-frame near-field removal — strips the robot/vehicle body returns.
        IncLIO::CloudPtr sw_body(new IncLIO::PointCloudType());
        scan_crop_.setInputCloud(entry.cloud);
        scan_crop_.filter(*sw_body);

        // 2. Transform to world frame.
        auto sw = std::make_shared<IncLIO::PointCloudType>();
        IncLIO::transformCloudOMP(*sw_body, *sw, entry.pose.matrix().cast<float>());

        // 3. Voxel downsample before accumulating.
        auto sw_filtered = std::make_shared<IncLIO::PointCloudType>();
        pcl::VoxelGrid<IncLIO::PointType> vf;
        vf.setInputCloud(sw);
        const float leaf = static_cast<float>(map_voxel_size_);
        vf.setLeafSize(leaf, leaf, leaf);
        vf.filter(*sw_filtered);

        scans_world.push_back(sw_filtered);
        current_pose_ = entry.pose;
    }

    // Append pre-filtered scans to full_map_.
    {
        std::lock_guard<std::mutex> lock(full_map_mutex_);
        for (const auto& sw : scans_world)
            *full_map_ += *sw;
    }

    // Push pre-filtered scans into the viz sliding window — try_lock so we never block on the worker.
    if (viz_map_mutex_.try_lock()) {
        for (const auto& sw : scans_world)
            viz_scan_window_.push_back(sw);
        while (static_cast<int>(viz_scan_window_.size()) > viz_max_scans_)
            viz_scan_window_.pop_front();
        viz_map_mutex_.unlock();
    }

    // Wake the viz worker. Pass the crop centre under the cv mutex.
    {
        std::lock_guard<std::mutex> lk(viz_cv_mutex_);
        viz_work_ready_  = true;
        viz_crop_center_ = current_pose_;
    }
    viz_cv_.notify_one();
}

// ─────────────────────────────────────────────────────────────────────────────
// VizWorkerLoop  (dedicated std::thread, not in the executor pool)
// ─────────────────────────────────────────────────────────────────────────────
// Owns the slow path: concatenate viz_scan_window_, voxel downsample,
// radius crop, pcl::toROSMsg, publish.
// Woken by ui_callback via viz_cv_. Signals are coalesced — no cycle is
// permanently lost, just merged if the worker is still running.
void LioNode::VizWorkerLoop() {
    while (true) {
        IncLIO::SE3 crop_center;
        {
            std::unique_lock<std::mutex> lk(viz_cv_mutex_);
            viz_cv_.wait(lk, [this] { return viz_work_ready_ || viz_worker_stop_; });
            if (viz_worker_stop_) break;
            viz_work_ready_ = false;
            crop_center = viz_crop_center_;
        }

        if (!publish_cloud_ || !cloud_world_pub_) continue;

        // const auto& t = crop_center.translation();
        // const float cx = static_cast<float>(t.x());
        // const float cy = static_cast<float>(t.y());
        // const float cz = static_cast<float>(t.z());
        // const float r2 = static_cast<float>(publish_radius_ * publish_radius_);

        // Snapshot the sliding window under lock, then release so ui_callback can push.
        IncLIO::CloudPtr raw(new IncLIO::PointCloudType());
        {
            std::lock_guard<std::mutex> lock(viz_map_mutex_);
            for (const auto& scan : viz_scan_window_)
                *raw += *scan;
        }

        // Radius crop first — bounds the spatial extent before voxel filtering.
        // IncLIO::CloudPtr cropped(new IncLIO::PointCloudType());
        // {
        //     cropped->points.reserve(raw->points.size());
        //     for (const auto& pt : raw->points) {
        //         const float dx = pt.x - cx;
        //         const float dy = pt.y - cy;
        //         const float dz = pt.z - cz;
        //         if (dx*dx + dy*dy + dz*dz <= r2)
        //             cropped->points.push_back(pt);
        //     }
        // }

        // Light voxel pass to merge overlapping voxels between adjacent scans in the window.
        // Each scan is already pre-filtered at map_voxel_size_, and the extent is bounded
        // by publish_radius_, so this will not overflow.
        // IncLIO::CloudPtr local_map(new IncLIO::PointCloudType());
        // if (!cropped->empty()) {
        //     pcl::VoxelGrid<IncLIO::PointType> vf;
        //     vf.setInputCloud(cropped);
        //     const float leaf = static_cast<float>(publish_voxel_size_);
        //     vf.setLeafSize(leaf, leaf, leaf);
        //     vf.filter(*local_map);
        // }

        raw->width    = raw->points.size();
        raw->height   = 1;
        raw->is_dense = true;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*raw, msg);
        msg.header.stamp    = now();
        msg.header.frame_id = world_frame_;
        cloud_world_pub_->publish(msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LoopWorkerLoop  (loop_worker_ thread — descriptor build + candidate search)
// ─────────────────────────────────────────────────────────────────────────────
// Drains keyframe_queue_, builds the NDTMC descriptor for each new keyframe
// and appends it to keyframe_db_. At lc_search_frequency_hz_, runs a top-K
// search of the current keyframe against the historical DB (with a min-gap
// guard). Phase A only logs candidates — Step 6 adds ICP verification, Step 7
// adds RViz markers.
void LioNode::LoopWorkerLoop() {
    using clock = std::chrono::steady_clock;
    auto last_search = clock::now();
    const auto search_period = std::chrono::milliseconds(
        std::max(1, static_cast<int>(1000.0 / lc_search_frequency_hz_)));

    while (true) {
        {
            std::unique_lock<std::mutex> lk(loop_cv_mutex_);
            loop_cv_.wait_for(lk, std::chrono::milliseconds(200),
                              [this] { return loop_worker_stop_; });
            if (loop_worker_stop_) break;
        }

        // Drain pending keyframes (cheap swap under queue lock).
        std::deque<IncLIO::KeyframePtr> pending;
        {
            std::lock_guard<std::mutex> qlock(keyframe_queue_mutex_);
            pending.swap(keyframe_queue_);
        }

        // Build the N-neighbor submap, voxelize fresh, and compute the
        // descriptor for each pending keyframe before publishing into the DB.
        for (auto& kf : pending) {
            if (!kf->raw_scan_lidar) continue;

            // Snapshot up to lc_kf_search_num_ predecessors. Cheap shared_ptr
            // copies under the DB lock; descriptor work happens lock-free.
            std::vector<IncLIO::KeyframePtr> neighbors;
            {
                std::lock_guard<std::mutex> dblock(keyframe_db_mutex_);
                const int n_db   = static_cast<int>(keyframe_db_.size());
                const int start  = std::max(0, n_db - lc_kf_search_num_);
                neighbors.assign(keyframe_db_.begin() + start, keyframe_db_.end());
            }

            // Transform each neighbor scan into the target keyframe's LiDAR
            // frame. The target's own scan is already there.
            std::vector<IncLIO::CloudPtr> submap;
            submap.reserve(neighbors.size() + 1);
            submap.push_back(kf->raw_scan_lidar);

            const Eigen::Matrix4d T_target_inv = kf->T_W_L.inverse().matrix();
            for (const auto& n : neighbors) {
                if (!n->raw_scan_lidar) continue;
                const Eigen::Matrix4f T_target_n =
                    (T_target_inv * n->T_W_L.matrix()).cast<float>();
                auto transformed = std::make_shared<IncLIO::PointCloudType>();
                IncLIO::transformCloudOMP(*n->raw_scan_lidar, *transformed, T_target_n);
                submap.push_back(transformed);
            }

            IncLIO::BuildKeyframeVoxels(submap, kf_voxelizer_cfg_,
                                         kf->map_voxels_local);

            descriptor_builder_.Build(kf->map_voxels_local,
                                       kf->descriptor, kf->histogram);

            RCLCPP_DEBUG(get_logger(),
                "Loop kf=%d: submap=%zu scans, voxels=%zu",
                kf->id, submap.size(), kf->map_voxels_local.size());

            std::lock_guard<std::mutex> dblock(keyframe_db_mutex_);
            keyframe_db_.push_back(kf);
            keyframe_descriptors_.push_back(kf->descriptor);
        }

        if (clock::now() - last_search >= search_period) {
            last_search = clock::now();
            DetectLoopClosure();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DetectLoopClosure  (loop_worker_ thread — top-K candidate scan)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::DetectLoopClosure() {
    int cur_idx = -1;
    int cur_id  = -1;
    IncLIO::SE3 cur_pose;
    Eigen::MatrixXd cur_desc;

    // Snapshot the comparable prefix of the DB so the search runs lock-free.
    std::vector<Eigen::MatrixXd> historical;
    {
        std::lock_guard<std::mutex> dblock(keyframe_db_mutex_);
        if (keyframe_db_.empty()) return;
        cur_idx = static_cast<int>(keyframe_db_.size()) - 1;
        if (cur_idx < lc_min_keyframe_gap_) return;
        cur_desc = keyframe_descriptors_[cur_idx];
        cur_id   = keyframe_db_[cur_idx]->id;
        cur_pose = keyframe_db_[cur_idx]->T_W_L;
        const int prefix = cur_idx - lc_min_keyframe_gap_;
        historical.assign(keyframe_descriptors_.begin(),
                          keyframe_descriptors_.begin() + prefix);
    }

    if (historical.empty()) return;

    // Pass 1 — descriptor distance gate. O(eligible) Distance() calls.
    // Track the best score regardless of threshold so the per-tick log can
    // show "the closest match this tick was X" — useful when desc_pass=0 to
    // tell apart "marginal miss" (0.61) from "completely different" (0.95).
    int    desc_pass    = 0;
    double min_dist     = std::numeric_limits<double>::infinity();
    int    min_dist_idx = -1;
    std::vector<std::tuple<double, int, int>> scored;
    scored.reserve(historical.size());
    for (int i = 0; i < static_cast<int>(historical.size()); ++i) {
        auto [dist, shift] = IncLIO::NdtDescriptor::Distance(cur_desc, historical[i]);
        if (dist < min_dist) { min_dist = dist; min_dist_idx = i; }
        if (dist > lc_sc_dist_thres_) continue;
        ++desc_pass;
        scored.emplace_back(dist, i, shift);
    }

    // Pass 2 — spatial gate. Applied to ALL desc-passed candidates (not just
    // top-K), so the diagnostic counters show how many places "look similar"
    // vs. how many "look similar AND are physically near".
    std::vector<LoopCandidateViz> filtered;
    filtered.reserve(scored.size());
    {
        std::lock_guard<std::mutex> dblock(keyframe_db_mutex_);
        for (const auto& [dist, cand_idx, shift] : scored) {
            if (cand_idx < 0 || cand_idx >= static_cast<int>(keyframe_db_.size())) continue;
            const IncLIO::SE3& cand_pose = keyframe_db_[cand_idx]->T_W_L;
            const double spatial =
                (cur_pose.translation() - cand_pose.translation()).norm();
            if (spatial > lc_max_spatial_dist_) continue;
            filtered.push_back({cand_pose, keyframe_db_[cand_idx]->id,
                                dist, shift});
        }
    }

    // Capture spatial_pass before reducing to the single best — it's the more
    // useful stat (how many places passed BOTH gates, not just the winner).
    const int spatial_pass = static_cast<int>(filtered.size());

    // Single best by descriptor distance — matches NDTMC-LIO-SAM
    // detectLoopClosureID, which tracks only `if (candidate_dist < min_dist)`.
    if (filtered.size() > 1) {
        auto best_it = std::min_element(
            filtered.begin(), filtered.end(),
            [](const auto& a, const auto& b) { return a.dist < b.dist; });
        const auto best = *best_it;
        filtered.clear();
        filtered.push_back(best);
    }

    // Look up the kf id + spatial distance of the closest match (regardless
    // of whether it passed the gates). This tells you whether the descriptor
    // *almost* matched something and where.
    // int    min_dist_kf_id   = -1;
    // double min_dist_spatial = -1.0;
    // if (min_dist_idx >= 0) {
    //     std::lock_guard<std::mutex> dblock(keyframe_db_mutex_);
    //     if (min_dist_idx < static_cast<int>(keyframe_db_.size())) {
    //         min_dist_kf_id = keyframe_db_[min_dist_idx]->id;
    //         min_dist_spatial =
    //             (cur_pose.translation() -
    //              keyframe_db_[min_dist_idx]->T_W_L.translation()).norm();
    //     }
    // }

    if (filtered.empty()) return;

    // ── Verification: NDT P2D align cur scan against pre-submap NdtMap ──
    // Skip pairs already verified (dedup) — within a 1-second-long detection
    // burst the same (cur_id, pre_id) re-fires; one verification is enough.
    auto& cand = filtered[0];
    {
        std::lock_guard<std::mutex> lk(closed_pairs_mutex_);
        auto range = closed_pairs_.equal_range(cur_id);
        for (auto it = range.first; it != range.second; ++it) {
            if (it->second == cand.id) {
                cand.verified = true;   // already verified previously — re-flag for marker
                break;
            }
        }
    }

    if (!cand.verified) {
        IncLIO::SE3 T_pre_cur;
        double mean_res  = std::numeric_limits<double>::infinity();
        int    eff_num   = 0;
        double shift_t   = 0.0;
        double shift_deg = 0.0;
        const bool ok = VerifyLoopCandidate(cur_id, cand.id,
                                             T_pre_cur, mean_res, eff_num,
                                             shift_t, shift_deg);
        cand.mean_res  = mean_res;
        cand.eff_num   = eff_num;
        cand.shift_t   = shift_t;
        cand.shift_deg = shift_deg;
        cand.verified  = ok;

        if (ok) {
            {
                std::lock_guard<std::mutex> lk(verified_loops_mutex_);
                verified_loops_.push_back({cur_id, cand.id, T_pre_cur,
                                           mean_res, eff_num,
                                           shift_t, shift_deg});
            }
            {
                std::lock_guard<std::mutex> lk(closed_pairs_mutex_);
                closed_pairs_.insert({cur_id, cand.id});
            }
        }
    }

    // Per-tick search summary.
    RCLCPP_INFO(get_logger(),
        "Loop search: db=%d eligible=%d desc_pass=%d emitted=%d "
        "(best: kf=%d desc=%.3f)",
        cur_idx + 1, static_cast<int>(historical.size()),
        desc_pass, static_cast<int>(filtered.size()),
        cand.id, min_dist);

    const double yaw_deg = 360.0 * cand.shift / static_cast<double>(cur_desc.cols());
    if (cand.verified) {
        RCLCPP_INFO(get_logger(),
            "  [VERIFIED]  kf=%d <- kf=%d  desc=%.4f yaw=%.0f deg "
            "(NDT mean_res=%.3f eff=%d shift=%.3fm/%.2fdeg)",
            cur_id, cand.id, cand.dist, yaw_deg,
            cand.mean_res, cand.eff_num, cand.shift_t, cand.shift_deg);
    } else {
        RCLCPP_INFO(get_logger(),
            "  [candidate] kf=%d <- kf=%d  desc=%.4f yaw=%.0f deg "
            "(NDT mean_res=%.3f eff=%d shift=%.3fm/%.2fdeg — rejected)",
            cur_id, cand.id, cand.dist, yaw_deg,
            cand.mean_res, cand.eff_num, cand.shift_t, cand.shift_deg);
    }

    PublishLoopMarkers(cur_pose, cur_id, filtered);
}

// ─────────────────────────────────────────────────────────────────────────────
// VerifyLoopCandidate  (loop_worker_ thread)
// Builds an ephemeral NdtMap from kf_pre + N predecessors (transformed into
// kf_pre's LiDAR frame), then runs NDT P2D from kf_cur's raw scan with the
// relative odometry pose as initial guess. Acceptance: AlignNdt converges,
// ≥ verify_min_eff_pts effective matches, mean residual < verify_mean_res_thres.
// ─────────────────────────────────────────────────────────────────────────────
bool LioNode::VerifyLoopCandidate(int cur_id, int cand_id,
                                  IncLIO::SE3& T_pre_cur_out,
                                  double& mean_res_out,
                                  int& eff_num_out,
                                  double& shift_t_out,
                                  double& shift_deg_out) {
    mean_res_out = std::numeric_limits<double>::infinity();
    eff_num_out  = 0;
    shift_t_out  = 0.0;
    shift_deg_out = 0.0;

    // Snapshot the candidate keyframe + N predecessors under the DB lock.
    // Cheap shared_ptr copies; descriptor compute happens lock-free.
    std::vector<IncLIO::KeyframePtr> target_kfs;
    IncLIO::KeyframePtr cur_kf, pre_kf;
    {
        std::lock_guard<std::mutex> lock(keyframe_db_mutex_);
        const int n_db = static_cast<int>(keyframe_db_.size());
        if (cur_id  < 0 || cur_id  >= n_db) return false;
        if (cand_id < 0 || cand_id >= n_db) return false;
        cur_kf = keyframe_db_[cur_id];
        pre_kf = keyframe_db_[cand_id];

        // Symmetric window [cand_id - N, cand_id + N], clamped to [0, n_db-1].
        // Upper end is also kept strictly below cur_id's neighborhood so the
        // submap can never bleed into the current keyframe's own scans (would
        // cause a trivial self-match). With default N <= lc_min_keyframe_gap_
        // the cur-side clamp is inactive; it only kicks in if N is raised.
        const int start = std::max(0, cand_id - lc_kf_search_num_);
        const int end   = std::min({n_db - 1,
                                    cand_id + lc_kf_search_num_,
                                    cur_id  - lc_min_keyframe_gap_});
        target_kfs.reserve(end - start + 1);
        for (int i = start; i <= end; ++i) {
            target_kfs.push_back(keyframe_db_[i]);
        }
    }
    if (!cur_kf || !pre_kf || !cur_kf->raw_scan_lidar || !pre_kf->raw_scan_lidar) {
        return false;
    }

    // Build target NdtMap in kf_pre's LiDAR frame (capacity disabled — ephemeral).
    IncLIO::NdtMap::Options topts;
    topts.voxel_size       = kf_voxelizer_cfg_.voxel_size;
    topts.min_pts_in_voxel = kf_voxelizer_cfg_.min_pts_in_voxel;
    topts.max_pts_in_voxel = kf_voxelizer_cfg_.max_pts_in_voxel;
    topts.capacity         = std::numeric_limits<size_t>::max();
    IncLIO::NdtMap target_map(topts);

    RCLCPP_INFO(get_logger(),
        "  Building NDT map for candidate kf=%d: submap %zu scans, current kf=%d",
        cand_id, target_kfs.size(), cur_id);

    const Eigen::Matrix4d T_pre_inv = pre_kf->T_W_L.inverse().matrix();
    for (const auto& n : target_kfs) {
        if (!n->raw_scan_lidar) continue;
        const Eigen::Matrix4f T_pre_n =
            (T_pre_inv * n->T_W_L.matrix()).cast<float>();
        auto t = std::make_shared<IncLIO::PointCloudType>();
        IncLIO::transformCloudOMP(*n->raw_scan_lidar, *t, T_pre_n);
        target_map.AddCloud(t);
    }

    if (target_map.NumVoxels() == 0) return false;

    // Initial guess: relative odometry pose (cur expressed in pre's frame).
    const IncLIO::SE3 T_init    = pre_kf->T_W_L.inverse() * cur_kf->T_W_L;
    IncLIO::SE3       T_pre_cur = T_init;

    IncLIO::NdtRegistration::Options ropts;
    ropts.max_iteration     = lc_verify_max_iter_;
    ropts.min_effective_pts = lc_verify_min_eff_pts_;
    IncLIO::NdtRegistration reg(ropts);
    reg.SetSource(cur_kf->raw_scan_lidar);
    reg.SetNdtMap(&target_map);

    RCLCPP_INFO(get_logger(),
        "  target map size : =%d <- cur ky lidar size =%d "
        "(NDT init guess: mean_res=%.3f eff=%d)",
        target_map.NumVoxels(), cur_kf->raw_scan_lidar->size(), mean_res_out, eff_num_out);

    const bool ok = reg.AlignNdt(T_pre_cur, &mean_res_out, &eff_num_out);
    T_pre_cur_out = T_pre_cur;

    // How far did NDT have to pull from the odometry guess? Real loop closures
    // on a drifting trajectory show non-trivial corrections; near-zero shift
    // means either drift-free odom or a basin lock-in (possible false positive).
    const IncLIO::SE3 T_delta = T_init.inverse() * T_pre_cur;
    shift_t_out   = T_delta.translation().norm();
    shift_deg_out = T_delta.so3().log().norm() * 180.0 / M_PI;

    return ok && eff_num_out >= lc_verify_min_eff_pts_
           && mean_res_out  <  lc_verify_mean_res_thres_;
}

// ─────────────────────────────────────────────────────────────────────────────
// PublishLoopMarkers  (loop_worker_ thread — RViz visualization of candidates)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::PublishLoopMarkers(const IncLIO::SE3& cur_pose, int cur_id,
                                 const std::vector<LoopCandidateViz>& candidates) {
    if (!loop_marker_pub_) return;

    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();
    const builtin_interfaces::msg::Duration ttl = rclcpp::Duration::from_seconds(2.0);

    auto make_sphere = [&](int id, const std::string& ns, const IncLIO::Vec3d& p,
                           float r, float g, float b, double diameter) {
        visualization_msgs::msg::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = world_frame_;
        m.ns              = ns;
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::SPHERE;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = p.x();
        m.pose.position.y = p.y();
        m.pose.position.z = p.z();
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = diameter;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9f;
        m.lifetime = ttl;
        return m;
    };

    auto make_text = [&](int id, const std::string& ns, const IncLIO::Vec3d& p,
                         const std::string& text) {
        visualization_msgs::msg::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = world_frame_;
        m.ns              = ns;
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = p.x();
        m.pose.position.y = p.y();
        m.pose.position.z = p.z() + 1.5;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.7;
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f;
        m.text = text;
        m.lifetime = ttl;
        return m;
    };

    // Current keyframe — orange sphere + label.
    arr.markers.push_back(make_sphere(0, "loop_current", cur_pose.translation(),
                                       1.0f, 0.5f, 0.0f, 1.2));
    arr.markers.push_back(make_text(0, "loop_current_label", cur_pose.translation(),
                                     "kf=" + std::to_string(cur_id)));

    // Candidates — green spheres if NDT-verified, red if rejected by verifier.
    // Verified link line is thick green; rejected link is thin yellow.
    int marker_id = 0;
    for (const auto& c : candidates) {
        const float r_s = c.verified ? 0.0f : 1.0f;
        const float g_s = c.verified ? 1.0f : 0.2f;
        const float b_s = 0.0f;
        const std::string ns_sphere = c.verified ? "loop_verified" : "loop_rejected";
        arr.markers.push_back(make_sphere(marker_id, ns_sphere,
                                           c.pose.translation(),
                                           r_s, g_s, b_s, 1.0));
        char buf[120];
        if (c.verified) {
            std::snprintf(buf, sizeof(buf), "kf=%d  d=%.3f  res=%.2f eff=%d",
                          c.id, c.dist, c.mean_res, c.eff_num);
        } else {
            std::snprintf(buf, sizeof(buf), "kf=%d  d=%.3f  (rejected)",
                          c.id, c.dist);
        }
        arr.markers.push_back(make_text(marker_id,
                                         c.verified ? "loop_verified_label"
                                                    : "loop_rejected_label",
                                         c.pose.translation(), buf));

        visualization_msgs::msg::Marker line;
        line.header.stamp    = stamp;
        line.header.frame_id = world_frame_;
        line.ns              = c.verified ? "loop_link_verified" : "loop_link_rejected";
        line.id              = marker_id;
        line.type            = visualization_msgs::msg::Marker::LINE_LIST;
        line.action          = visualization_msgs::msg::Marker::ADD;
        line.scale.x         = c.verified ? 0.3 : 0.15;
        if (c.verified) { line.color.r = 0.0f; line.color.g = 1.0f; line.color.b = 0.0f; }
        else            { line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.0f; }
        line.color.a = 0.9f;
        line.pose.orientation.w = 1.0;
        line.lifetime = ttl;
        geometry_msgs::msg::Point p1, p2;
        p1.x = cur_pose.translation().x();
        p1.y = cur_pose.translation().y();
        p1.z = cur_pose.translation().z();
        p2.x = c.pose.translation().x();
        p2.y = c.pose.translation().y();
        p2.z = c.pose.translation().z();
        line.points.push_back(p1);
        line.points.push_back(p2);
        arr.markers.push_back(line);

        ++marker_id;
    }

    loop_marker_pub_->publish(arr);
}

// ─────────────────────────────────────────────────────────────────────────────
// SaveMapCallback  (~/save_map service)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::SaveMapCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    // full_map_ is already voxel-filtered per scan at map_voxel_size_ in ui_callback,
    // so we just copy and save directly — no additional filter pass needed.
    IncLIO::CloudPtr map(new IncLIO::PointCloudType());
    {
        std::lock_guard<std::mutex> lock(full_map_mutex_);
        *map = *full_map_;
    }
    map->width = map->points.size();
    map->height = 1;
    map->is_dense = true;

    if (map->empty()) {
        res->success = false;
        res->message = "Map is empty, nothing to save";
        return;
    }

    std::string path = "/tmp/inclio_map.pcd";
    if (pcl::io::savePCDFileBinary(path, *map) == 0) {
        res->success = true;
        res->message = "Saved " + std::to_string(map->points.size()) +
                       " points to " + path;
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    } else {
        res->success = false;
        res->message = "Failed to write " + path;
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
}

} // namespace inclio_ros2

// ─────────────────────────────────────────────────────────────────────────────
// Register as a composable component
// ─────────────────────────────────────────────────────────────────────────────
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(inclio_ros2::LioNode)
