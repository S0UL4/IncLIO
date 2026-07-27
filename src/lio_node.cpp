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
    declare_parameter<std::string>("wheel_odom_topic", "");   // empty => wheel odom disabled

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
}

// ─────────────────────────────────────────────────────────────────────────────
// InitLIO
// ─────────────────────────────────────────────────────────────────────────────
bool LioNode::InitLIO() {
    // ── Read parameters ───────────────────────────────────────────────────────
    config_file_   = get_parameter("config_file").as_string();
    world_frame_   = get_parameter("world_frame").as_string();
    body_frame_    = get_parameter("body_frame").as_string();
    publish_path_  = get_parameter("publish_path").as_bool();
    publish_cloud_ = get_parameter("publish_cloud").as_bool();
    map_voxel_size_   = get_parameter("map_voxel_size").as_double();
    body_crop_radius_ = get_parameter("body_crop_radius").as_double();
    publish_radius_ = get_parameter("publish_radius").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    full_map_      = std::make_shared<IncLIO::PointCloudType>();
    viz_max_scans_ = get_parameter("local_map_scans").as_int();

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

    RCLCPP_INFO(get_logger()," Config file loaded %s", config_file_.c_str());

    IncLIO::LIOConfig lio_config;
    
    if (yaml["frames"]) {
        auto f = yaml["frames"];
        if (f["publish_tf"])        publish_tf_       = f["publish_tf"].as<bool>();
        if (f["world_frame"])       world_frame_id    = f["world_frame"].as<std::string>();
        if (f["base_frame"])        base_frame_id     = f["base_frame"].as<std::string>();
        if (f["imu_frame"])         imu_frame_id      = f["imu_frame"].as<std::string>();
        if (f["lidar_frame"])       lidar_frame_id    = f["lidar_frame"].as<std::string>();
        if (f["use_tf_extrinsic"])  use_tf_extrinsic  = f["use_tf_extrinsic"].as<bool>();
        if (f["tf_lookup_timeout"]) tf_lookup_timeout = f["tf_lookup_timeout"].as<double>();
    }
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
    if (yaml["mapping"] && !use_tf_extrinsic ) {
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

    // Wheel-odometry fusion params (optional 'wheel_odom:' block). The topic itself
    // is the ROS param 'wheel_odom_topic'; these tune the IESKF observation.
    if (yaml["wheel_odom"]) {
        auto w = yaml["wheel_odom"];
        if (w["r_imu_wheel"]) {
            auto v = w["r_imu_wheel"].as<std::vector<double>>();
            lio_config.wheel_r_imu_wheel = IncLIO::Vec3d(v[0], v[1], v[2]);
        }
        if (w["vel_noise"]) lio_config.wheel_vel_noise = w["vel_noise"].as<double>();
        if (w["nhc_noise"]) lio_config.wheel_nhc_noise = w["nhc_noise"].as<double>();
        if (w["yaw_noise"]) lio_config.wheel_yaw_noise = w["yaw_noise"].as<double>();
        if (w["use_lever_arm"]) lio_config.wheel_use_lever_arm = w["use_lever_arm"].as<bool>();
        if (w["use_yaw_rate"])  lio_config.wheel_use_yaw_rate  = w["use_yaw_rate"].as<bool>();
        if (w["chi2_thresh"])   lio_config.wheel_chi2_thresh   = w["chi2_thresh"].as<double>();
        RCLCPP_INFO(get_logger(),
            "wheel_odom: r_imu_wheel=[%.3f, %.3f, %.3f] vel=%.3f nhc=%.3f yaw=%.3f lever=%s yaw_row=%s chi2=%.2f",
            lio_config.wheel_r_imu_wheel.x(), lio_config.wheel_r_imu_wheel.y(),
            lio_config.wheel_r_imu_wheel.z(), lio_config.wheel_vel_noise,
            lio_config.wheel_nhc_noise, lio_config.wheel_yaw_noise,
            lio_config.wheel_use_lever_arm ? "true" : "false",
            lio_config.wheel_use_yaw_rate ? "true" : "false",
            lio_config.wheel_chi2_thresh);
    }


    RCLCPP_INFO(get_logger(),"publish_tf_ = %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(),"use_tf_extrinsic = %s", use_tf_extrinsic ? "true" : "false");
    RCLCPP_INFO(get_logger(),"tf_lookup_timeout = %f", tf_lookup_timeout);
    RCLCPP_INFO(get_logger(),"world_frame_id = %s", world_frame_id.c_str());
    RCLCPP_INFO(get_logger(),"base_frame_id = %s", base_frame_id.c_str());
    RCLCPP_INFO(get_logger(),"imu_frame_id = %s", imu_frame_id.c_str());
    RCLCPP_INFO(get_logger(),"lidar_frame_id = %s", lidar_frame_id.c_str());

    {
        const Eigen::Vector3d t = lio_config.T_imu_lidar.translation();
        const Eigen::Quaterniond q = lio_config.T_imu_lidar.unit_quaternion();
        RCLCPP_INFO(get_logger(),
                    "LIOConfig summary T_IMU_LIDAR before TF lookup:\n"
                    "  t = [%.4f, %.4f, %.4f]\n"
                    "  q (wxyz) = [%.4f, %.4f, %.4f, %.4f]",
                    t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z());
    }
    // For tf listener 
    const auto tf_timeout = tf2::durationFromSec(tf_lookup_timeout);

    // tf2 buffer + listener (listener runs its own internal spin thread)
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    if (use_tf_extrinsic)
    {
        try {
            auto tf = tf_buffer_->lookupTransform(imu_frame_id, lidar_frame_id, tf2::TimePointZero, tf_timeout);
            Lidar_T_wrt_IMU << tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z;
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
            q.normalize();  // TF quaternions are not exactly unit-norm; Sophus::SO3 asserts orthogonality
            Lidar_R_wrt_IMU = q.toRotationMatrix();
            lio_config.T_imu_lidar.so3() = IncLIO::SO3(Lidar_R_wrt_IMU);
            lio_config.T_imu_lidar.translation() = Lidar_T_wrt_IMU;
            RCLCPP_INFO(this->get_logger(), "Loaded lidar->imu extrinsic from TF: %s <- %s",
                        imu_frame_id.c_str(), lidar_frame_id.c_str());
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not look up %s <- %s from TF (%s). Falling back to YAML extrinsic.",
                        imu_frame_id.c_str(), lidar_frame_id.c_str(), ex.what());
        }
    }

    {
        const Eigen::Vector3d t = lio_config.T_imu_lidar.translation();
        const Eigen::Quaterniond q = lio_config.T_imu_lidar.unit_quaternion();
        RCLCPP_INFO(get_logger(),
                    "LIOConfig summary T_IMU_LIDAR after TF lookup:\n"
                    "  t = [%.4f, %.4f, %.4f]\n"
                    "  q (wxyz) = [%.4f, %.4f, %.4f, %.4f]",
                    t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z());
    }


    if (base_frame_id != imu_frame_id)
    {
        try {
            auto tf = tf_buffer_->lookupTransform(imu_frame_id, base_frame_id, tf2::TimePointZero, tf_timeout);
            t_imu_base << tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z;
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
            q.normalize();  // TF quaternions are not exactly unit-norm; Sophus::SO3 asserts orthogonality
            R_imu_base = q.toRotationMatrix();
            RCLCPP_INFO(this->get_logger(), "Loaded base->imu offset from TF: %s <- %s",
                        imu_frame_id.c_str(), base_frame_id.c_str());
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not look up %s <- %s from TF (%s). Using identity (published TF = IMU pose).",
                        imu_frame_id.c_str(), base_frame_id.c_str(), ex.what());
        }
    }

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

    // Wheel odometry (optional) — nav_msgs/Odometry twist, base_link frame.
    // Shares the IMU callback group so it never blocks LiDAR processing.
    std::string wheel_odom_topic = get_parameter("wheel_odom_topic").as_string();
    if (!wheel_odom_topic.empty()) {
        auto odom_qos = rclcpp::SensorDataQoS();
        odom_qos.keep_last(200);
        wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            wheel_odom_topic, odom_qos,
            [this](nav_msgs::msg::Odometry::UniquePtr msg) { OdomCallback(std::move(msg)); },
            imu_opts);
        RCLCPP_INFO(get_logger(), "Subscribing to wheel odom: %s", wheel_odom_topic.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Wheel odom disabled (wheel_odom_topic empty)");
    }

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

    path_msg_.header.frame_id = world_frame_id;

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

        // auto state = lio_->GetPropagatedState();
        // auto pose = state.GetSE3();

        // nav_msgs::msg::Odometry odom;
        // odom.header.stamp    = msg->header.stamp;
        // odom.header.frame_id = world_frame_;
        // odom.child_frame_id  = body_frame_;

        // const auto& t = pose.translation();
        // const auto  q = pose.unit_quaternion();
        // odom.pose.pose.position.x    = t.x();
        // odom.pose.pose.position.y    = t.y();
        // odom.pose.pose.position.z    = t.z();
        // odom.pose.pose.orientation.x = q.x();
        // odom.pose.pose.orientation.y = q.y();
        // odom.pose.pose.orientation.z = q.z();
        // odom.pose.pose.orientation.w = q.w();

        // IncLIO::Vec3d v_body = pose.so3().inverse() * state.v_;
        // odom.twist.twist.linear.x = v_body.x();
        // odom.twist.twist.linear.y = v_body.y();
        // odom.twist.twist.linear.z = v_body.z();

        // odom_fast_pub_->publish(odom);

        // // TF at IMU rate — smooth transform for RViz and downstream nodes
        // if (publish_tf_) {
        //     geometry_msgs::msg::TransformStamped tf_msg;
        //     tf_msg.header.stamp    = msg->header.stamp;
        //     tf_msg.header.frame_id = world_frame_;
        //     tf_msg.child_frame_id  = body_frame_;
        //     tf_msg.transform.translation.x = t.x();
        //     tf_msg.transform.translation.y = t.y();
        //     tf_msg.transform.translation.z = t.z();
        //     tf_msg.transform.rotation.x    = q.x();
        //     tf_msg.transform.rotation.y    = q.y();
        //     tf_msg.transform.rotation.z    = q.z();
        //     tf_msg.transform.rotation.w    = q.w();
        //     tf_broadcaster_->sendTransform(tf_msg);
        // }
    } else {
        // Before init: buffer only — ProcessCloud drains these into LIO
        std::lock_guard<std::mutex> lock(imu_buf_mutex_);
        imu_buffer_.push_back(imu_out);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// OdomCallback  (nav_msgs/Odometry wheel odometry — twist in base_link frame)
// ─────────────────────────────────────────────────────────────────────────────
void LioNode::OdomCallback(nav_msgs::msg::Odometry::UniquePtr msg) {
    if (!lio_ || !lio_->IsInitialized()) {
        // Wheel updates only make sense once the filter is running.
        return;
    }

    const double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // twist is expressed in child_frame_id (base_link): forward speed + yaw rate
    // are taken directly, no pulse/wheel-radius conversion needed.
    IncLIO::Odom odom(ts, msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    lio_->AddOdom(odom);
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

    ////////////////////// Publish base_link pose in the world frame //////////////////////
    // T_world_base = T_world_imu * T_imu_base
    // Tracking base_link (not the IMU) removes the lever-arm arc on in-place turns.
    // IncLIO::Mat3d R_world_base = pose.rotationMatrix() * R_imu_base;
    // IncLIO::Vec3d t_world_base = pose.rotationMatrix() * t_imu_base + pose.translation();

    Sophus::SE3 T_world_imu(pose.rotationMatrix(), pose.translation());
    Sophus::SE3 T_imu_base(R_imu_base, t_imu_base);

    Sophus::SE3 T_base_imu = T_imu_base.inverse();

    // Sophus::SE3 T_world_base = T_world_imu * T_imu_base;


    // At t=0, save the initial base pose once
    static Sophus::SE3d T_world_base_init;
    static bool initialized = false;

    Sophus::SE3d T_world_base = T_world_imu * T_imu_base;

    if (!initialized) {
        T_world_base_init = T_world_base;
        initialized = true;
    }

    // Every frame
    // Sophus::SE3d T_world_imu(pose.rotationMatrix(), pose.translation());
    
    // Re-anchor: express pose relative to initial base pose
    Sophus::SE3d T_base0_base = T_world_base_init.inverse() * T_world_base;


    // IncLIO::Mat3d R_imu_init_imu = pose.rotationMatrix();

    // // imu_init_T_base = imu_init_T_imu * imu_T_base
    // IncLIO::Mat3d R_imu_init_base = R_imu_init_imu * R_imu_base;
    // IncLIO::Vec3d t_imu_init_base = R_imu_init_imu * t_imu_base + pose.translation();

    // // base_T_imu = inv(imu_T_base)
    // IncLIO::Mat3d R_base_imu = R_imu_base.transpose();
    // IncLIO::Vec3d t_base_imu = -R_base_imu * t_imu_base;

    // // published_T_base = base_T_imu * imu_init_T_base
    // IncLIO::Mat3d R_pub = R_base_imu * R_imu_init_base;
    // IncLIO::Vec3d t_pub = R_base_imu * t_imu_init_base + t_base_imu;

    // pose.so3() = IncLIO::SO3(R_world_base);
    // pose.translation() = t_world_base;

    Sophus::SE3d T_base0_imu = T_world_base_init.inverse() * T_world_imu;

    PublishOdometry(stamp, T_base0_base, state);
    if (publish_path_)  PublishPath(stamp, T_base0_base);
    if (publish_cloud_ && (lio_->WasKeyframe() || lio_->frame_num() % 10 == 0)) PublishCloud(stamp, T_base0_imu);
    if(publish_tf_) PublishTF(stamp, T_base0_base);
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
    odom.header.frame_id = world_frame_id;
    odom.child_frame_id  = base_frame_id;

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
    ps.header.frame_id = world_frame_id;

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
    tf_msg.header.frame_id = world_frame_id;
    tf_msg.child_frame_id  = base_frame_id;

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
        msg.header.frame_id = world_frame_id;
        cloud_world_pub_->publish(msg);
    }
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
