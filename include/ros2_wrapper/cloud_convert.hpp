#include <omp.h> // for parallel loops in cloud conversion
#pragma once

// cloud_convert.hpp
//
// Converts incoming ROS2 sensor messages into IncLIO's FullPointCloudType.
//
// Supported input formats:
//   - sensor_msgs/msg/PointCloud2  (Velodyne, Ouster, Livox PointCloud2 mode)
//   - livox_ros_driver2/msg/CustomMsg  (Livox native, if available)

#include <common/utils/point_types.hpp>  // FullPointCloudType, FullPointType, FullCloudPtr

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#ifdef HAVE_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include <string>
#include <cstdint>
#include <cstring>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <execution>
#include <rclcpp/rclcpp.hpp>


namespace inclio_ros2 {


enum class LidarType {
    LIVOX = 1,  // Lviox CustomMSG ( not tested yet)
    VELO32,    // Velodyne 32 or 16 ? ( not tested yet)
    OUST64,    // ouster 64 (not tested yet)
    HESAI,     // Hesai Pandar128 (tested)
};

// ─────────────────────────────────────────────────────────────────────────────
// CloudConvertConfig
// ─────────────────────────────────────────────────────────────────────────────
struct CloudConvertConfig {
    LidarType lidar_type       = LidarType::HESAI;  // default to Hesai Pandar128
    int    num_scans        = 128;    // number of scan lines (rings)
    double time_scale       = 1e-3;  // per-point time field scale to seconds
    int    point_filter_num = 1;     // keep every N-th point (1 = keep all)
    double imu_coeff        = 1.0;   // coefficient to apply to IMU data (e.g. 9.81 if using raw accel in m/s²)
};

// ─────────────────────────────────────────────────────────────────────────────
// CloudConverter
// ─────────────────────────────────────────────────────────────────────────────
class CloudConverter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit CloudConverter(const CloudConvertConfig& cfg = {}) : cfg_(cfg) {}

    void SetConfig(const CloudConvertConfig& cfg) { cfg_ = cfg; }
    const CloudConvertConfig& Config() const { return cfg_; }

    /**
     * sensor_msgs::PointCloud2 input
     * @param msg
     * @param pcl_out
     */
    void Process(const sensor_msgs::msg::PointCloud2 & msg, IncLIO::FullCloudPtr &pcl_out);

    /**
     * Load configuration from a YAML file
     * @param yaml
     */
    void LoadFromYAML(const std::string &yaml);


#ifdef HAVE_LIVOX_ROS_DRIVER2
    void Process(const livox_ros_driver2::msg::CustomMsg &msg, IncLIO::FullCloudPtr &pcl_out);
#endif

private:
    void Oust64Handler(const sensor_msgs::msg::PointCloud2 & msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2 & msg);
    void HesaiHandler(const sensor_msgs::msg::PointCloud2 & msg);

#ifdef HAVE_LIVOX_ROS_DRIVER2
    void AviaHandler(const livox_ros_driver2::msg::CustomMsg &msg);
#endif

    CloudConvertConfig cfg_;
    IncLIO::FullPointCloudType cloud_out_;
};

} // namespace inclio_ros2
