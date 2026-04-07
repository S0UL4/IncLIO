#ifndef INCLIO_LIDAR_PROCESSOR_HPP
#define INCLIO_LIDAR_PROCESSOR_HPP

#include "utils/point_types.hpp"
#include "utils/logger.hpp"
#include <yaml-cpp/yaml.h>
#include <string>

namespace IncLIO {

// Supported LiDAR sensor types
enum class LidarType {
    LIVOX = 1,   // Avia, Mid-360, HAP
    VELODYNE,    // VLP-16, VLP-32, HDL-64
    OUSTER,      // OS0/OS1/OS2
    HESAI,       // XT32, Pandar, AT128
    ROBOSENSE,   // RS-LiDAR-16/32/M1
    CUSTOM       // user-defined
};

struct LidarProcessorConfig {
    LidarType lidar_type = LidarType::VELODYNE;
    int point_filter_num = 1;
    int num_scans = 6;
    float time_scale = 1e-3f;
};

// LiDAR point cloud preprocessing (ROS-agnostic)
//
// Vendor-specific message conversion (sensor_msgs, livox_ros, etc.)
// belongs in the ROS2 wrapper layer, not here.
//
// This class handles:
//   - Processing already-converted FullPointCloud data
//   - Filtering (range, intensity, voxel downsampling)
//   - Configuration from YAML
class LidarProcessor {
   public:
    LidarProcessor() = default;
    explicit LidarProcessor(const LidarProcessorConfig& config) : config_(config) {}
    ~LidarProcessor() = default;

    // Process an already-converted point cloud (vendor conversion done externally)
    void ProcessOuster(const pcl::PointCloud<ouster_ros::Point>& cloud_in, FullCloudPtr& cloud_out);
    void ProcessVelodyne(const pcl::PointCloud<velodyne_ros::Point>& cloud_in, FullCloudPtr& cloud_out);

    // Load parameters from YAML
    void LoadFromYAML(const std::string& yaml_file);

    LidarType GetLidarType() const { return config_.lidar_type; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    LidarProcessorConfig config_;
    FullPointCloudType cloud_out_;
};

} // namespace IncLIO

#endif // INCLIO_LIDAR_PROCESSOR_HPP
