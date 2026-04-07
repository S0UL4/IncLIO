// cloud_convert.cpp
//
// Implements CloudConverter::Convert() for PointCloud2 and (optionally)
// Livox CustomMsg.  The logic mirrors run_bag.cc's parse_pointcloud2() and
// parse_livox_custom() but operates on the already-deserialised ROS2 message
// structs rather than raw CDR bytes.

#include "ros2_wrapper/cloud_convert.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstring>
#include <algorithm>

namespace inclio_ros2 {

void CloudConverter::LoadFromYAML(const std::string &yaml_file) {
    auto yaml = YAML::LoadFile(yaml_file);
    cfg_.time_scale = yaml["preprocess"]["time_scale"].as<double>();
    int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
    cfg_.num_scans = yaml["preprocess"]["scan_line"].as<int>();
    cfg_.point_filter_num = yaml["point_filter_num"].as<int>();

    if (lidar_type == 1) {
        cfg_.lidar_type = LidarType::LIVOX;
        RCLCPP_INFO(rclcpp::get_logger("CloudConverter"), "Using Livox Lidar");
    } else if (lidar_type == 2) {
        cfg_.lidar_type = LidarType::VELO32;
        RCLCPP_INFO(rclcpp::get_logger("CloudConverter"), "Using Velodyne 32 Lidar");
    } else if (lidar_type == 3) {
        cfg_.lidar_type = LidarType::OUST64;
        RCLCPP_INFO(rclcpp::get_logger("CloudConverter"), "Using OUST 64 Lidar");
    } else if (lidar_type == 4) {
        cfg_.lidar_type = LidarType::HESAI;
        RCLCPP_INFO(rclcpp::get_logger("CloudConverter"), "Using Hesai Pandar128 Lidar");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("CloudConverter"), "unknown lidar_type");
    }
}


void CloudConverter::Process(const sensor_msgs::msg::PointCloud2 & msg, IncLIO::FullCloudPtr &pcl_out) {
    switch (cfg_.lidar_type) {
        // case LidarType::OUST64:
        //     Oust64Handler(msg);
        //     break;

        // case LidarType::VELO32:
        //     VelodyneHandler(msg);
        //     break;
        case LidarType::HESAI:
            HesaiHandler(msg);
            break;

        default:
            RCLCPP_ERROR(rclcpp::get_logger("CloudConverter"), "Error LiDAR Type: %d", int(cfg_.lidar_type));
            break;
    }
    pcl_out = std::make_shared<IncLIO::FullPointCloudType>(cloud_out_);
    //*pcl_out = cloud_out_;
}


void CloudConverter::HesaiHandler(const sensor_msgs::msg::PointCloud2 & msg) {
    cloud_out_.clear();
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    // Determine the earliest timestamp in this scan to compute relative offsets.
    // Hesai timestamps are absolute (seconds since epoch); the undistortion code
    // expects relative offsets in milliseconds from scan start.
    double t0 = std::numeric_limits<double>::max();
    for (const auto& pt : pl_orig.points) {
        if (pt.timestamp < t0) t0 = pt.timestamp;
    }

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % cfg_.point_filter_num != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        IncLIO::FullPointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.ring = pl_orig.points[i].ring;
        added_pt.intensity = pl_orig.points[i].intensity;
        // Convert absolute timestamp to relative offset in milliseconds
        added_pt.time = (pl_orig.points[i].timestamp - t0) * 1e3;
        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = true;
}


} // namespace inclio_ros2
