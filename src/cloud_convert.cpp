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
    cfg_.imu_coeff = yaml["preprocess"]["imu_coeff"].as<double>();

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
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;
        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;
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


void CloudConverter::Process(const livox_ros_driver2::msg::CustomMsg &msg, IncLIO::FullCloudPtr &pcl_out) {
    switch (cfg_.lidar_type) {
        case LidarType::LIVOX:
            AviaHandler(msg);
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("CloudConverter"), "Error LiDAR Type: %d", int(cfg_.lidar_type));
            break;
    }
    pcl_out = std::make_shared<IncLIO::FullPointCloudType>(cloud_out_);
}

void CloudConverter::Oust64Handler(const sensor_msgs::msg::PointCloud2 &msg) {
    cloud_out_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    const int plsize = pl_orig.size();
    if (plsize == 0) return;

    // Pass 1: collect surviving indices (every N-th point, range > 4 m).
    std::vector<int> kept;
    kept.reserve(plsize / cfg_.point_filter_num + 1);
    for (int i = 0; i < plsize; i += cfg_.point_filter_num) {
        const auto& p = pl_orig.points[i];
        if (p.x * p.x + p.y * p.y + p.z * p.z >= 16.0f)
            kept.push_back(i);
    }
    cloud_out_.resize(kept.size());

    // Pass 2: parallel write into pre-sized buffer.
    #pragma omp parallel for schedule(static)
    for (size_t j = 0; j < kept.size(); j++) {
        const int i = kept[j];
        auto& added_pt = cloud_out_.points[j];
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].t / cfg_.time_scale;
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = true;
}


void CloudConverter::AviaHandler(const livox_ros_driver2::msg::CustomMsg &msg) {
    cloud_out_.clear();
    IncLIO::FullPointCloudType cloud_temp;
    cloud_temp.clear();
    int plsize = msg.point_num;

    cloud_out_.reserve(plsize);
    cloud_temp.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg.points[i].line < cfg_.num_scans) &&
            ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00)) {
            if (i % cfg_.point_filter_num == 0) {
                cloud_temp.points[i].x = msg.points[i].x;
                cloud_temp.points[i].y = msg.points[i].y;
                cloud_temp.points[i].z = msg.points[i].z;
                cloud_temp.points[i].intensity = msg.points[i].reflectivity;
                cloud_temp.points[i].time = msg.points[i].offset_time / float(cfg_.time_scale);

                if ((abs(cloud_temp.points[i].x - cloud_temp.points[i - 1].x) > 1e-7) ||
                    (abs(cloud_temp.points[i].y - cloud_temp.points[i - 1].y) > 1e-7) ||
                    (abs(cloud_temp.points[i].z - cloud_temp.points[i - 1].z) > 1e-7)) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_temp.points[i]);
        }
    }
}



void CloudConverter::HesaiHandler(const sensor_msgs::msg::PointCloud2 & msg) {
    cloud_out_.clear();
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    int plsize = pl_orig.size();

    // Determine the earliest timestamp in this scan to compute relative offsets.
    // Hesai timestamps are absolute (seconds since epoch); the undistortion code
    // expects relative offsets in milliseconds from scan start.
    // Pass 1: find earliest timestamp with proper OMP reduction
    double t0 = std::numeric_limits<double>::max();
    #pragma omp parallel for reduction(min:t0)
    for (size_t i = 0; i < pl_orig.points.size(); i++) {
        if (pl_orig.points[i].timestamp < t0) t0 = pl_orig.points[i].timestamp;
    }

    // Pass 2: collect surviving indices (every N-th point)
    std::vector<int> kept;
    kept.reserve(plsize / cfg_.point_filter_num + 1);
    for (int i = 0; i < plsize; i += cfg_.point_filter_num) {
        kept.push_back(i);
    }
    cloud_out_.resize(kept.size());

    // Pass 3: parallel write into pre-sized buffer (no push_back race)
    #pragma omp parallel for schedule(static)
    for (size_t j = 0; j < kept.size(); j++) {
        const int i = kept[j];
        auto& added_pt = cloud_out_.points[j];
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.ring = pl_orig.points[i].ring;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = (pl_orig.points[i].timestamp - t0) * 1e3;
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = true;
}

void CloudConverter::VelodyneHandler(const sensor_msgs::msg::PointCloud2 & msg) {
    cloud_out_.clear();

    // Deserialize into PointNoIntensity — always succeeds regardless of
    // whether the message has an intensity field (e.g. NCLT dataset omits it).
    pcl::PointCloud<velodyne_ros::PointNoIntensity> pl;
    pcl::fromROSMsg(msg, pl);
    const int plsize = pl.points.size();
    if (plsize == 0) return;

    // Read intensity from raw PointCloud2 bytes when the field exists
    int off_intensity = -1;
    for (const auto& f : msg.fields) {
        if (f.name == "intensity") { off_intensity = f.offset; break; }
    }
    const uint8_t* raw = msg.data.data();
    const uint32_t step = msg.point_step;
    auto intensity_at = [&](int i) -> float {
        if (off_intensity < 0) return 0.0f;
        float v;
        std::memcpy(&v, raw + static_cast<size_t>(i) * step + off_intensity, sizeof(float));
        return v;
    };

    bool given_offset_time = (pl.points[plsize - 1].time > 0);

    if (given_offset_time) {
        // Timestamps provided — no per-ring state, fully parallelizable.
        std::vector<int> kept;
        kept.reserve(plsize / cfg_.point_filter_num + 1);
        for (int i = 0; i < plsize; i += cfg_.point_filter_num) {
            const auto& p = pl.points[i];
            if (p.x * p.x + p.y * p.y + p.z * p.z >= 16.0f)
                kept.push_back(i);
        }
        cloud_out_.resize(kept.size());

        #pragma omp parallel for schedule(static)
        for (size_t j = 0; j < kept.size(); j++) {
            const int i = kept[j];
            auto& pt = cloud_out_.points[j];
            pt.x = pl.points[i].x;
            pt.y = pl.points[i].y;
            pt.z = pl.points[i].z;
            pt.intensity = intensity_at(i);
            pt.time = pl.points[i].time * cfg_.time_scale;
        }
    } else {
        // No timestamps — compute yaw-based offsets per ring (sequential).
        cloud_out_.reserve(plsize / cfg_.point_filter_num + 1);

        double omega_l = 3.61;  // scan angular velocity
        std::vector<bool> is_first(cfg_.num_scans, true);
        std::vector<double> yaw_fp(cfg_.num_scans, 0.0);
        std::vector<float> yaw_last(cfg_.num_scans, 0.0);
        std::vector<float> time_last(cfg_.num_scans, 0.0);

        for (int i = 0; i < plsize; i++) {
            IncLIO::FullPointType added_pt;
            added_pt.x = pl.points[i].x;
            added_pt.y = pl.points[i].y;
            added_pt.z = pl.points[i].z;
            added_pt.intensity = intensity_at(i);

            if (added_pt.getVector3fMap().squaredNorm() < 16.0f) continue;

            int layer = pl.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = 0.0;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;

            if (i % cfg_.point_filter_num == 0) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = true;
}




} // namespace inclio_ros2
