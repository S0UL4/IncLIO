#include "inclio/lidar_processor.hpp"

namespace IncLIO {

void LidarProcessor::ProcessOuster(const pcl::PointCloud<ouster_ros::Point>& cloud_in, FullCloudPtr& cloud_out) {
    cloud_out_.clear();
    cloud_out->clear();
    cloud_out->reserve(cloud_in.size());

    for (size_t i = 0; i < cloud_in.points.size(); i++) {
        if (static_cast<int>(i) % config_.point_filter_num != 0) continue;

        FullPointType added_pt;
        added_pt.x = cloud_in.points[i].x;
        added_pt.y = cloud_in.points[i].y;
        added_pt.z = cloud_in.points[i].z;
        added_pt.intensity = cloud_in.points[i].intensity;
        added_pt.time = cloud_in.points[i].t / 1e6;  // ns -> ms

        cloud_out->points.push_back(added_pt);
    }
}

void LidarProcessor::ProcessVelodyne(const pcl::PointCloud<velodyne_ros::Point>& cloud_in, FullCloudPtr& cloud_out) {
    cloud_out_.clear();
    cloud_out->clear();
    int plsize = cloud_in.points.size();
    cloud_out->reserve(plsize);

    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(config_.num_scans, true);
    std::vector<double> yaw_fp(config_.num_scans, 0.0);
    std::vector<float> yaw_last(config_.num_scans, 0.0);
    std::vector<float> time_last(config_.num_scans, 0.0);

    bool given_offset_time = (plsize > 0 && cloud_in.points[plsize - 1].time > 0);

    if (!given_offset_time && plsize > 0) {
        // Compute yaw bounds for time estimation
        double yaw_first = atan2(cloud_in.points[0].y, cloud_in.points[0].x) * 57.29578;
        int layer_first = cloud_in.points[0].ring;
        for (int i = plsize - 1; i > 0; i--) {
            if (cloud_in.points[i].ring == layer_first) {
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        FullPointType added_pt;
        added_pt.x = cloud_in.points[i].x;
        added_pt.y = cloud_in.points[i].y;
        added_pt.z = cloud_in.points[i].z;
        added_pt.intensity = cloud_in.points[i].intensity;
        added_pt.time = cloud_in.points[i].time * config_.time_scale;

        if (added_pt.getVector3fMap().norm() < 4.0f) {
            continue;
        }

        if (!given_offset_time) {
            int layer = cloud_in.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
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
        }

        if (i % config_.point_filter_num == 0) {
            cloud_out->points.push_back(added_pt);
        }
    }
}

void LidarProcessor::LoadFromYAML(const std::string& yaml_file) {
    auto yaml = YAML::LoadFile(yaml_file);
    config_.time_scale = yaml["preprocess"]["time_scale"].as<float>();
    int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
    config_.num_scans = yaml["preprocess"]["scan_line"].as<int>();
    config_.point_filter_num = yaml["point_filter_num"].as<int>();

    if (lidar_type == 1) {
        config_.lidar_type = LidarType::LIVOX;
        INCLIO_INFO("Using LIVOX Lidar");
    } else if (lidar_type == 2) {
        config_.lidar_type = LidarType::VELODYNE;
        INCLIO_INFO("Using Velodyne Lidar");
    } else if (lidar_type == 3) {
        config_.lidar_type = LidarType::OUSTER;
        INCLIO_INFO("Using OUSTER Lidar");
    } else {
        INCLIO_WARN("Unknown lidar_type: {}", lidar_type);
    }
}

} // namespace IncLIO
