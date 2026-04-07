// Offline ROS2 bag player for IncLIO.
//
// Reads a ROS2 bag (SQLite3 .db3 format) containing IMU and LiDAR data,
// feeds them into the LIO pipeline, and outputs poses.
//
// Supports:
//   - sensor_msgs/msg/Imu
//   - sensor_msgs/msg/PointCloud2  (Velodyne, Ouster, Livox PointCloud2 mode, etc.)
//   - livox_ros_driver2/msg/CustomMsg  (Livox native format)
//
// Usage:
//   run_bag <bag_path> <config.yaml> [options]
//
// Options:
//   --imu-topic <topic>      IMU topic name       (default: auto-detect)
//   --lidar-topic <topic>    LiDAR topic name     (default: auto-detect)

#include "rosbag2_reader.h"

#include "inclio/inclio.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

using namespace IncLIO;

// ---------------------------------------------------------------------------
// sensor_msgs/msg/PointField datatype constants
// ---------------------------------------------------------------------------
enum PointFieldType : uint8_t {
    PF_INT8 = 1,
    PF_UINT8 = 2,
    PF_INT16 = 3,
    PF_UINT16 = 4,
    PF_INT32 = 5,
    PF_UINT32 = 6,
    PF_FLOAT32 = 7,
    PF_FLOAT64 = 8,
};

// ---------------------------------------------------------------------------
// Parse sensor_msgs/msg/Imu
// ---------------------------------------------------------------------------
IMUPtr parse_imu(const uint8_t* data, size_t size, double imu_coeff = 1.0) {
    rosbag2::CdrReader r(data, size);
    auto stamp = rosbag2::read_header(r);

    // orientation (quaternion) — skip
    r.read_float64();
    r.read_float64();
    r.read_float64();
    r.read_float64();
    // orientation_covariance[9] — skip
    for (int i = 0; i < 9; i++) r.read_float64();

    // angular_velocity
    double gx = r.read_float64();
    double gy = r.read_float64();
    double gz = r.read_float64();
    for (int i = 0; i < 9; i++) r.read_float64();  // covariance

    // linear_acceleration
    double ax = r.read_float64();
    double ay = r.read_float64();
    double az = r.read_float64();

    return std::make_shared<IMUData>(stamp.to_sec(), Vec3d(gx, gy, gz), Vec3d(ax * imu_coeff, ay * imu_coeff, az * imu_coeff));
}

// ---------------------------------------------------------------------------
// Parse sensor_msgs/msg/PointCloud2 → FullCloudPtr
//
// Works with any LiDAR that publishes PointCloud2 (Velodyne, Ouster, Livox
// in PointCloud2 mode, etc.). Extracts x, y, z, intensity, time, ring fields
// by name from the PointField descriptors.
// ---------------------------------------------------------------------------
FullCloudPtr parse_pointcloud2(const uint8_t* data, size_t size, double& timestamp_out,
                               int point_filter_num, int num_scans, double time_scale) {
    rosbag2::CdrReader r(data, size);
    auto stamp = rosbag2::read_header(r);
    timestamp_out = stamp.to_sec();

    uint32_t height = r.read_uint32();
    uint32_t width = r.read_uint32();

    // PointField[] fields
    struct FieldInfo {
        std::string name;
        uint32_t offset;
        uint8_t datatype;
        uint32_t count;
    };
    uint32_t num_fields = r.read_uint32();
    std::vector<FieldInfo> fields(num_fields);
    for (uint32_t i = 0; i < num_fields; i++) {
        fields[i].name = r.read_string();
        fields[i].offset = r.read_uint32();
        fields[i].datatype = r.read_uint8();
        fields[i].count = r.read_uint32();
    }

    /*is_bigendian=*/r.read_uint8();
    uint32_t point_step = r.read_uint32();
    /*row_step=*/r.read_uint32();

    // data[] (uint8 sequence)
    uint32_t data_len = r.read_uint32();
    const uint8_t* point_data = r.read_raw(data_len);

    // Locate fields by name
    int x_off = -1, y_off = -1, z_off = -1;
    int intensity_off = -1, time_off = -1, ring_off = -1;
    uint8_t intensity_type = 0, time_type = 0, ring_type = 0;

    for (const auto& f : fields) {
        if (f.name == "x")
            x_off = static_cast<int>(f.offset);
        else if (f.name == "y")
            y_off = static_cast<int>(f.offset);
        else if (f.name == "z")
            z_off = static_cast<int>(f.offset);
        else if (f.name == "intensity" || f.name == "reflectivity") {
            intensity_off = static_cast<int>(f.offset);
            intensity_type = f.datatype;
        } else if (f.name == "time" || f.name == "t" || f.name == "timestamp" ||
                   f.name == "time_stamp") {
            time_off = static_cast<int>(f.offset);
            time_type = f.datatype;
        } else if (f.name == "ring" || f.name == "line") {
            ring_off = static_cast<int>(f.offset);
            ring_type = f.datatype;
        }
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) {
        spdlog::warn("PointCloud2 missing x/y/z fields, skipping");
        return nullptr;
    }

    uint32_t num_points = height * width;
    auto cloud = std::make_shared<FullPointCloudType>();
    cloud->reserve(num_points / std::max(point_filter_num, 1) + 1);

    for (uint32_t i = 0; i < num_points; i++) {
        if (static_cast<int>(i) % point_filter_num != 0) continue;

        const uint8_t* pt = point_data + static_cast<size_t>(i) * point_step;
        float x, y, z;
        std::memcpy(&x, pt + x_off, 4);
        std::memcpy(&y, pt + y_off, 4);
        std::memcpy(&z, pt + z_off, 4);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
        float range = std::sqrt(x * x + y * y + z * z);
        if (range < 0.5f) continue;

        FullPointType fp;
        fp.x = x;
        fp.y = y;
        fp.z = z;
        fp.range = range;

        if (intensity_off >= 0) {
            switch (intensity_type) {
                case PF_FLOAT32: {
                    float v;
                    std::memcpy(&v, pt + intensity_off, 4);
                    fp.intensity = static_cast<uint8_t>(std::min(v, 255.0f));
                } break;
                case PF_FLOAT64: {
                    double v;
                    std::memcpy(&v, pt + intensity_off, 8);
                    fp.intensity = static_cast<uint8_t>(std::min(v, 255.0));
                } break;
                case PF_UINT8:
                    fp.intensity = pt[intensity_off];
                    break;
                case PF_UINT16: {
                    uint16_t v;
                    std::memcpy(&v, pt + intensity_off, 2);
                    fp.intensity = static_cast<uint8_t>(std::min<uint16_t>(v, 255));
                } break;
                case PF_UINT32: {
                    uint32_t v;
                    std::memcpy(&v, pt + intensity_off, 4);
                    fp.intensity = static_cast<uint8_t>(std::min<uint32_t>(v, 255));
                } break;
                default:
                    fp.intensity = 0;
            }
        }

        if (time_off >= 0) {
            switch (time_type) {
                case PF_FLOAT32: {
                    float v;
                    std::memcpy(&v, pt + time_off, 4);
                    fp.time = v * time_scale;
                } break;
                case PF_FLOAT64: {
                    double v;
                    std::memcpy(&v, pt + time_off, 8);
                    fp.time = v ;//* time_scale;
                } break;
                case PF_UINT32: {
                    uint32_t v;
                    std::memcpy(&v, pt + time_off, 4);
                    fp.time = v / 1e6;  // ns → ms
                } break;
                default:
                    fp.time = 0;
            }
        }

        if (ring_off >= 0) {
            switch (ring_type) {
                case PF_UINT8:
                    fp.ring = pt[ring_off];
                    break;
                case PF_UINT16: {
                    uint16_t v;
                    std::memcpy(&v, pt + ring_off, 2);
                    fp.ring = static_cast<uint8_t>(v);
                } break;
                default:
                    fp.ring = 0;
            }
            if (fp.ring >= num_scans) continue;
        }

        cloud->push_back(fp);
    }
    return cloud;
}

// ---------------------------------------------------------------------------
// Parse livox_ros_driver2/msg/CustomMsg → FullCloudPtr
//
// Message layout (CDR):
//   Header header
//   uint64 timebase        (ns since epoch, scan start)
//   uint32 point_num
//   uint8  lidar_id
//   uint8[3] rsvd
//   CustomPoint[] points   (CDR sequence)
//
// CustomPoint layout:
//   uint32  offset_time    (ns from timebase)
//   float32 x, y, z
//   uint8   reflectivity
//   uint8   tag
//   uint8   line
// ---------------------------------------------------------------------------
FullCloudPtr parse_livox_custom(const uint8_t* data, size_t size, double& timestamp_out,
                                int point_filter_num, int num_scans) {
    rosbag2::CdrReader r(data, size);
    auto stamp = rosbag2::read_header(r);
    timestamp_out = stamp.to_sec();

    uint64_t timebase = r.read_uint64();
    (void)timebase;
    /*point_num=*/r.read_uint32();
    /*lidar_id=*/r.read_uint8();
    // rsvd[3] — fixed-size array, no length prefix
    r.read_uint8();
    r.read_uint8();
    r.read_uint8();

    // CustomPoint[] — CDR sequence
    uint32_t num_points = r.read_uint32();

    auto cloud = std::make_shared<FullPointCloudType>();
    cloud->reserve(num_points / std::max(point_filter_num, 1) + 1);

    for (uint32_t i = 0; i < num_points; i++) {
        uint32_t offset_time = r.read_uint32();
        float x = r.read_float32();
        float y = r.read_float32();
        float z = r.read_float32();
        uint8_t reflectivity = r.read_uint8();
        /*tag=*/r.read_uint8();
        uint8_t line = r.read_uint8();

        if (line >= num_scans) continue;
        if (static_cast<int>(i) % point_filter_num != 0) continue;
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
        float range = std::sqrt(x * x + y * y + z * z);
        if (range < 0.5f) continue;

        FullPointType fp;
        fp.x = x;
        fp.y = y;
        fp.z = z;
        fp.intensity = reflectivity;
        fp.time = offset_time / 1e6;  // ns → ms
        fp.ring = line;
        fp.range = range;
        cloud->push_back(fp);
    }
    return cloud;
}

// ---------------------------------------------------------------------------
// Auto-detect topics from the bag
// ---------------------------------------------------------------------------
struct DetectedTopics {
    const rosbag2::TopicInfo* imu = nullptr;
    const rosbag2::TopicInfo* lidar = nullptr;
    bool is_livox_custom = false;
};

DetectedTopics auto_detect_topics(const rosbag2::BagReader& bag) {
    DetectedTopics det;
    for (const auto& [id, ti] : bag.topics()) {
        // IMU
        if (ti.type == "sensor_msgs/msg/Imu") {
            if (!det.imu) det.imu = &ti;
        }
        // Livox CustomMsg (preferred over PointCloud2 if both exist)
        if (ti.type == "livox_ros_driver2/msg/CustomMsg" ||
            ti.type == "livox_interfaces/msg/CustomMsg") {
            det.lidar = &ti;
            det.is_livox_custom = true;
        }
        // PointCloud2 (fallback if no CustomMsg)
        if (ti.type == "sensor_msgs/msg/PointCloud2" && !det.is_livox_custom) {
            if (!det.lidar) det.lidar = &ti;
        }
    }
    return det;
}

// ---------------------------------------------------------------------------
// Print usage
// ---------------------------------------------------------------------------
void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " <bag_path> <config.yaml> [options]\n"
              << "\n"
              << "Options:\n"
              << "  --imu-topic <topic>     IMU topic name (default: auto-detect)\n"
              << "  --lidar-topic <topic>   LiDAR topic name (default: auto-detect)\n"
              << "  --imu-coefficitent <1.0> for livox imu data ( default: 1.0, )\n"
              << "  --output <file>         Save trajectory to TUM-format file\n"
              << "  -h, --help              Show this help\n";
}

// ---------------------------------------------------------------------------
// Save pose in TUM format: timestamp tx ty tz qx qy qz qw
// ---------------------------------------------------------------------------
void save_pose_tum(std::ofstream& ofs, double timestamp, const SE3& pose) {
    auto t = pose.translation();
    Quatd q = pose.unit_quaternion();
    ofs << std::fixed << std::setprecision(6) << timestamp << " " << std::setprecision(9) << t.x()
        << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << "\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string bag_path = argv[1];
    std::string config_yaml = argv[2];
    std::string imu_topic_override;
    std::string lidar_topic_override;
    double imu_coeff = 1.0;
    std::string output_file;

    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "--imu-topic") && i + 1 < argc)
            imu_topic_override = argv[++i];
        else if ((arg == "--lidar-topic") && i + 1 < argc)
            lidar_topic_override = argv[++i];
        else if ((arg == "--imu-coefficient") && i + 1 < argc)
            imu_coeff = std::stod(argv[++i]);
        else if ((arg == "--output" || arg == "-o") && i + 1 < argc)
            output_file = argv[++i];
        else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // ── Open bag ──────────────────────────────────────────────────────────
    spdlog::info("Opening bag: {}", bag_path);
    rosbag2::BagReader bag;
    bag.open(bag_path);

    spdlog::info("Topics in bag:");
    for (const auto& [id, ti] : bag.topics()) {
        int64_t count = bag.count_messages(ti.id);
        spdlog::info("  [{}] {} ({}) — {} messages", ti.id, ti.name, ti.type, count);
    }

    // ── Resolve topics ───────────────────────────────────────────────────
    auto detected = auto_detect_topics(bag);
    bool is_livox_custom = detected.is_livox_custom;

    const rosbag2::TopicInfo* imu_topic = nullptr;
    const rosbag2::TopicInfo* lidar_topic = nullptr;

    if (!imu_topic_override.empty()) {
        imu_topic = bag.find_topic(imu_topic_override);
        if (!imu_topic) {
            spdlog::error("IMU topic '{}' not found in bag", imu_topic_override);
            return 1;
        }
    } else {
        imu_topic = detected.imu;
    }

    if (!lidar_topic_override.empty()) {
        lidar_topic = bag.find_topic(lidar_topic_override);
        if (!lidar_topic) {
            spdlog::error("LiDAR topic '{}' not found in bag", lidar_topic_override);
            return 1;
        }
        // Check if the overridden topic is Livox CustomMsg
        is_livox_custom = (lidar_topic->type == "livox_ros_driver2/msg/CustomMsg" ||
                           lidar_topic->type == "livox_interfaces/msg/CustomMsg");
    } else {
        lidar_topic = detected.lidar;
    }

    if (!imu_topic) {
        spdlog::error("No IMU topic found. Use --imu-topic to specify.");
        return 1;
    }
    if (!lidar_topic) {
        spdlog::error("No LiDAR topic found. Use --lidar-topic to specify.");
        return 1;
    }

    spdlog::info("IMU topic:   {} ({})", imu_topic->name, imu_topic->type);
    spdlog::info("LiDAR topic: {} ({}){}",
                 lidar_topic->name, lidar_topic->type,
                 is_livox_custom ? " [Livox CustomMsg]" : "");

    // ── Load config & init LIO ───────────────────────────────────────────
    auto yaml = YAML::LoadFile(config_yaml);
    int num_scans = yaml["preprocess"]["scan_line"] ? yaml["preprocess"]["scan_line"].as<int>() : 6;
    double time_scale = yaml["preprocess"]["time_scale"] ? yaml["preprocess"]["time_scale"].as<double>() : 1e-3;
    int point_filter_num = yaml["point_filter_num"] ? yaml["point_filter_num"].as<int>() : 4;

    spdlog::info("num_scans: {}, time_scale: {}, point_filter_num: {}", num_scans, time_scale, point_filter_num);

    LIOConfig lio_config;
    lio_config.with_ui = yaml["with_ui"] ? yaml["with_ui"].as<bool>() : false;
    lio_config.imu_config.init_time_seconds = yaml["imu_init_time"] ? yaml["imu_init_time"].as<double>() : 5.0;
    lio_config.imu_config.max_static_gyro_var = yaml["max_static_gyro_var"] ? yaml["max_static_gyro_var"].as<double>() : 0.1;
    lio_config.imu_config.max_static_acce_var = yaml["max_static_acce_var"] ? yaml["max_static_acce_var"].as<double>() : 0.05;
    // IMU-LiDAR extrinsics
    if (yaml["mapping"]) {
        auto ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        auto ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
        Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
        Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
        lio_config.T_imu_lidar = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    } else {
        spdlog::warn("No extrinsic calibration found in config. Assuming identity.");
        lio_config.T_imu_lidar = SE3();
    }

    LIO lio(lio_config);
    if (!lio.Init(config_yaml)) {
        spdlog::error("LIO initialization failed");
        return 1;
    }

    // ── Output file ──────────────────────────────────────────────────────
    std::ofstream traj_ofs;
    if (!output_file.empty()) {
        traj_ofs.open(output_file);
        if (!traj_ofs.is_open()) {
            spdlog::error("Cannot open output file: {}", output_file);
            return 1;
        }
        spdlog::info("Saving trajectory to: {}", output_file);
    }

    // ── Play messages ────────────────────────────────────────────────────
    int imu_count = 0, lidar_count = 0;

    bag.for_each_message(
        {imu_topic->id, lidar_topic->id},
        [&](const rosbag2::RawMessage& msg) {
            if (msg.topic_id == imu_topic->id) {
                auto imu = parse_imu(msg.data.data(), msg.data.size(), imu_coeff);
                lio.AddIMU(imu);
                imu_count++;
            } else if (msg.topic_id == lidar_topic->id) {
                double ts = 0;
                FullCloudPtr cloud;

                if (is_livox_custom) {
                    cloud = parse_livox_custom(msg.data.data(), msg.data.size(), ts,
                                              point_filter_num, num_scans);
                } else {
                    cloud = parse_pointcloud2(msg.data.data(), msg.data.size(), ts,
                                             point_filter_num, num_scans, time_scale);
                }

                if (cloud && !cloud->empty()) {
                    lio.AddCloud(cloud, ts);
                    lidar_count++;

                    if (lio.IsInitialized()) {
                        SE3 pose = lio.GetCurrentPose();
                        auto t = pose.translation();
                        if (lidar_count % 10 == 0) {
                            spdlog::info("Frame {:>5}: pos=[{:.2f}, {:.2f}, {:.2f}]  pts={}",
                                         lidar_count, t.x(), t.y(), t.z(), cloud->size());
                        }
                        if (traj_ofs.is_open()) {
                            save_pose_tum(traj_ofs, ts, pose);
                        }
                    }
                }
            }
        });

    // ── Done ─────────────────────────────────────────────────────────────
    
    spdlog::info("Playback complete: {} IMU messages, {} LiDAR scans", imu_count, lidar_count);
    if (lio.IsInitialized()) {
        SE3 final_pose = lio.GetCurrentPose();
        auto t = final_pose.translation();
        spdlog::info("Final pose: [{:.4f}, {:.4f}, {:.4f}]", t.x(), t.y(), t.z());
    } else {
        spdlog::warn("IMU never initialized — not enough static IMU data?");
    }

    if (traj_ofs.is_open()) {
        traj_ofs.close();
        spdlog::info("Trajectory saved to: {}", output_file);
    }

    lio.Finish();

    return 0;
}
