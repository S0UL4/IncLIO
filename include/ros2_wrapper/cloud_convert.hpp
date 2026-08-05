#include <omp.h> // for parallel loops in cloud conversion
#pragma once

// cloud_convert.hpp
//
// Converts incoming ROS2 sensor messages into IncLIO's FullPointCloudType.
//
// Supported input formats:
//   - sensor_msgs/msg/PointCloud2  (Velodyne, Ouster, Hesai, Robosense, ...)
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
    UNKNOWN = 0, // not declared / not yet deduced from the stream
    LIVOX   = 1, // Livox CustomMsg
    VELO    = 2, // Velodyne
    OUST    = 3, // Ouster
    HESAI   = 4, // Hesai Pandar (and other absolute-timestamp clouds)
    GENERIC = 5, // a PointCloud2 we could parse but not attribute to a vendor
};

const char* ToString(LidarType type);

// How the per-point time channel is encoded. Deduced from the field table plus
// the observed value range; see the header comment above.
enum class PointTimeEncoding {
    kNone,      // no usable time channel -> reconstruct offsets from yaw
    kRelative,  // offset from the scan start/end, in some power-of-1000 unit
    kAbsolute,  // absolute Unix time, in seconds or ns
};

const char* ToString(PointTimeEncoding enc);

// ─────────────────────────────────────────────────────────────────────────────
// CloudConvertConfig
//
// Every field below is auto-detected when left at its "unset" value. The YAML
// keys exist purely as an escape hatch for streams that misreport themselves.
// ─────────────────────────────────────────────────────────────────────────────
struct CloudConvertConfig {
    LidarType lidar_type    = LidarType::UNKNOWN;  // UNKNOWN => deduce from the message
    int    num_scans        = 0;     // 0 => deduce (max ring + 1, or msg.height)
    double time_scale       = 0.0;   // raw time field -> milliseconds; 0 => deduce
    int    point_filter_num = 1;     // keep every N-th point (1 = keep all)
    double imu_coeff        = 0.0;   // accel scale; 0 => deduce (see IMUConverter)
    double blind            = 4.0;   // drop returns closer than this to the sensor [m]
};

// ─────────────────────────────────────────────────────────────────────────────
// CloudConverter
// ─────────────────────────────────────────────────────────────────────────────
class CloudConverter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit CloudConverter(const CloudConvertConfig& cfg = {}) : cfg_(cfg) {}

    void SetConfig(const CloudConvertConfig& cfg) { cfg_ = cfg; user_cfg_ = cfg; }
    const CloudConvertConfig& Config() const { return cfg_; }

    /**
     * sensor_msgs::PointCloud2 input. Detects the point layout on the first
     * message (and again whenever the field table changes).
     */
    void Process(const sensor_msgs::msg::PointCloud2 & msg, IncLIO::FullCloudPtr &pcl_out);

    /**
     * Load configuration from a YAML file. Every `preprocess:` key is optional;
     * anything missing (or set to the string "auto") is deduced from the stream.
     */
    void LoadFromYAML(const std::string &yaml);

    /**
     * Seconds to ADD to the message header stamp to obtain the true time of the
     * first point of the scan just converted.
     *
     * Emitted point times are always non-negative offsets from the scan start
     * (measurement_sync.cpp derives the scan end time from the last point), so
     * when the raw field is negative (Velodyne times its points from the scan
     * *end*) or absolute (Hesai), the shift that was applied to normalize them
     * has to be handed back to the caller instead of silently dropped.
     * Clamped to zero when the two clocks disagree by more than half a second,
     * which means the point stamps are simply not on the header's clock.
     */
    double LastTimeOffsetSeconds() const { return last_time_offset_s_; }

#ifdef HAVE_LIVOX_ROS_DRIVER2
    void Process(const livox_ros_driver2::msg::CustomMsg &msg, IncLIO::FullCloudPtr &pcl_out);
#endif

private:
    // One field of the incoming PointCloud2, resolved to a byte offset + datatype.
    struct FieldRef {
        int     offset   = -1;
        uint8_t datatype = 0;
        bool valid() const { return offset >= 0; }
    };

    // Everything deduced from the field table of the incoming clouds.
    struct PointLayout {
        bool     resolved = false;   // field offsets are known
        uint32_t point_step = 0;
        std::string signature;       // "name:datatype:offset|..." — change => re-detect

        FieldRef x, y, z, intensity, ring, time;
        std::string time_field;      // name of the time channel, for logging

        PointTimeEncoding encoding = PointTimeEncoding::kNone;
        bool   unit_resolved = false;  // the raw->ms scale has been confirmed against data
        double to_ms = 1.0;            // multiplier: raw time value -> milliseconds
        int    num_rings = 0;          // deduced ring count (yaw fallback needs it)
    };

    // Rebuilds layout_ from the message's field table. Returns false when the
    // cloud cannot be parsed at all (no x/y/z).
    bool DetectLayout(const sensor_msgs::msg::PointCloud2 &msg);

    // Confirms/corrects layout_.to_ms from the observed raw time span (relative
    // encodings) — the one thing the field table cannot tell us.
    void ResolveTimeUnit(double raw_span);

    void PointCloud2Handler(const sensor_msgs::msg::PointCloud2 &msg);
    void YawFallbackHandler(const sensor_msgs::msg::PointCloud2 &msg);

#ifdef HAVE_LIVOX_ROS_DRIVER2
    void AviaHandler(const livox_ros_driver2::msg::CustomMsg &msg);
#endif

    CloudConvertConfig cfg_;
    CloudConvertConfig user_cfg_;   // values explicitly pinned in YAML (0/UNKNOWN = auto)
    PointLayout        layout_;
    double             last_time_offset_s_ = 0.0;
    rclcpp::Clock      clock_{RCL_SYSTEM_TIME};   // for throttled logging only
    IncLIO::FullPointCloudType cloud_out_;
};

} // namespace inclio_ros2
