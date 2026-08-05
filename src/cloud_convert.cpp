// cloud_convert.cpp
//
// Implements CloudConverter::Process() for PointCloud2 and (optionally)
// Livox CustomMsg. See cloud_convert.hpp for how the point layout, the
// per-point time encoding and its unit are deduced from the message itself.

#include "ros2_wrapper/cloud_convert.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstring>
#include <algorithm>
#include <limits>

namespace inclio_ros2 {

namespace {

using PointField = sensor_msgs::msg::PointField;

rclcpp::Logger Log() { return rclcpp::get_logger("CloudConverter"); }

// IncLIO expresses FullPointType::time in milliseconds from the scan start
// (measurement_sync.cpp / lio.cpp both multiply it by 1e-3). Every conversion
// below therefore produces milliseconds.
constexpr double kNsToMs = 1e-6;
constexpr double kUsToMs = 1e-3;
constexpr double kMsToMs = 1.0;
constexpr double kSToMs  = 1e3;

// A spinning LiDAR revolution lasts 10 ms (100 Hz) to 500 ms (2 Hz). Used to
// pick the unit of a relative time field: the four candidate units are 1000x
// apart, so at most one of them lands the observed span inside this window.
constexpr double kMinScanSpanMs = 1.0;
constexpr double kMaxScanSpanMs = 500.0;

// Beyond this the per-point stamps are simply not on the same clock as the
// message header, so the derived offset is dropped rather than applied.
constexpr double kMaxHeaderSkewS = 0.5;

// Reads a scalar of any PointField datatype from a (possibly unaligned) address.
template <typename T>
inline T ReadAt(const uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

inline double ReadField(const uint8_t* p, uint8_t datatype) {
    switch (datatype) {
        case PointField::INT8:    return static_cast<double>(ReadAt<int8_t>(p));
        case PointField::UINT8:   return static_cast<double>(*p);
        case PointField::INT16:   return static_cast<double>(ReadAt<int16_t>(p));
        case PointField::UINT16:  return static_cast<double>(ReadAt<uint16_t>(p));
        case PointField::INT32:   return static_cast<double>(ReadAt<int32_t>(p));
        case PointField::UINT32:  return static_cast<double>(ReadAt<uint32_t>(p));
        case PointField::FLOAT32: return static_cast<double>(ReadAt<float>(p));
        case PointField::FLOAT64: return ReadAt<double>(p);
        default:                  return 0.0;
    }
}

// Absolute Unix stamps: the epoch is ~1.8e9 s, so the magnitude alone pins the
// unit down (each candidate is three orders of magnitude from the next).
double AbsoluteStampToMs(double raw) {
    if (raw > 1e17) return kNsToMs;   // nanoseconds
    if (raw > 1e14) return kUsToMs;   // microseconds
    if (raw > 1e11) return kMsToMs;   // milliseconds
    return kSToMs;                    // seconds
}

} // namespace

const char* ToString(LidarType type) {
    switch (type) {
        case LidarType::LIVOX:   return "LIVOX";
        case LidarType::VELO:    return "VELODYNE";
        case LidarType::OUST:    return "OUSTER";
        case LidarType::HESAI:   return "HESAI";
        case LidarType::GENERIC: return "GENERIC PointCloud2";
        default:                 return "UNKNOWN";
    }
}

const char* ToString(PointTimeEncoding enc) {
    switch (enc) {
        case PointTimeEncoding::kRelative: return "relative-to-scan-start";
        case PointTimeEncoding::kAbsolute: return "absolute-unix";
        default:                           return "none (yaw reconstruction)";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LoadFromYAML
//
// Everything under `preprocess:` is optional. A missing key — or the string
// "auto" — leaves the value to be deduced from the incoming stream.
// ─────────────────────────────────────────────────────────────────────────────
void CloudConverter::LoadFromYAML(const std::string &yaml_file) {
    YAML::Node yaml = YAML::LoadFile(yaml_file);

    // Reads a numeric key that also accepts the string "auto" (or absence).
    auto read_opt_number = [](const YAML::Node& node, double& out) {
        if (!node || !node.IsScalar()) return false;
        const std::string s = node.as<std::string>();
        if (s.empty() || s == "auto" || s == "AUTO") return false;
        out = node.as<double>();
        return true;
    };

    if (yaml["point_filter_num"]) cfg_.point_filter_num = yaml["point_filter_num"].as<int>();
    if (cfg_.point_filter_num < 1) cfg_.point_filter_num = 1;

    const YAML::Node pre = yaml["preprocess"];
    if (pre) {
        // lidar_type accepts either the legacy integer or a sensor name.
        if (pre["lidar_type"] && pre["lidar_type"].IsScalar()) {
            const std::string s = pre["lidar_type"].as<std::string>();
            if (s == "1" || s == "livox")                        cfg_.lidar_type = LidarType::LIVOX;
            else if (s == "2" || s == "velodyne")                cfg_.lidar_type = LidarType::VELO;
            else if (s == "3" || s == "ouster")                  cfg_.lidar_type = LidarType::OUST;
            else if (s == "4" || s == "hesai")                   cfg_.lidar_type = LidarType::HESAI;
            else if (s == "auto" || s == "AUTO" || s.empty())    cfg_.lidar_type = LidarType::UNKNOWN;
            else RCLCPP_WARN(Log(), "Unknown lidar_type '%s' — auto-detecting instead", s.c_str());
        }

        double v = 0.0;
        if (read_opt_number(pre["scan_line"], v))  cfg_.num_scans  = static_cast<int>(v);
        if (read_opt_number(pre["time_scale"], v)) cfg_.time_scale = v;
        if (read_opt_number(pre["imu_coeff"], v))  cfg_.imu_coeff  = v;
        if (read_opt_number(pre["blind"], v))      cfg_.blind      = v;
    }

    // Remember what the user pinned: detection must never overwrite those.
    user_cfg_ = cfg_;

    if (cfg_.lidar_type == LidarType::UNKNOWN) {
        RCLCPP_INFO(Log(), "lidar_type: auto (deduced from the LiDAR topic)");
    } else {
        RCLCPP_INFO(Log(), "lidar_type: %s (pinned in config)", ToString(cfg_.lidar_type));
    }
    if (cfg_.time_scale > 0.0) {
        RCLCPP_INFO(Log(), "time_scale: %g (pinned in config, raw point time -> ms)",
                    cfg_.time_scale);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DetectLayout
//
// Resolves the byte offsets of x/y/z/intensity/ring/time from the message's own
// field table, picks the time encoding, and labels the vendor for logging.
// Re-runs whenever the field table changes (e.g. the topic is re-published by a
// different driver).
// ─────────────────────────────────────────────────────────────────────────────
bool CloudConverter::DetectLayout(const sensor_msgs::msg::PointCloud2 &msg) {
    std::string signature;
    signature.reserve(msg.fields.size() * 16);
    for (const auto& f : msg.fields) {
        signature += f.name;
        signature += ':';
        signature += std::to_string(f.datatype);
        signature += ':';
        signature += std::to_string(f.offset);
        signature += '|';
    }
    if (layout_.resolved && signature == layout_.signature &&
        msg.point_step == layout_.point_step) {
        return true;   // same producer as last time — nothing to redo
    }

    PointLayout layout;
    layout.signature  = signature;
    layout.point_step = msg.point_step;

    auto find = [&msg](std::initializer_list<const char*> names) -> CloudConverter::FieldRef {
        for (const char* want : names) {
            for (const auto& f : msg.fields) {
                if (f.name == want) return {static_cast<int>(f.offset), f.datatype};
            }
        }
        return {};
    };
    auto has = [&msg](const char* name) {
        return std::any_of(msg.fields.begin(), msg.fields.end(),
                           [name](const PointField& f) { return f.name == name; });
    };

    layout.x = find({"x"});
    layout.y = find({"y"});
    layout.z = find({"z"});
    if (!layout.x.valid() || !layout.y.valid() || !layout.z.valid()) {
        RCLCPP_ERROR(Log(), "PointCloud2 has no x/y/z fields — cannot convert (fields: %s)",
                     signature.c_str());
        return false;
    }

    layout.intensity = find({"intensity", "reflectivity", "i"});
    // Ouster/Robosense call the ring channel "channel", Livox's PointCloud2 mode
    // calls it "line", and NCLT bags call it "l".
    layout.ring      = find({"ring", "channel", "line", "l"});

    // Per-point time. Names are checked in order of decreasing specificity;
    // "timestamp*" is absolute Unix time, everything else is a scan-relative
    // offset.
    if (const FieldRef abs_t = find({"timestamp", "timestamp_ns"}); abs_t.valid()) {
        layout.time       = abs_t;
        layout.time_field = has("timestamp") ? "timestamp" : "timestamp_ns";
        layout.encoding   = PointTimeEncoding::kAbsolute;
    } else if (const FieldRef rel_t = find({"t", "time", "time_offset", "point_time_offset",
                                            "time_stamp"}); rel_t.valid()) {
        layout.time     = rel_t;
        layout.encoding = PointTimeEncoding::kRelative;
        for (const char* n : {"t", "time", "time_offset", "point_time_offset", "time_stamp"}) {
            if (has(n)) { layout.time_field = n; break; }
        }
        // Driver defaults, corrected against the observed span on the first
        // cloud (ResolveTimeUnit): Ouster ships uint32 nanoseconds, Velodyne
        // float32 seconds.
        layout.to_ms = (rel_t.datatype == PointField::UINT32 ||
                        rel_t.datatype == PointField::INT32) ? kNsToMs : kSToMs;
    } else {
        layout.encoding = PointTimeEncoding::kNone;
    }

    // Ring count — only the yaw fallback needs it, but it is cheap to record.
    if (msg.height > 1 && msg.width > 1) {
        layout.num_rings = static_cast<int>(msg.height);   // organized cloud: one row per ring
    } else if (layout.ring.valid()) {
        const size_t n = static_cast<size_t>(msg.width) * msg.height;
        const uint8_t* base = msg.data.data();
        int max_ring = 0;
        for (size_t i = 0; i < n; ++i) {
            const int r = static_cast<int>(
                ReadField(base + i * msg.point_step + layout.ring.offset, layout.ring.datatype));
            max_ring = std::max(max_ring, r);
        }
        layout.num_rings = max_ring + 1;
    }

    // Vendor label — logging only; the conversion itself is vendor-agnostic.
    LidarType detected = LidarType::GENERIC;
    if (has("tag") && has("line")) {
        detected = LidarType::LIVOX;   // Livox driver in PointCloud2 (non-CustomMsg) mode
    } else if (layout.encoding == PointTimeEncoding::kAbsolute) {
        detected = LidarType::HESAI;
    } else if (has("t") && (has("ambient") || has("reflectivity") || has("range"))) {
        detected = LidarType::OUST;
    } else if (has("time") || has("time_offset")) {
        detected = LidarType::VELO;
    }

    layout.resolved = true;
    layout_ = layout;

    if (user_cfg_.lidar_type == LidarType::UNKNOWN) cfg_.lidar_type = detected;
    if (user_cfg_.num_scans <= 0 && layout.num_rings > 0) cfg_.num_scans = layout.num_rings;
    if (user_cfg_.time_scale > 0.0) {
        layout_.to_ms         = user_cfg_.time_scale;
        layout_.unit_resolved = true;
    }
    cfg_.time_scale = layout_.to_ms;

    RCLCPP_INFO(Log(),
        "LiDAR stream detected: %s | %u pts/msg | time field: %s (%s) | rings: %d | "
        "intensity: %s",
        ToString(detected), msg.width * msg.height,
        layout.time.valid() ? layout.time_field.c_str() : "<none>",
        ToString(layout.encoding),
        cfg_.num_scans, layout.intensity.valid() ? "yes" : "no");

    if (layout.encoding == PointTimeEncoding::kNone) {
        RCLCPP_WARN(Log(),
            "No per-point time field in this cloud — deskewing will fall back to "
            "yaw-angle reconstruction%s.",
            layout.ring.valid() ? "" : " (and no ring field either: DESKEW DISABLED)");
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// ResolveTimeUnit
//
// The field table gives us the time channel but never its unit. A scan lasts
// 10-500 ms, and the candidate units are 1000x apart, so the observed span
// picks exactly one of them. Runs once, on the first cloud with a usable span.
// ─────────────────────────────────────────────────────────────────────────────
void CloudConverter::ResolveTimeUnit(double raw_span) {
    if (layout_.unit_resolved || raw_span <= 0.0) return;

    struct Candidate { const char* name; double to_ms; };
    static constexpr Candidate kUnits[] = {
        {"s", kSToMs}, {"ms", kMsToMs}, {"us", kUsToMs}, {"ns", kNsToMs},
    };

    for (const auto& c : kUnits) {
        const double span_ms = raw_span * c.to_ms;
        if (span_ms >= kMinScanSpanMs && span_ms <= kMaxScanSpanMs) {
            if (std::abs(c.to_ms - layout_.to_ms) > 1e-12 * std::max(1.0, layout_.to_ms)) {
                RCLCPP_WARN(Log(),
                    "Per-point time field is in %s, not the %g ms/unit assumed for this "
                    "datatype (scan span %.1f ms). Using the detected unit.",
                    c.name, layout_.to_ms, span_ms);
            } else {
                RCLCPP_INFO(Log(), "Per-point time unit: %s (scan span %.1f ms)", c.name, span_ms);
            }
            layout_.to_ms         = c.to_ms;
            layout_.unit_resolved = true;
            cfg_.time_scale       = c.to_ms;
            return;
        }
    }

    RCLCPP_WARN(Log(),
        "Per-point time span %g (raw) matches no plausible scan duration under any of "
        "s/ms/us/ns; keeping the datatype default (%g ms per unit). Set "
        "preprocess.time_scale explicitly if deskewing looks wrong.",
        raw_span, layout_.to_ms);
    layout_.unit_resolved = true;   // don't re-warn every scan
    cfg_.time_scale       = layout_.to_ms;
}

// ─────────────────────────────────────────────────────────────────────────────
// Process (PointCloud2)
// ─────────────────────────────────────────────────────────────────────────────
void CloudConverter::Process(const sensor_msgs::msg::PointCloud2 & msg,
                             IncLIO::FullCloudPtr &pcl_out) {
    cloud_out_.clear();
    last_time_offset_s_ = 0.0;

    if (!DetectLayout(msg)) {
        pcl_out = std::make_shared<IncLIO::FullPointCloudType>();
        return;
    }

    PointCloud2Handler(msg);

    cloud_out_.width    = cloud_out_.size();
    cloud_out_.height   = 1;
    cloud_out_.is_dense = true;
    pcl_out = std::make_shared<IncLIO::FullPointCloudType>(cloud_out_);
}

#ifdef HAVE_LIVOX_ROS_DRIVER2
void CloudConverter::Process(const livox_ros_driver2::msg::CustomMsg &msg,
                             IncLIO::FullCloudPtr &pcl_out) {
    cloud_out_.clear();
    last_time_offset_s_ = 0.0;
    AviaHandler(msg);
    pcl_out = std::make_shared<IncLIO::FullPointCloudType>(cloud_out_);
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// PointCloud2Handler
//
// Vendor-agnostic: reads straight out of the flat PointCloud2 buffer using the
// offsets resolved by DetectLayout, so it needs neither pcl::fromROSMsg nor a
// per-vendor point struct.
// ─────────────────────────────────────────────────────────────────────────────
void CloudConverter::PointCloud2Handler(const sensor_msgs::msg::PointCloud2 &msg) {
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    if (n == 0) return;

    const uint8_t* base = msg.data.data();
    const size_t   step = msg.point_step;

    // ── Pass 1: raw time range ───────────────────────────────────────────────
    // Needed to resolve the unit, to normalize the offsets to a zero start
    // (Velodyne times its points from the scan end, so they are negative) and,
    // for absolute stamps, to recover the scan start time itself.
    double t_min = 0.0, t_max = 0.0;
    if (layout_.time.valid()) {
        const int     off = layout_.time.offset;
        const uint8_t dt  = layout_.time.datatype;
        t_min = std::numeric_limits<double>::max();
        t_max = std::numeric_limits<double>::lowest();
        #pragma omp parallel for reduction(min:t_min) reduction(max:t_max) schedule(static)
        for (size_t i = 0; i < n; ++i) {
            const double v = ReadField(base + i * step + off, dt);
            if (v < t_min) t_min = v;
            if (v > t_max) t_max = v;
        }
    }

    if (layout_.encoding == PointTimeEncoding::kAbsolute && !layout_.unit_resolved) {
        layout_.to_ms         = AbsoluteStampToMs(t_max);
        layout_.unit_resolved = true;
        cfg_.time_scale       = layout_.to_ms;
        RCLCPP_INFO(Log(), "Absolute per-point stamps: %g ms per raw unit", layout_.to_ms);
    } else if (layout_.encoding == PointTimeEncoding::kRelative) {
        ResolveTimeUnit(t_max - t_min);
    }

    // A time field that never varies carries no deskew information (some
    // drivers publish a constant, e.g. NCLT-style Velodyne bags) — fall back to
    // reconstructing the offsets from the yaw angle, exactly as when the field
    // is missing entirely.
    if (!layout_.time.valid() || t_max <= t_min) {
        YawFallbackHandler(msg);
        return;
    }

    // Time of the first point relative to the message header stamp. Applied by
    // the caller (LioNode) to the scan start time; see LastTimeOffsetSeconds().
    const double header_s = static_cast<double>(msg.header.stamp.sec) +
                            static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    double offset_s = t_min * layout_.to_ms * 1e-3;
    if (layout_.encoding == PointTimeEncoding::kAbsolute) offset_s -= header_s;
    if (std::abs(offset_s) > kMaxHeaderSkewS) {
        RCLCPP_WARN_THROTTLE(Log(), clock_, 10000,
            "Per-point stamps lead/lag the header stamp by %.3f s — they are not on the "
            "header's clock; ignoring the offset and trusting the header.", offset_s);
        offset_s = 0.0;
    }
    last_time_offset_s_ = offset_s;

    // ── Pass 2: select the points that survive decimation + the blind zone ───
    const float  blind2 = static_cast<float>(cfg_.blind * cfg_.blind);
    const int    stride = std::max(1, cfg_.point_filter_num);
    const bool   xyz_f32 = layout_.x.datatype == PointField::FLOAT32 &&
                           layout_.y.datatype == PointField::FLOAT32 &&
                           layout_.z.datatype == PointField::FLOAT32;
    const int ox = layout_.x.offset, oy = layout_.y.offset, oz = layout_.z.offset;

    auto xyz_at = [&](const uint8_t* p, float& x, float& y, float& z) {
        if (xyz_f32) {
            x = ReadAt<float>(p + ox);
            y = ReadAt<float>(p + oy);
            z = ReadAt<float>(p + oz);
        } else {
            x = static_cast<float>(ReadField(p + ox, layout_.x.datatype));
            y = static_cast<float>(ReadField(p + oy, layout_.y.datatype));
            z = static_cast<float>(ReadField(p + oz, layout_.z.datatype));
        }
    };

    std::vector<size_t> kept;
    kept.reserve(n / stride + 1);
    for (size_t i = 0; i < n; i += stride) {
        float x, y, z;
        xyz_at(base + i * step, x, y, z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (x * x + y * y + z * z < blind2) continue;
        kept.push_back(i);
    }
    cloud_out_.resize(kept.size());

    // ── Pass 3: parallel fill of the pre-sized buffer ────────────────────────
    const int  off_t = layout_.time.offset;
    const uint8_t dt_t = layout_.time.datatype;
    const bool has_i = layout_.intensity.valid();
    const bool has_r = layout_.ring.valid();
    const double to_ms = layout_.to_ms;

    #pragma omp parallel for schedule(static)
    for (size_t j = 0; j < kept.size(); ++j) {
        const uint8_t* p  = base + kept[j] * step;
        auto&          pt = cloud_out_.points[j];

        float x, y, z;
        xyz_at(p, x, y, z);
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = has_i ? static_cast<float>(ReadField(p + layout_.intensity.offset,
                                                           layout_.intensity.datatype))
                             : 0.0f;
        pt.ring = has_r ? static_cast<uint16_t>(ReadField(p + layout_.ring.offset,
                                                         layout_.ring.datatype))
                        : 0;
        pt.time = (ReadField(p + off_t, dt_t) - t_min) * to_ms;   // ms from scan start
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// YawFallbackHandler
//
// Used when the cloud carries no usable per-point time. Reconstructs the offset
// of each point from how far its ring has rotated since that ring's first
// return, assuming a constant spin rate. Sequential by nature (per-ring state).
// ─────────────────────────────────────────────────────────────────────────────
void CloudConverter::YawFallbackHandler(const sensor_msgs::msg::PointCloud2 &msg) {
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    if (n == 0) return;

    const uint8_t* base   = msg.data.data();
    const size_t   step   = msg.point_step;
    const float    blind2 = static_cast<float>(cfg_.blind * cfg_.blind);
    const int      stride = std::max(1, cfg_.point_filter_num);
    const bool     has_i  = layout_.intensity.valid();

    cloud_out_.reserve(n / stride + 1);

    // Without a ring channel every point is treated as belonging to one ring, so
    // the ramp is driven by the azimuth of the cloud as a whole. That is still a
    // fair approximation for a spinning sensor publishing in acquisition order,
    // but it degrades once the rings are interleaved.
    if (!layout_.ring.valid()) {
        RCLCPP_WARN_ONCE(Log(),
            "Cloud has neither a per-point time nor a ring channel — deskew offsets are "
            "reconstructed from azimuth alone and assume a 10 Hz spin.");
    }

    const int num_scans = layout_.ring.valid() ? std::max(1, cfg_.num_scans) : 1;
    const double omega_l = 3.61;   // deg/ms at 10 Hz — the usual spinning-LiDAR rate
    std::vector<bool>   is_first(num_scans, true);
    std::vector<double> yaw_fp(num_scans, 0.0);
    std::vector<double> time_last(num_scans, 0.0);

    for (size_t i = 0; i < n; ++i) {
        const uint8_t* p = base + i * step;

        IncLIO::FullPointType pt;
        pt.x = static_cast<float>(ReadField(p + layout_.x.offset, layout_.x.datatype));
        pt.y = static_cast<float>(ReadField(p + layout_.y.offset, layout_.y.datatype));
        pt.z = static_cast<float>(ReadField(p + layout_.z.offset, layout_.z.datatype));
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        if (pt.getVector3fMap().squaredNorm() < blind2) continue;

        pt.intensity = has_i ? static_cast<float>(ReadField(p + layout_.intensity.offset,
                                                           layout_.intensity.datatype))
                             : 0.0f;

        const int layer = layout_.ring.valid()
            ? static_cast<int>(ReadField(p + layout_.ring.offset, layout_.ring.datatype))
            : 0;
        if (layer < 0 || layer >= num_scans) continue;
        pt.ring = static_cast<uint16_t>(layer);

        const double yaw = std::atan2(pt.y, pt.x) * 57.2957;
        if (is_first[layer]) {
            yaw_fp[layer]    = yaw;
            is_first[layer]  = false;
            time_last[layer] = 0.0;
            pt.time          = 0.0;
            continue;
        }

        pt.time = (yaw <= yaw_fp[layer]) ? (yaw_fp[layer] - yaw) / omega_l
                                         : (yaw_fp[layer] - yaw + 360.0) / omega_l;
        if (pt.time < time_last[layer]) pt.time += 360.0 / omega_l;   // wrapped past a full turn
        time_last[layer] = pt.time;

        if (i % static_cast<size_t>(stride) == 0) cloud_out_.points.push_back(pt);
    }
}

#ifdef HAVE_LIVOX_ROS_DRIVER2
// ─────────────────────────────────────────────────────────────────────────────
// AviaHandler
//
// Livox CustomMsg is not self-describing, but it does not need to be: the
// layout is fixed by the driver and `offset_time` is always nanoseconds from
// the message stamp.
// ─────────────────────────────────────────────────────────────────────────────
void CloudConverter::AviaHandler(const livox_ros_driver2::msg::CustomMsg &msg) {
    const int plsize = static_cast<int>(msg.point_num);
    if (plsize <= 1) return;

    if (cfg_.lidar_type != LidarType::LIVOX) {
        cfg_.lidar_type = LidarType::LIVOX;
        RCLCPP_INFO(Log(), "LiDAR stream detected: LIVOX CustomMsg (offset_time in ns)");
    }
    // Fixed by the driver: ns -> ms. Overridable, like every other scale.
    const double to_ms = (user_cfg_.time_scale > 0.0) ? user_cfg_.time_scale : kNsToMs;
    cfg_.time_scale = to_ms;

    IncLIO::FullPointCloudType cloud_temp;
    cloud_temp.resize(plsize);
    cloud_out_.reserve(plsize);

    // A line filter only makes sense when the beam count is known; with
    // scan_line left on auto every line is kept.
    const int  max_line   = cfg_.num_scans;
    const bool line_check = max_line > 0;
    const int  stride     = std::max(1, cfg_.point_filter_num);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < static_cast<uint>(plsize) - 1; ++i) index[i] = i + 1;

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if (line_check && msg.points[i].line >= max_line) return;
        // tag bits 4-5: 0x00 = confident return, 0x10 = moderate. Anything else
        // is noise (rain/dust/very-near) and is dropped.
        if ((msg.points[i].tag & 0x30) != 0x10 && (msg.points[i].tag & 0x30) != 0x00) return;
        if (i % static_cast<uint>(stride) != 0) return;

        auto& pt = cloud_temp.points[i];
        pt.x = msg.points[i].x;
        pt.y = msg.points[i].y;
        pt.z = msg.points[i].z;
        pt.ring = msg.points[i].line;
        pt.intensity = msg.points[i].reflectivity;
        pt.time = static_cast<double>(msg.points[i].offset_time) * to_ms;

        // Livox repeats the previous point when a beam gets no return.
        if ((std::abs(pt.x - cloud_temp.points[i - 1].x) > 1e-7f) ||
            (std::abs(pt.y - cloud_temp.points[i - 1].y) > 1e-7f) ||
            (std::abs(pt.z - cloud_temp.points[i - 1].z) > 1e-7f)) {
            is_valid_pt[i] = true;
        }
    });

    for (int i = 1; i < plsize; ++i) {
        if (is_valid_pt[i]) cloud_out_.points.push_back(cloud_temp.points[i]);
    }

    cloud_out_.width    = cloud_out_.size();
    cloud_out_.height   = 1;
    cloud_out_.is_dense = true;
}
#endif

} // namespace inclio_ros2
