#ifndef INCLIO_MEASUREMENT_SYNC_HPP
#define INCLIO_MEASUREMENT_SYNC_HPP

#include "inclio/imu.hpp"
#include "utils/point_types.hpp"
#include "utils/logger.hpp"

#include <deque>
#include <functional>

namespace IncLIO {

/// A time-aligned group of measurements ready for the LIO pipeline.
/// Contains one LiDAR scan plus all IMU samples spanning that scan.
struct MeasureGroup {
    MeasureGroup() { this->lidar_.reset(new FullPointCloudType()); }
    double lidar_begin_time_ = 0;   // LiDAR packet start time
    double lidar_end_time_ = 0;     // LiDAR packet end time
    FullCloudPtr lidar_ = nullptr;  // LiDAR point cloud
    std::deque<IMUPtr> imu_;        // IMU measurements between frames
};

/// Synchronizes asynchronous IMU and LiDAR streams.
///
/// IMU and LiDAR arrive at different rates and different times.
/// This class buffers both and produces MeasureGroups:
///
///   timeline: ---|---i---i---i---i---[====LIDAR SCAN====]---i---i---i---[====LIDAR====]---
///                                    ^                   ^
///                              scan_start            scan_end
///
/// For each scan, it extracts all IMU samples between the previous
/// scan_end and the current scan_end, forming one MeasureGroup.
///
/// Usage:
///   MessageSync sync([&](const MeasureGroup& mg) { lio.ProcessMeasurements(mg); });
///   sync.Init(yaml_path);
///   // from sensor threads:
///   sync.ProcessIMU(imu);
///   sync.ProcessCloud(cloud, timestamp);
class MessageSync {
   public:
    using Callback = std::function<void(const MeasureGroup&)>;

    MessageSync(Callback cb) : callback_(cb) {}

    /// Initialize from YAML config
    void Init(const std::string& yaml);

    /// Buffer an IMU measurement
    void ProcessIMU(IMUPtr imu) {
        double timestamp = imu->timestamp_;
        if (timestamp < last_timestamp_imu_) {
            INCLIO_WARN("IMU loop back, clearing buffer");
            imu_buffer_.clear();
        }

        last_timestamp_imu_ = timestamp;
        imu_buffer_.emplace_back(imu);
    }

    /// Buffer a LiDAR scan (ROS-free interface)
    void ProcessCloud(FullCloudPtr cloud, double timestamp) {
        if (timestamp < last_timestamp_lidar_) {
            INCLIO_WARN("LiDAR loop back, clearing buffer");
            lidar_buffer_.clear();
            time_buffer_.clear();
        }

        lidar_buffer_.push_back(cloud);
        time_buffer_.push_back(timestamp);
        last_timestamp_lidar_ = timestamp;

        Sync();
    }

   private:
    /// Try to synchronize IMU and LiDAR data
    bool Sync();

    Callback callback_;
    std::deque<FullCloudPtr> lidar_buffer_;
    std::deque<IMUPtr> imu_buffer_;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_lidar_ = 0;
    std::deque<double> time_buffer_;
    bool lidar_pushed_ = false;
    MeasureGroup measures_;
    double lidar_end_time_ = 0;
};

} // namespace IncLIO

#endif // INCLIO_MEASUREMENT_SYNC_HPP
