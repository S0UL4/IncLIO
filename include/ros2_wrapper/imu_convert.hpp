#pragma once

// imu_convert.hpp
//
// Converts sensor_msgs/msg/Imu into IncLIO::IMUData.
//
// The only per-sensor unknown is the unit of the accelerometer channel: REP-145
// mandates m/s², but Livox (and a number of bag converters) publish it in
// multiples of g. That is self-evident from the data — gravity dominates any
// hand-held or vehicle motion — so the scale is deduced from the norm of the
// first samples instead of being configured:
//
//   |a| ~ 1     -> the stream is in g       -> multiply by 9.80665
//   |a| ~ 9.81  -> the stream is in m/s²    -> multiply by 1
//
// The two regimes are an order of magnitude apart, so a single sample settles
// it; the following samples are still checked, and a warning is raised if the
// stream turns out to disagree with the initial verdict.

#include <common/utils/point_types.hpp>  // FullPointCloudType, FullPointType, FullCloudPtr
#include <inclio/imu.hpp>  // IMU msg
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <cstdint>
#include <cstring>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <execution>


namespace inclio_ros2 {
// ─────────────────────────────────────────────────────────────────────────────
// IMUConverter
// ─────────────────────────────────────────────────────────────────────────────
class IMUConverter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit IMUConverter(double imu_coeff = 0.0) { SetIMUCoeff(imu_coeff); }

    /// coeff <= 0 selects auto-detection (the default); any positive value pins
    /// the accelerometer scale and disables detection.
    void SetIMUCoeff(double coeff) {
        auto_detect_ = (coeff <= 0.0);
        imu_coeff_   = auto_detect_ ? 1.0 : coeff;
        decided_     = !auto_detect_;
    }

    /// The scale currently in use (1.0 until the first sample in auto mode).
    double Coeff() const { return imu_coeff_; }

    void Process(const sensor_msgs::msg::Imu & msg, IMUPtr &imu_out);

private:
    // Picks imu_coeff_ from the accelerometer magnitude of one sample, then
    // audits the mean of the following samples against that choice.
    void UpdateCoeff(double accel_norm);

    static constexpr double kGravity = 9.80665;

    double imu_coeff_   = 1.0;   // g coeff (1.0 for m/s², 9.80665 for g-units)
    bool   auto_detect_ = true;
    bool   decided_     = false;
    int    audited_     = 0;     // samples accumulated since the initial decision
    double accel_sum_   = 0.0;   // sum of |a| over those samples
};

} // namespace inclio_ros2
