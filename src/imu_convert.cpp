#include "ros2_wrapper/imu_convert.hpp"

namespace inclio_ros2 {

namespace {
rclcpp::Logger Log() { return rclcpp::get_logger("IMUConverter"); }

// Gravity is always present, so a stream in g-units reads ~1 and one in m/s²
// reads ~9.81. The two windows below are deliberately narrow and far apart:
// a single sample decides the scale, so it must not be possible for ordinary
// vehicle dynamics to push one regime into the other's window. A sample landing
// in the gap between them simply defers the decision to the next one.
constexpr double kGUnitLo   = 0.5, kGUnitHi   = 2.0;
constexpr double kMs2UnitLo = 6.0, kMs2UnitHi = 15.0;

// The audit that follows the decision runs on the MEAN over this many samples,
// not on individual ones — a hard brake or a pothole routinely puts a single
// |a| far from gravity, but the mean cannot leave the right regime.
constexpr int    kAuditSamples = 400;    // ~2 s at 200 Hz
constexpr double kAuditMeanLo  = 7.0;    // accepted band for mean(|a|) * coeff
constexpr double kAuditMeanHi  = 13.0;
} // namespace

void IMUConverter::UpdateCoeff(double accel_norm) {
    if (!decided_) {
        if (accel_norm > kGUnitLo && accel_norm < kGUnitHi) {
            imu_coeff_ = kGravity;
            decided_   = true;
            RCLCPP_INFO(Log(), "IMU accelerometer is in g (|a| = %.3f) — scaling by %.5f",
                        accel_norm, kGravity);
        } else if (accel_norm > kMs2UnitLo && accel_norm < kMs2UnitHi) {
            imu_coeff_ = 1.0;
            decided_   = true;
            RCLCPP_INFO(Log(), "IMU accelerometer is in m/s^2 (|a| = %.3f) — no scaling",
                        accel_norm);
        }
        // In the gap between the two windows: the sensor is accelerating hard,
        // saturated or not streaming gravity yet. Leave the coefficient at 1.0
        // (a no-op) and retry on the next sample.
        return;
    }

    // Post-decision audit: catch a verdict made from an atypical first sample.
    if (audited_ >= kAuditSamples) return;
    accel_sum_ += accel_norm;
    if (++audited_ < kAuditSamples) return;

    const double mean_ms2 = (accel_sum_ / kAuditSamples) * imu_coeff_;
    if (mean_ms2 < kAuditMeanLo || mean_ms2 > kAuditMeanHi) {
        RCLCPP_WARN(Log(),
            "IMU accelerometer scale looks wrong: mean |a| over %d samples is %.3f m/s^2 "
            "with coefficient %.5f, but gravity is %.3f. Pin preprocess.imu_coeff in the "
            "config if the odometry drifts.",
            kAuditSamples, mean_ms2, imu_coeff_, kGravity);
    }
}

void IMUConverter::Process(const sensor_msgs::msg::Imu & msg, IMUPtr &imu_out) {
    const double ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

    const auto& av = msg.angular_velocity;
    const auto& la = msg.linear_acceleration;

    if (auto_detect_) {
        UpdateCoeff(std::sqrt(la.x * la.x + la.y * la.y + la.z * la.z));
    }

    imu_out = std::make_shared<IncLIO::IMUData>(
        ts,
        Vec3d(av.x, av.y, av.z),
        Vec3d(la.x * imu_coeff_, la.y * imu_coeff_, la.z * imu_coeff_));
}

} // namespace inclio_ros2
