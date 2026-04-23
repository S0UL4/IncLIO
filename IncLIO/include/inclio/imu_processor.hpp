#ifndef INCLIO_IMU_PROCESSOR_HPP
#define INCLIO_IMU_PROCESSOR_HPP

#include "inclio/state.hpp"
#include "inclio/imu.hpp"
#include "inclio/odom.hpp"
#include "utils/eigen_types.hpp"
#include "utils/math_utils.hpp"
#include "utils/logger.hpp"
#include "utils/point_types.hpp"

#include <vector>
#include <deque>

namespace IncLIO {

// IMU Processor
//
// Responsibilities:
//   1. IMU initialization (estimate initial gravity, biases from static data)
//   2. Forward propagation of state between LiDAR scans via IESKF::Predict
//   3. Point cloud undistortion using IMU-propagated poses
//      (de-skew each point to the scan-end timestamp)

struct IMUProcessorConfig {
    // Init mode
    // true  → collect static data, estimate biases and gravity from IMU variance
    // false → use prior_bg / prior_ba immediately; only collect enough samples for imu_dt_
    bool use_static_init = true;

    // Static initialization parameters (used only when use_static_init = true)
    double init_time_seconds = 5.0;
    int init_imu_queue_max_size = 2000;
    int static_odom_pulse = 5;
    double max_static_gyro_var = 0.5;
    double max_static_acce_var = 1.0;
    bool use_speed_for_static_checking = true;

    double gravity_norm = 9.81;

    // Prior biases (used when use_static_init = false)
    Vec3d prior_bg = Vec3d::Zero();
    Vec3d prior_ba = Vec3d::Zero();

    // Prior gravity in body frame (used when use_static_init = false).
    // If zero, gravity direction is estimated from mean accelerometer — only valid
    // when the platform is stationary at startup. For moving-start bags set this
    // explicitly, e.g. {0, 0, -9.81} for a level-mounted IMU (z-down convention).
    Vec3d prior_gravity = Vec3d::Zero();
};

class IMUProcessor {
   public:
    IMUProcessor() = default;
    explicit IMUProcessor(const IMUProcessorConfig& config) : config_(config) {}

    // ── Initialization ──────────────────────────

    /// Add IMU sample for static initialization.
    /// Returns true when initialization succeeds.
    bool AddIMUForInit(const IMU& imu);

    /// Add odometry data for static state checking.
    bool AddOdomForInit(const Odom& odom);

    /// Whether static initialization has completed
    bool InitSuccess() const { return init_success_; }

    /// Initialization results
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }
    double GetIMUDt() const { return imu_dt_; }

    // ── Propagation & Undistortion ──────────────
    // TODO: void Process(const std::deque<IMU>& imu_buffer, double scan_end_time);
    // TODO: void UndistortCloud(const FullPointCloud& raw, PointCloud& corrected);

   private:
    /// Attempt initialization from buffered IMU data
    bool TryInit();

    IMUProcessorConfig config_;

    // Initialization state
    bool init_success_ = false;
    Vec3d cov_gyro_ = Vec3d::Zero();
    Vec3d cov_acce_ = Vec3d::Zero();
    Vec3d init_bg_ = Vec3d::Zero();
    Vec3d init_ba_ = Vec3d::Zero();
    Vec3d gravity_ = Vec3d::Zero();
    bool is_static_ = false;
    std::deque<IMU> init_imu_deque_;
    double current_time_ = 0.0;
    double init_start_time_ = 0.0;
    double imu_dt_ = 0.005;  // measured IMU period (seconds), updated during init

    // TODO: Propagation state
    // std::vector<State> propagated_states_;
};

} // namespace IncLIO

#endif // INCLIO_IMU_PROCESSOR_HPP
