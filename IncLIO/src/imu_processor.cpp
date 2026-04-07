#include "inclio/imu_processor.hpp"
#include <sstream>

namespace IncLIO {

bool IMUProcessor::AddIMUForInit(const IMU& imu) {
    if (init_success_) {
        return true;
    }

    if (config_.use_speed_for_static_checking && !is_static_) {
        INCLIO_WARN("Waiting for vehicle to be static");
        init_imu_deque_.clear();
        return false;
    }

    if (init_imu_deque_.empty()) {
        init_start_time_ = imu.timestamp_;
    }

    init_imu_deque_.push_back(imu);

    double init_time = imu.timestamp_ - init_start_time_;
    if (init_time > config_.init_time_seconds) {
        TryInit();
    }

    // Maintain initialization queue length
    while (static_cast<int>(init_imu_deque_.size()) > config_.init_imu_queue_max_size) {
        init_imu_deque_.pop_front();
    }

    current_time_ = imu.timestamp_;
    return false;
}

bool IMUProcessor::AddOdomForInit(const Odom& odom) {
    if (init_success_) {
        return true;
    }

    if (odom.left_pulse_ < config_.static_odom_pulse && odom.right_pulse_ < config_.static_odom_pulse) {
        is_static_ = true;
    } else {
        is_static_ = false;
    }

    current_time_ = odom.timestamp_;
    return true;
}

bool IMUProcessor::TryInit() {
    if (init_imu_deque_.size() < 10) {
        return false;
    }

    // Compute mean and variance of gyro and accel readings
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_,
                                [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [](const IMU& imu) { return imu.acce_; });

    // Take mean acce as direction, use configured gravity norm
    INCLIO_INFO("mean acce: [{:.4f}, {:.4f}, {:.4f}] norm: {:.4f}",
                mean_acce.x(), mean_acce.y(), mean_acce.z(), mean_acce.norm());
    gravity_ = -mean_acce / mean_acce.norm() * config_.gravity_norm;

    // Recompute acce covariance with gravity removed
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });

    // Check IMU noise levels
    if (cov_gyro_.norm() > config_.max_static_gyro_var) {
        INCLIO_ERROR("Gyro noise too large: {} > {}", cov_gyro_.norm(), config_.max_static_gyro_var);
        return false;
    }

    if (cov_acce_.norm() > config_.max_static_acce_var) {
        INCLIO_ERROR("Accel noise too large: {} > {}", cov_acce_.norm(), config_.max_static_acce_var);
        return false;
    }

    // Compute actual IMU measurement interval from init buffer
    if (init_imu_deque_.size() >= 2) {
        double total_dt = init_imu_deque_.back().timestamp_ - init_imu_deque_.front().timestamp_;
        imu_dt_ = total_dt / static_cast<double>(init_imu_deque_.size() - 1);
        INCLIO_INFO("measured IMU dt: {:.6f} s ({:.1f} Hz)", imu_dt_, 1.0 / imu_dt_);
    }

    // Estimate biases
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    INCLIO_INFO("IMU initialization successful, time={:.2f}s, bg=[{:.6f},{:.6f},{:.6f}], ba=[{:.6f},{:.6f},{:.6f}], gravity=[{:.4f},{:.4f},{:.4f}] (norm={:.4f})",
                current_time_ - init_start_time_,
                init_bg_.x(), init_bg_.y(), init_bg_.z(),
                init_ba_.x(), init_ba_.y(), init_ba_.z(),
                gravity_.x(), gravity_.y(), gravity_.z(), gravity_.norm());

    init_success_ = true;
    return true;
}

} // namespace IncLIO
