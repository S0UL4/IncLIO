#include "inclio/imu_processor.hpp"
#include <sstream>

namespace IncLIO {

bool IMUProcessor::AddIMUForInit(const IMU& imu) {
    if (init_success_) {
        return true;
    }

    if (init_imu_deque_.empty()) {
        init_start_time_ = imu.timestamp_;
    }
    init_imu_deque_.push_back(imu);
    current_time_ = imu.timestamp_;

    if (!config_.use_static_init) {
        // Prior-based path: just need enough samples to measure imu_dt_
        if (static_cast<int>(init_imu_deque_.size()) >= 20) {
            return TryInit();
        }
        return false;
    }

    // Static init path
    if (config_.use_speed_for_static_checking && !is_static_) {
        INCLIO_WARN("Waiting for vehicle to be static");
        init_imu_deque_.clear();
        return false;
    }

    double init_time = imu.timestamp_ - init_start_time_;
    if (init_time > config_.init_time_seconds) {
        TryInit();
    }

    while (static_cast<int>(init_imu_deque_.size()) > config_.init_imu_queue_max_size) {
        init_imu_deque_.pop_front();
    }

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

    // Measure imu_dt_ from the collected buffer regardless of init mode
    if (init_imu_deque_.size() >= 2) {
        double total_dt = init_imu_deque_.back().timestamp_ - init_imu_deque_.front().timestamp_;
        imu_dt_ = total_dt / static_cast<double>(init_imu_deque_.size() - 1);
        INCLIO_INFO("measured IMU dt: {:.6f} s ({:.1f} Hz)", imu_dt_, 1.0 / imu_dt_);
    }

    if (!config_.use_static_init) {
        init_bg_ = config_.prior_bg;
        init_ba_ = config_.prior_ba;

        // Gravity: use prior if provided, else estimate from mean accel with a warning
        if (config_.prior_gravity.norm() > 1e-3) {
            gravity_ = config_.prior_gravity;
        } else {
            Vec3d mean_acce, dummy_cov;
            math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, dummy_cov,
                                        [](const IMU& imu) { return imu.acce_; });
            INCLIO_WARN("prior_gravity not set — estimating gravity from mean accel. "
                        "This is unreliable if the platform is moving at startup.");
            gravity_ = -mean_acce / mean_acce.norm() * config_.gravity_norm;
        }

        // Use conservative fixed noise since we cannot measure it from moving data
        cov_gyro_ = Vec3d::Constant(0.01);
        cov_acce_ = Vec3d::Constant(0.1);

        INCLIO_INFO("IMU init from priors — bg=[{:.6f},{:.6f},{:.6f}], "
                    "ba=[{:.6f},{:.6f},{:.6f}], gravity=[{:.4f},{:.4f},{:.4f}] (norm={:.4f})",
                    init_bg_.x(), init_bg_.y(), init_bg_.z(),
                    init_ba_.x(), init_ba_.y(), init_ba_.z(),
                    gravity_.x(), gravity_.y(), gravity_.z(), gravity_.norm());
        init_success_ = true;
        return true;
    }

    // Static init path: compute mean and variance from buffered data
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_,
                                [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [](const IMU& imu) { return imu.acce_; });

    INCLIO_INFO("mean acce: [{:.4f}, {:.4f}, {:.4f}] norm: {:.4f}",
                mean_acce.x(), mean_acce.y(), mean_acce.z(), mean_acce.norm());
    gravity_ = -mean_acce / mean_acce.norm() * config_.gravity_norm;

    // Recompute acce covariance with gravity removed
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });

    if (cov_gyro_.norm() > config_.max_static_gyro_var) {
        INCLIO_ERROR("Gyro noise too large: {} > {}", cov_gyro_.norm(), config_.max_static_gyro_var);
        return false;
    }

    if (cov_acce_.norm() > config_.max_static_acce_var) {
        INCLIO_ERROR("Accel noise too large: {} > {}", cov_acce_.norm(), config_.max_static_acce_var);
        return false;
    }

    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    INCLIO_INFO("IMU static init successful — time={:.2f}s, bg=[{:.6f},{:.6f},{:.6f}], "
                "ba=[{:.6f},{:.6f},{:.6f}], gravity=[{:.4f},{:.4f},{:.4f}] (norm={:.4f})",
                current_time_ - init_start_time_,
                init_bg_.x(), init_bg_.y(), init_bg_.z(),
                init_ba_.x(), init_ba_.y(), init_ba_.z(),
                gravity_.x(), gravity_.y(), gravity_.z(), gravity_.norm());
    init_success_ = true;
    return true;
}

} // namespace IncLIO
