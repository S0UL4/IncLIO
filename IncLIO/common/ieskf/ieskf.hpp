#ifndef INCLIO_IESKF_HPP
#define INCLIO_IESKF_HPP

#include "utils/eigen_types.hpp"
#include "utils/logger.hpp"
#include "utils/math_utils.hpp"
#include "inclio/imu.hpp"
#include "inclio/state.hpp"

namespace IncLIO {

// TODO: Define the IESKF (Iterated Error-State Kalman Filter)
//
// The IESKF operates on a manifold state (with SO3 rotation) and iterates
// the update step to handle nonlinear observation models (e.g. NDT).
//
// Key components:
//   - State:      position, velocity, rotation (SO3), accel bias, gyro bias, gravity
//   - ErrorState: tangent-space error (dx) of dimension N (e.g. 18 or 15)
//   - Predict:    IMU-driven propagation of state and covariance
//   - Update:     Iterated measurement update with generic observation model
//
// The observation model is injected via a callback / functor so that
// this filter remains decoupled from NDT or any specific registration.

template <typename S>
class IESKF {
   public:
    using SO3 = Sophus::SO3<S>;                     // Rotation variable type
    using VecT = Eigen::Matrix<S, 3, 1>;            // Vector type
    using Vec18T = Eigen::Matrix<S, 18, 1>;         // 18-dimensional vector type
    using Mat3T = Eigen::Matrix<S, 3, 3>;           // 3x3 matrix type
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // Motion noise type
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;      // Odometry noise type
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // GNSS noise type
    using Mat18T = Eigen::Matrix<S, 18, 18>;        // 18-dimensional covariance type
    using StateT = State<S>;                  // Overall nominal state variable type

    struct Options {
        Options() = default;
        /// IEKF configuration
        int num_iterations_ = 3;  // number of iterations
        double quit_eps_ = 1e-3;  // termination threshold for iterations

        /// IMU measurements and biases
        double imu_dt_ = 0.01;         // IMU measurement interval
        double gyro_var_ = 1e-5;       // gyroscope measurement variance
        double acce_var_ = 1e-2;       // accelerometer measurement variance
        double bias_gyro_var_ = 1e-6;  // gyroscope bias random walk variance
        double bias_acce_var_ = 1e-4;  // accelerometer bias random walk variance

        // RTK observation parameters
        double gnss_pos_noise_ = 0.1;                   // GNSS position noise
        double gnss_height_noise_ = 0.1;                // GNSS height noise
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS rotation noise

        // update bias or not
        bool update_bias_gyro_ = true;  // whether to update gyro bias
        bool update_bias_acce_ = true;  // whether to update accelerometer bias
    };

    /**
     * Initial bias is set to zero
     */
    IESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

    /**
     * Initial bias is set by external input
     * @param init_bg
     * @param init_ba
     * @param gravity
     */
    IESKF(Options options, const VecT& init_bg, const VecT& init_ba, const VecT& gravity = VecT(0, 0, -9.8))
        : options_(options) {
        BuildNoise(options);
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
    }

    // Set initial conditions for the filter
    void SetInitialConditions(Options options, const VecT& init_bg, const VecT& init_ba,
                              const VecT& gravity = VecT(0, 0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;

        cov_ = 1e-4 * Mat18T::Identity();
        cov_.template block<3, 3>(6, 6) = 0.1 * math::kDEG2RAD * Mat3T::Identity();
    }

    // Propagate the state and covariance using IMU measurements
    bool Predict(const IMU& imu);

    /**
     * NDT observation function, takes an SE3 pose as input and returns the terms HT_Vinv_H and HT_Vinv_r for the Kalman update
     * HT V^{-1} H
     * H^T V{-1} r
     * both can be implemented in a summation form
     */
    using CustomObsFunc = std::function<void(const SE3& input_pose, Eigen::Matrix<S, 18, 18>& HT_Vinv_H,
                                             Eigen::Matrix<S, 18, 1>& HT_Vinv_r)>;

    /// Update the filter using a custom observation function
    bool UpdateUsingCustomObserve(CustomObsFunc obs);

    /// accessors
    /// Get the full nominal state as a NavStateT struct
    StateT GetNominalState() const { return StateT(current_time_, R_, p_, v_, bg_, ba_); }

    /// Get the nominal pose as an SE3
    SE3 GetNominalSE3() const { return SE3(R_, p_); }

    void SetX(const StateT& x) {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
    }

    void SetR(const SO3& R) { R_ = R; }

    void SetCov(const Mat18T& cov) { cov_ = cov; }
    Vec3d GetGravity() const { return g_; }

   private:
    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev * ev;
        double et2 = et * et;
        double eg2 = eg * eg;
        double ea2 = ea * ea;

        // set Q
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_; // position noise variance
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_; // height noise variance
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_; // rotation noise variance
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;  
    }

    /// Update the nominal state variables using the current error state (dx)
    void Update() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (options_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }

        if (options_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }
        g_ += dx_.template block<3, 1>(15, 0);
    }

    double current_time_ = 0.0;

    // nominal state
    SO3 R_;
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    VecT bg_ = VecT::Zero();
    VecT ba_ = VecT::Zero();
    VecT g_{0, 0, -9.8};

    // error state
    Vec18T dx_ = Vec18T::Zero();

    // covariance
    Mat18T cov_ = Mat18T::Identity();

    // noise
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    Options options_;
};

using IESKFD = IESKF<double>;
using IESKFF = IESKF<float>;

} // namespace IncLIO

#endif // INCLIO_IESKF_HPP
