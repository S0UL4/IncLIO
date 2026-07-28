#include "ieskf/ieskf.hpp"

namespace IncLIO {

template <typename S>
bool IESKF<S>::Predict(const IMU& imu) {
    /// The predict step is the same as ESKF because the error state is defined in the tangent space of the manifold, so the IMU propagation is the same as ESKF, 
    /// and the only difference is that we need to use the exponential map to update the rotation part of the nominal state.
    assert(imu.timestamp_ >= current_time_);

    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        INCLIO_INFO("skipping IMU predict because dt is too large or negative: {:.3f} s", dt);
        current_time_ = imu.timestamp_;
        return false;
    }

    VecT acce = imu.acce_.template cast<S>();
    VecT gyro = imu.gyro_.template cast<S>();
    last_gyro_ = gyro;   // cache body angular rate for the wheel-odom update

    VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (acce - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    VecT new_v = v_ + R_ * (acce - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((gyro - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;

    Mat18T F = Mat18T::Identity();
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(acce - ba_) * dt;
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;
    F.template block<3, 3>(6, 6) = SO3::exp(-(gyro - bg_) * dt).matrix();
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;

    cov_ = F * cov_ * F.transpose() + Q_;
    current_time_ = imu.timestamp_;
    return true;
}
template <typename S>
bool IESKF<S>::UpdateUsingCustomObserve(IESKF::CustomObsFunc obs) {
    // The H matrix is provided by the user

    SO3 start_R = R_;
    Eigen::Matrix<S, 18, 1> HTVr;
    Eigen::Matrix<S, 18, 18> HTVH;
    Eigen::Matrix<S, 18, Eigen::Dynamic> K;
    Mat18T Pk, Qk;

    for (int iter = 0; iter < options_.num_iterations_; ++iter) {
        // call the obs function
        obs(GetNominalSE3(), HTVH, HTVr);

        // Projection of P to the tangent space at the current nominal state, which is necessary 
        // for the Kalman update, otherwise the update may be incorrect because the error state 
        // is defined in the tangent space of the manifold, and the H matrix is also defined in 
        // the tangent space, so we need to project P to the tangent space before the update. 
        //The projection is done by a Jacobian matrix J, which is derived from the exponential map of SO3,
        // and it can be shown that J = I - 0.5 * hat(dtheta), where dtheta is the rotation difference 
        // between the current nominal state and the start nominal state (the nominal state before iterations), 
        // and hat() is the hat operator of SO3.
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat((R_.inverse() * start_R).log());
        Pk = J * cov_ * J.transpose();

        // Kalman update
        Qk = (Pk.inverse() + HTVH).inverse(); // this is an intermediate variable, can be used for final update
        dx_ = Qk * HTVr;
        // LOG(INFO) << "iter " << iter << " dx = " << dx_.transpose() << ", dxn: " << dx_.norm();

        // dx update
        Update();

        if (dx_.norm() < options_.quit_eps_) {
            break;
        }
    }

    // update P
    cov_ = (Mat18T::Identity() - Qk * HTVH) * Pk;

    // project P
    Mat18T J = Mat18T::Identity();
    Vec3d dtheta = (R_.inverse() * start_R).log();
    J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dtheta);
    cov_ = J * cov_ * J.inverse();

    dx_.setZero();
    return true;
}

template <typename S>
bool IESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);

    // --- 1. Predicted body-frame velocity at the wheel frame (with lever arm) ---
    const Mat3T Rt = R_.matrix().transpose();    // Rᵀ
    VecT v_body = Rt * v_;                        // Rᵀ·v   (IMU velocity in body frame)
    VecT r_iw   = options_.r_imu_wheel_.template cast<S>();
    VecT omega  = last_gyro_ - bg_;              // ω = gyro − bg   (needs last_gyro_)
    if (options_.use_wheel_lever_arm_) {
        v_body += omega.cross(r_iw);             // + ω × r_iw
    }

    // --- 2. Measurement z + residual r (4×1: v_fwd (x), NHC 0(y), NHC 0(z), yaw) ---
    Eigen::Matrix<S, 4, 1> z, r;
    z << static_cast<S>(odom.v_fwd_), S(0), S(0), static_cast<S>(odom.yaw_rate_);
    r.template head<3>() = z.template head<3>() - v_body;
    r(3) = z(3) - omega.z();                      // ω_z = (gyro − bg)_z

    // --- 3. Jacobian H (4×18) ---
    Eigen::Matrix<S, 4, 18> H = Eigen::Matrix<S, 4, 18>::Zero();
    H.template block<3, 3>(0, 3) = Rt;                       // ∂/∂δv
    H.template block<3, 3>(0, 6) = SO3::hat(v_body);         // ∂/∂δθ  (≈ hat(Rᵀv))
    if (options_.use_wheel_lever_arm_) {
        H.template block<3, 3>(0, 9) = SO3::hat(r_iw);       // ∂/∂δbg (lever arm)
    }
    H(3, 11) = S(-1);                                        // ∂ω_z/∂δbg_z

    // Disable the yaw row cleanly (keeps the update fixed-size 4×18):
    if (!options_.use_wheel_yaw_rate_) { H.row(3).setZero(); r(3) = S(0); }

    // --- 4. Gain-form Kalman update (4×4 inverse) ---
    Eigen::Matrix<S, 4, 4> Skk = H * cov_ * H.transpose() + odom_noise_;   // innovation cov
    const Eigen::Matrix<S, 4, 4> Skk_inv = Skk.inverse();

    // Slip gate: when the residual is wildly inconsistent with the filter
    // (lifted robot / wheel slip), the measurement is a fault, not noise —
    // reject it instead of averaging it in.
    last_wheel_chi2_ = (r.transpose() * Skk_inv * r)(0, 0);
    if (options_.odom_chi2_thresh_ > S(0) && last_wheel_chi2_ > options_.odom_chi2_thresh_) {
        ++wheel_gate_rejects_;
        return false;
    }

    Eigen::Matrix<S, 18, 4> K = cov_ * H.transpose() * Skk_inv;

    dx_ = K * r;          // error-state correction
    Update();             // inject dx_ into the nominal state (private helper)

    // Joseph-free covariance update (single-step, fine for this near-linear obs)
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    dx_.setZero();
    return true;
}


// Explicit template instantiation
template class IESKF<double>;

} // namespace IncLIO
