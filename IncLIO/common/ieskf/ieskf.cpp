#include "ieskf/ieskf.hpp"

namespace IncLIO {

// TODO: Implement IESKF methods
//
// Predict():
//   - Propagate nominal state using IMU kinematics
//   - Propagate error-state covariance: P = F * P * F^T + Q
//   - F is the error-state transition matrix
//
// Update():
//   - For iter = 0..max_iterations:
//       1. Compute observation Jacobian H and residual z at current state
//       2. Kalman gain: K = P * H^T * (H * P * H^T + R)^{-1}
//       3. Error-state correction: dx = K * z
//       4. Inject dx into nominal state (manifold retraction)
//       5. Check convergence: if ||dx|| < threshold, break
//   - Update covariance: P = (I - K*H) * P
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



// Explicit template instantiation
template class IESKF<double>;

} // namespace IncLIO
