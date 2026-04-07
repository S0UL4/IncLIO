#ifndef INCLIO_STATE_HPP
#define INCLIO_STATE_HPP

#include "utils/eigen_types.hpp"
#include <sophus/so3.hpp>

namespace IncLIO {

// TODO: Define the full LIO state used by the IESKF
//
// Nominal state (on manifold):
//   - position     (Vec3d)
//   - velocity     (Vec3d)
//   - rotation     (SO3)
//   - accel_bias   (Vec3d)
//   - gyro_bias    (Vec3d)
//   - gravity      (Vec3d)
//
// Error state (tangent space, dim = 18):
//   - dp (3), dv (3), dtheta (3), dba (3), dbg (3), dg (3)

template <typename T>
struct State {
    // TODO: Define state members
    // TODO: SE3 GetPose() const;
    // TODO: void Reset();
    // TODO: static constexpr int ErrorDim = 18;

    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using SO3 = Sophus::SO3<T>;

    State() = default;

    // from time, R, p, v, bg, ba
    explicit State(double time, const SO3& R = SO3(), const Vec3& t = Vec3::Zero(), const Vec3& v = Vec3::Zero(),
                      const Vec3& bg = Vec3::Zero(), const Vec3& ba = Vec3::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {}

    // from pose and vel
    State(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

    Sophus::SE3<T> GetSE3() const { return SE3(R_, p_); }

    friend std::ostream& operator<<(std::ostream& os, const State<T>& s) {
        os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose();
        return os;
    }

    double timestamp_ = 0.0;
    SO3 R_;
    Vec3 p_= Vec3::Zero();
    Vec3 v_= Vec3::Zero();
    Vec3 ba_= Vec3::Zero();
    Vec3 bg_= Vec3::Zero();
};

using Stated = State<double>;
using Statef = State<float>;

} // namespace IncLIO


#endif // INCLIO_STATE_HPP
