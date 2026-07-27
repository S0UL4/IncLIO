#ifndef INCLIO_ODOM_HPP
#define INCLIO_ODOM_HPP

namespace IncLIO {

/// Wheel-odometry measurement (from nav_msgs/Odometry twist, in the base_link frame).
struct Odom {
    Odom() = default;
    Odom(double t, double v_fwd, double yaw_rate)
        : timestamp_(t), v_fwd_(v_fwd), yaw_rate_(yaw_rate) {}

    double timestamp_ = 0.0;
    double v_fwd_     = 0.0;   // forward speed [m/s],  from twist.twist.linear.x
    double yaw_rate_  = 0.0;   // yaw rate [rad/s],     from twist.twist.angular.z
};

}  // namespace IncLIO
#endif  // INCLIO_ODOM_HPP
