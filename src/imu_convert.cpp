#include "ros2_wrapper/imu_convert.hpp"

namespace inclio_ros2 {

void IMUConverter::Process(const sensor_msgs::msg::Imu & msg, IMUPtr &imu_out) {
    double imu_coeff = imu_coeff_;
    double ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

    // orientation (quaternion) — skip
    // auto& q = msg.orientation;
    // double qw = q.w;
    // double qx = q.x;
    // double qy = q.y;
    // double qz = q.z;

    // angular_velocity
    auto& av = msg.angular_velocity;
    double gx = av.x;
    double gy = av.y;
    double gz = av.z;

    // linear_acceleration
    auto& la = msg.linear_acceleration;
    double ax = la.x;
    double ay = la.y;
    double az = la.z;

    imu_out = std::make_shared<IncLIO::IMUData>(ts, Vec3d(gx, gy, gz), Vec3d(ax * imu_coeff, ay * imu_coeff, az * imu_coeff));
}

}
