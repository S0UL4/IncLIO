#pragma once

#include <common/utils/point_types.hpp>  // FullPointCloudType, FullPointType, FullCloudPtr
#include <inclio/imu.hpp>  // IMU msg
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <cstdint>
#include <cstring>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <execution>


namespace inclio_ros2 {
// ─────────────────────────────────────────────────────────────────────────────
// IMUConverter
// ─────────────────────────────────────────────────────────────────────────────
class IMUConverter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit IMUConverter(double imu_coeff = 1.0) : imu_coeff_(imu_coeff) {}
    void SetIMUCoeff(double coeff) { imu_coeff_ = coeff; }

    void Process(const sensor_msgs::msg::Imu & msg, IMUPtr &imu_out);

private:
    double imu_coeff_ = 1.0;  // g coeff for livox imu (default 1.0, set to 9.81 for raw accel in m/s²)
};

} // namespace inclio_ros2
