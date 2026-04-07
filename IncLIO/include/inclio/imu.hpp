#ifndef INCLIO_IMU_HPP
#define INCLIO_IMU_HPP

#include "utils/eigen_types.hpp"
#include <memory>

namespace IncLIO {

// TODO: Define IMU measurement struct
struct IMUData {
    IMUData() = default;
    IMUData(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}
    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};

using IMU = IMUData;

} // namespace IncLIO

using IMUPtr = std::shared_ptr<IncLIO::IMUData>;


#endif // INCLIO_IMU_HPP 
