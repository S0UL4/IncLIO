#ifndef INCLIO_POINT_TYPES_HPP
#define INCLIO_POINT_TYPES_HPP

#include "utils/eigen_types.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include <vector>
#include <cstdint>
#include <omp.h>

namespace IncLIO {

/// Fast rigid transform of a PointXYZI cloud using OMP.
/// Drop-in replacement for pcl::transformPointCloud.
inline void transformCloudOMP(const pcl::PointCloud<pcl::PointXYZI>& input,
                              pcl::PointCloud<pcl::PointXYZI>& output,
                              const Eigen::Matrix4f& T) {
    output.resize(input.size());
    output.width  = input.width;
    output.height = input.height;
    output.is_dense = input.is_dense;

    const float r00 = T(0,0), r01 = T(0,1), r02 = T(0,2), tx = T(0,3);
    const float r10 = T(1,0), r11 = T(1,1), r12 = T(1,2), ty = T(1,3);
    const float r20 = T(2,0), r21 = T(2,1), r22 = T(2,2), tz = T(2,3);

    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < input.size(); ++i) {
        const auto& p = input[i];
        auto& q = output[i];
        q.x = r00 * p.x + r01 * p.y + r02 * p.z + tx;
        q.y = r10 * p.x + r11 * p.y + r12 * p.z + ty;
        q.z = r20 * p.x + r21 * p.y + r22 * p.z + tz;
        q.intensity = p.intensity;
    }
}

// TODO: Define custom point types used throughout the pipeline
//       e.g. PointXYZI, PointXYZINormal, FullPointType with ring/time fields

// Define the point and point cloud types used in the system
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;

// Common conversion functions from PCL point types to Eigen vectors
inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

// ToEigen : Convert PCL point to Eigen vector, with template specialization for different dimensions and types
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt);

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2>(const PointType& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3>(const PointType& pt) {
    return Vec3f(pt.x, pt.y, pt.z);
}

template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointType p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    return p;
}

// Full point type with additional fields for range, ring, time, etc. Used for more detailed processing and visualization
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    float intensity = 0;
    uint16_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;
    
    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Register the custom point type with PCL
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

inline Vec3f ToVec3f(const FullPointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const FullPointType& pt) { return pt.getVector3fMap().cast<double>(); }

// UI point type with color information, used for visualization in the UI
using UiPointType = pcl::PointXYZRGBA;
using UiPointCloudType = pcl::PointCloud<UiPointType>;
using UiCloudPtr = UiPointCloudType::Ptr;

} // namespace IncLIO

POINT_CLOUD_REGISTER_POINT_STRUCT(IncLIO::FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)(
                                      std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(
                                      std::uint8_t, ring, ring)(double, time, time)(float, height, height))


namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointNoIntensity {
    PCL_ADD_POINT4D;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::PointNoIntensity,
                                  (float, x, x)(float, y, y)(float, z, z)
                                      (float, time, time)(std::uint16_t, ring, ring))


namespace hesai_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (std::uint16_t, ring, ring)(double, timestamp, timestamp))



namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros                                      

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range))


#endif // INCLIO_POINT_TYPES_HPP
