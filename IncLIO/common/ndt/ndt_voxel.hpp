#ifndef INCLIO_NDT_VOXEL_HPP
#define INCLIO_NDT_VOXEL_HPP

#include "utils/eigen_types.hpp"
#include "utils/math_utils.hpp"

#include <vector>

namespace IncLIO {

/// A single NDT voxel that accumulates points and maintains
/// a Gaussian distribution (mean + covariance).
/// Supports incremental updates: new points can be merged with
/// historical statistics without recomputing from scratch.
struct NdtVoxel {
    NdtVoxel() = default;
    explicit NdtVoxel(const Vec3d& pt) {
        pts_.emplace_back(pt);
        num_pts_ = 1;
    }

    void AddPoint(const Vec3d& pt) {
        pts_.emplace_back(pt);
        if (!ndt_estimated_) {
            num_pts_++;
        }
    }

    /// Finalize / update the Gaussian distribution from buffered points.
    /// @param is_first_scan  if true, estimate from scratch (no prior)
    /// @param min_pts_in_voxel  minimum points before estimating
    /// @param max_pts_in_voxel  stop accepting points after this count
    void UpdateDistribution(bool is_first_scan, int min_pts_in_voxel, int max_pts_in_voxel);

    bool IsValid() const { return ndt_estimated_; }
    const Vec3d& GetMean() const { return mu_; }
    const Mat3d& GetCovariance() const { return sigma_; }
    const Mat3d& GetInfoMatrix() const { return info_; }

    std::vector<Vec3d> pts_;       // Buffered points (cleared after estimation)
    Vec3d mu_ = Vec3d::Zero();     // Mean
    Mat3d sigma_ = Mat3d::Zero();  // Covariance
    Mat3d info_ = Mat3d::Zero();   // Information matrix (inverse covariance)

    bool ndt_estimated_ = false;   // Whether the distribution has been estimated
    int num_pts_ = 0;              // Total accumulated point count (for incremental update)
};

/// Cast a floating-point 3D vector to integer (floor)
inline Vec3i CastToInt(const Vec3d& v) {
    return Vec3i(static_cast<int>(std::floor(v[0])),
                 static_cast<int>(std::floor(v[1])),
                 static_cast<int>(std::floor(v[2])));
}

} // namespace IncLIO

#endif // INCLIO_NDT_VOXEL_HPP
