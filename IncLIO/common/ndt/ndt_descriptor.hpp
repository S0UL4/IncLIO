#ifndef INCLIO_NDT_DESCRIPTOR_HPP
#define INCLIO_NDT_DESCRIPTOR_HPP

#include "ndt/ndt_voxel.hpp"
#include "utils/eigen_types.hpp"

#include <Eigen/Core>
#include <utility>
#include <vector>

namespace IncLIO {

/// One NDT voxel snapshotted into the keyframe's local LiDAR frame.
/// Z is shifted by LIDAR_HEIGHT so ground sits at z=0 (NDTMC convention).
/// Eigenvalues are sorted ascending and precomputed at snapshot time
/// (rotation-invariant — no need to recompute after frame transform).
struct NdtSnapshot {
    Vec3d mean  = Vec3d::Zero();
    Mat3d cov   = Mat3d::Zero();
    Vec3d evals = Vec3d::Zero();  // ascending: evals(0) <= evals(1) <= evals(2)
    int   n_pts = 0;
};

struct NdtDescriptorConfig {
    int    pc_num_ring   = 20;
    int    pc_num_sector = 60;
    int    pc_num_z      = 6;
    double pc_max_radius = 80.0;  // m, max planar range from LiDAR origin
    double pc_max_z      = 6.0;   // m, max height above ground
};

/// NDT-Map-Code descriptor.
/// Builds a 2*R x S real-valued matrix from pre-snapshotted voxels:
///   - top R rows: per-cell Gaussian-entropy weighted by layer
///   - bottom R rows: per-cell shape-histogram dominant mode
/// Plus a 1x10 shape histogram used as a coarse pre-filter key.
/// Pure math: no voxelization, no PCL, no I/O.
class NdtDescriptor {
public:
    NdtDescriptor() = default;
    explicit NdtDescriptor(const NdtDescriptorConfig& cfg) : cfg_(cfg) {}

    /// Build descriptor + histogram from snapshot voxels in LiDAR frame.
    /// desc_out is (2*pc_num_ring, pc_num_sector); hist_out is length 10.
    void Build(const std::vector<NdtSnapshot>& voxels,
               Eigen::MatrixXd& desc_out,
               Eigen::VectorXd& hist_out) const;

    /// NDTMC-LIO-SAM-style similarity: SC sectorkey fast-align + bounded
    /// shift search + mean-centered correlation (distDirectNDTMC) inner metric.
    /// Returns (best_distance, best_sector_shift). Lower distance = closer match.
    static std::pair<double, int> Distance(const Eigen::MatrixXd& a,
                                           const Eigen::MatrixXd& b);

    const NdtDescriptorConfig& Config() const { return cfg_; }

private:
    NdtDescriptorConfig cfg_;
};

}  // namespace IncLIO

#endif  // INCLIO_NDT_DESCRIPTOR_HPP
