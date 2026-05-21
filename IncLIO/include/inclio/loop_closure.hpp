#ifndef INCLIO_LOOP_CLOSURE_HPP
#define INCLIO_LOOP_CLOSURE_HPP

#include "ndt/ndt_descriptor.hpp"
#include "utils/eigen_types.hpp"
#include "utils/point_types.hpp"

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace IncLIO {

/// One keyframe held by the loop-closure subsystem.
/// Fields are filled in two stages:
///   - LIDAR thread:  id, timestamp, T_W_L, raw_scan_lidar
///   - LOOP thread:   map_voxels_local, descriptor, histogram
struct Keyframe {
    int      id        = -1;
    double   timestamp = 0.0;
    SE3      T_W_L;
    CloudPtr raw_scan_lidar;            // LiDAR-frame undistorted scan
    std::vector<NdtSnapshot> map_voxels_local;
    Eigen::MatrixXd descriptor;         // 2*R x S
    Eigen::VectorXd histogram;          // length 10
};
using KeyframePtr = std::shared_ptr<Keyframe>;

/// Knobs for fresh per-keyframe voxelization. Defaults match NdtMap so the
/// descriptor sees voxel statistics in the same regime that NDTMC-LIO-SAM
/// was tuned against.
struct KeyframeVoxelizerConfig {
    double voxel_size       = 0.5;
    int    min_pts_in_voxel = 5;
    int    max_pts_in_voxel = 50;
    double max_radius       = 80.0;  // m, planar cutoff
    double lidar_height     = 0.0;   // m, added to z so ground sits at z=0
    double max_z            = 6.0;   // m, vertical cutoff after height shift
};

/// Voxelize one or more LiDAR-frame scans FRESH for the descriptor.
/// No NdtMap dependency: forward and reverse traversals see identical
/// inputs (same N neighbor scans transformed into the same target frame),
/// so descriptor distance reflects geometry rather than map history.
///
/// `scans` must already be in the *target* keyframe's LiDAR frame.
/// The caller transforms neighbor scans by T_targetLidar^-1 * T_W_neighbor
/// before passing them in.
void BuildKeyframeVoxels(const std::vector<CloudPtr>& scans_in_target_lidar_frame,
                         const KeyframeVoxelizerConfig& cfg,
                         std::vector<NdtSnapshot>& out);

}  // namespace IncLIO

#endif  // INCLIO_LOOP_CLOSURE_HPP
