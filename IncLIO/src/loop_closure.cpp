#include "inclio/loop_closure.hpp"

#include "ndt/ndt_voxel.hpp"

#include <Eigen/Eigenvalues>
#include <unordered_map>

namespace IncLIO {

void BuildKeyframeVoxels(const std::vector<CloudPtr>& scans,
                         const KeyframeVoxelizerConfig& cfg,
                         std::vector<NdtSnapshot>& out) {
    out.clear();
    if (cfg.voxel_size <= 0.0) return;

    const double inv = 1.0 / cfg.voxel_size;
    const double r2  = cfg.max_radius * cfg.max_radius;

    std::unordered_map<Vec3i, NdtVoxel, hash_vec<3>> voxels;
    voxels.reserve(8192);

    for (const auto& scan : scans) {
        if (!scan) continue;
        for (const auto& p : scan->points) {
            const Vec3d xyz(p.x, p.y, p.z);
            if (xyz.x() * xyz.x() + xyz.y() * xyz.y() > r2) continue;
            const double z_shifted = xyz.z() + cfg.lidar_height;
            if (z_shifted < 0.0 || z_shifted > cfg.max_z) continue;

            const Vec3i key = CastToInt(xyz * inv);
            auto it = voxels.find(key);
            if (it == voxels.end()) voxels.emplace(key, NdtVoxel(xyz));
            else                    it->second.AddPoint(xyz);
        }
    }

    Eigen::SelfAdjointEigenSolver<Mat3d> eig;
    out.reserve(voxels.size());
    for (auto& kv : voxels) {
        NdtVoxel& v = kv.second;
        if (v.num_pts_ < cfg.min_pts_in_voxel) continue;

        v.UpdateDistribution(/*is_first_scan=*/true,
                             cfg.min_pts_in_voxel,
                             cfg.max_pts_in_voxel);
        if (!v.IsValid()) continue;

        eig.compute(v.GetCovariance());
        if (eig.info() != Eigen::Success) continue;

        out.emplace_back();
        NdtSnapshot& s = out.back();
        s.mean      = v.GetMean();
        s.mean.z() += cfg.lidar_height;       // NDTMC convention
        s.cov       = v.GetCovariance();
        s.evals     = eig.eigenvalues();
        s.n_pts     = v.num_pts_;
    }
}

}  // namespace IncLIO
