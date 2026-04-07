#include "ndt/ndt_registration.hpp"
#include "utils/logger.hpp"

#include <execution>
#include <numeric>

namespace IncLIO {

bool NdtRegistration::AlignNdt(SE3& init_pose) {
    assert(ndt_map_ != nullptr && ndt_map_->NumVoxels() > 0);
    assert(source_ != nullptr);

    INCLIO_INFO("aligning with inc ndt, pts: {}, grids: {}", source_->size(), ndt_map_->NumVoxels());

    SE3 pose = init_pose;

    const auto& nearby_grids = ndt_map_->GetNearbyGrids();
    int num_residual_per_point = static_cast<int>(nearby_grids.size());
    double inv_voxel_size = ndt_map_->GetOptions().inv_voxel_size;

    std::vector<int> index(source_->points.size());
    std::iota(index.begin(), index.end(), 0);

    int total_size = static_cast<int>(index.size()) * num_residual_per_point;

    for (int iter = 0; iter < options_.max_iteration; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);

        // Compute residuals and Jacobians (parallel over points)
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // Transformed point

            Vec3i key = CastToInt(Vec3d(qs * inv_voxel_size));

            for (int i = 0; i < num_residual_per_point; ++i) {
                Vec3i real_key = key + nearby_grids[i];
                int real_idx = idx * num_residual_per_point + i;

                auto* v = ndt_map_->GetVoxel(real_key);
                if (v && v->IsValid()) {
                    Vec3d e = qs - v->GetMean();

                    double res = e.transpose() * v->GetInfoMatrix() * e;
                    if (std::isnan(res) || res > options_.res_outlier_th) {
                        effect_pts[real_idx] = false;
                        continue;
                    }

                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = Mat3d::Identity();

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v->GetInfoMatrix();
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
        });

        // Accumulate Hessian and gradient (parallel reduction over points)
        struct Accum {
            Mat6d H = Mat6d::Zero();
            Vec6d err = Vec6d::Zero();
            double total_res = 0;
            int effective_num = 0;
        };

        auto acc = std::transform_reduce(
            std::execution::par_unseq, index.begin(), index.end(),
            Accum{},
            [](const Accum& a, const Accum& b) -> Accum {
                return {a.H + b.H, a.err + b.err,
                        a.total_res + b.total_res,
                        a.effective_num + b.effective_num};
            },
            [&](int idx) -> Accum {
                Accum a;
                for (int i = 0; i < num_residual_per_point; ++i) {
                    int ri = idx * num_residual_per_point + i;
                    if (!effect_pts[ri]) continue;
                    a.total_res += errors[ri].transpose() * infos[ri] * errors[ri];
                    a.effective_num++;
                    a.H += jacobians[ri].transpose() * infos[ri] * jacobians[ri];
                    a.err += -jacobians[ri].transpose() * infos[ri] * errors[ri];
                }
                return a;
            });

        double total_res = acc.total_res;
        int effective_num = acc.effective_num;
        Mat6d H = acc.H;
        Vec6d err = acc.err;

        if (effective_num < options_.min_effective_pts) {
            INCLIO_WARN("effective num too small: {}", effective_num);
            init_pose = pose;
            return false;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        INCLIO_INFO("iter {} total res: {:.4f}, eff: {}, mean res: {:.4f}, dx norm: {:.6f}",
                    iter, total_res, effective_num, total_res / effective_num, dx.norm());

        if (dx.norm() < options_.eps) {
            INCLIO_INFO("converged, dx norm = {:.6f}", dx.norm());
            break;
        }
    }

    init_pose = pose;
    return true;
}

void NdtRegistration::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
    assert(ndt_map_ != nullptr && ndt_map_->NumVoxels() > 0);
    assert(source_ != nullptr);

    SE3 pose = input_pose;
    Mat3d R = pose.so3().matrix();

    const auto& nearby_grids = ndt_map_->GetNearbyGrids();
    int num_residual_per_point = static_cast<int>(nearby_grids.size());
    double inv_voxel_size = ndt_map_->GetOptions().inv_voxel_size;
    double res_outlier_th = options_.res_outlier_th;

    std::vector<int> index(source_->points.size());
    std::iota(index.begin(), index.end(), 0);

    // Fused compute + accumulate in a single parallel pass.
    // Exploits Jacobian sparsity: J is 3x18 but only cols 0:2 (position) and
    // 6:8 (rotation) are nonzero, so J^T*V*J has only 4 nonzero 3x3 blocks
    // and J^T*V*e has only 2 nonzero 3x1 blocks. This avoids:
    //   - ~37 MB of intermediate heap allocations (jacobians/errors/infos/flags)
    //   - Dense 18x3 * 3x3 * 3x18 multiplies (324 ops -> 36 ops per residual)
    const double info_ratio = 0.02;

    struct Accum18 {
        Mat18d HTVH = Mat18d::Zero();
        Vec18d HTVr = Vec18d::Zero();
        int effective_num = 0;
    };

    auto acc = std::transform_reduce(
        std::execution::par_unseq, index.begin(), index.end(),
        Accum18{},
        [](const Accum18& a, const Accum18& b) -> Accum18 {
            return {a.HTVH + b.HTVH, a.HTVr + b.HTVr,
                    a.effective_num + b.effective_num};
        },
        [&](int idx) -> Accum18 {
            Accum18 a;
            Vec3d q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;
            Vec3i key = CastToInt(Vec3d(qs * inv_voxel_size));

            // Pre-compute once per point (shared across all 7 neighbors)
            // M = -R * hat(q)  — the rotation Jacobian block
            Mat3d M = -R * SO3::hat(q);

            for (int i = 0; i < num_residual_per_point; ++i) {
                auto* v = ndt_map_->GetVoxel(key + nearby_grids[i]);
                if (!v || !v->IsValid()) continue;

                Vec3d e = qs - v->GetMean();
                const Mat3d& info = v->GetInfoMatrix();
                double res = e.transpose() * info * e;
                if (std::isnan(res) || res > res_outlier_th) continue;

                // Scaled info and info*error — used by all block accumulations
                Mat3d V = info * info_ratio;      // 3x3
                Vec3d Ve = V * e;                  // 3x1

                // Exploit J sparsity: J = [I₃ | 0 | M | 0 | 0 | 0]
                // HTVH += J^T * V * J  — only 4 nonzero 3x3 blocks:
                //   (0,0): I^T * V * I = V
                //   (0,6): I^T * V * M = V * M
                //   (6,0): M^T * V * I = M^T * V        (symmetric to (0,6))
                //   (6,6): M^T * V * M
                Mat3d VM = V * M;

                a.HTVH.block<3, 3>(0, 0) += V;
                a.HTVH.block<3, 3>(0, 6) += VM;
                a.HTVH.block<3, 3>(6, 0) += VM.transpose();
                a.HTVH.block<3, 3>(6, 6) += M.transpose() * VM;

                // HTVr += -J^T * V * e  — only 2 nonzero 3x1 blocks:
                //   (0): -I^T * V * e = -Ve
                //   (6): -M^T * V * e
                a.HTVr.segment<3>(0) += -Ve;
                a.HTVr.segment<3>(6) += -M.transpose() * Ve;

                a.effective_num++;
            }
            return a;
        });

    HTVH = acc.HTVH;
    HTVr = acc.HTVr;

    INCLIO_DEBUG("NDT observation effective: {}", acc.effective_num);
}

} // namespace IncLIO
