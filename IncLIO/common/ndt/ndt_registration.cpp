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

    const auto& nearby_grids = ndt_map_->GetNearbyGrids();
    int num_residual_per_point = static_cast<int>(nearby_grids.size());
    double inv_voxel_size = ndt_map_->GetOptions().inv_voxel_size;

    std::vector<int> index(source_->points.size());
    std::iota(index.begin(), index.end(), 0);

    int total_size = static_cast<int>(index.size()) * num_residual_per_point;

    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 18>> jacobians(total_size);
    std::vector<Vec3d> errors(total_size);
    std::vector<Mat3d> infos(total_size);

    // Compute residuals and Jacobians for IESKF (parallel)
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q;

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

                // IESKF state: [p(3), v(3), R(3), ba(3), bg(3), g(3)]
                // Jacobian: dz/dp = I, dz/dR = -R * hat(q)
                Eigen::Matrix<double, 3, 18> J;
                J.setZero();
                J.block<3, 3>(0, 0) = Mat3d::Identity();                   // w.r.t. position
                J.block<3, 3>(0, 6) = -pose.so3().matrix() * SO3::hat(q);  // w.r.t. rotation

                jacobians[real_idx] = J;
                errors[real_idx] = e;
                infos[real_idx] = v->GetInfoMatrix();
                effect_pts[real_idx] = true;
            } else {
                effect_pts[real_idx] = false;
            }
        }
    });

    // Accumulate (parallel reduction over points)
    const double info_ratio = 0.02;  // scale down the information to avoid overconfidence in IESKF update

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
            for (int i = 0; i < num_residual_per_point; ++i) {
                int ri = idx * num_residual_per_point + i;
                if (!effect_pts[ri]) continue;
                a.effective_num++;
                a.HTVH += jacobians[ri].transpose() * infos[ri] * jacobians[ri] * info_ratio;
                a.HTVr += -jacobians[ri].transpose() * infos[ri] * errors[ri] * info_ratio;
            }
            return a;
        });

    HTVH = acc.HTVH;
    HTVr = acc.HTVr;

    INCLIO_DEBUG("NDT observation effective: {}", acc.effective_num);
}

} // namespace IncLIO
