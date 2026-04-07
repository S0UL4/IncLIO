#include "ndt/ndt_voxel.hpp"

namespace IncLIO {

void NdtVoxel::UpdateDistribution(bool is_first_scan, int min_pts_in_voxel, int max_pts_in_voxel) {
    if (is_first_scan) {
        if (pts_.size() > 1) {
            math::ComputeMeanAndCov(pts_, mu_, sigma_, [](const Vec3d& p) { return p; });
            info_ = (sigma_ + Mat3d::Identity() * 1e-3).inverse();  // Regularize to avoid NaN
        } else {
            mu_ = pts_[0];
            info_ = Mat3d::Identity() * 1e2;
        }

        ndt_estimated_ = true;
        pts_.clear();
        return;
    }

    // Voxel already saturated — discard new points
    if (ndt_estimated_ && num_pts_ > max_pts_in_voxel) {
        pts_.clear();
        return;
    }

    if (!ndt_estimated_ && static_cast<int>(pts_.size()) > min_pts_in_voxel) {
        // New voxel with enough points — estimate from scratch
        math::ComputeMeanAndCov(pts_, mu_, sigma_, [](const Vec3d& p) { return p; });
        info_ = (sigma_ + Mat3d::Identity() * 1e-3).inverse();
        ndt_estimated_ = true;
        pts_.clear();
    } else if (ndt_estimated_ && static_cast<int>(pts_.size()) > min_pts_in_voxel) {
        // Already estimated — incrementally merge new points
        Vec3d cur_mu, new_mu;
        Mat3d cur_var, new_var;
        math::ComputeMeanAndCov(pts_, cur_mu, cur_var, [](const Vec3d& p) { return p; });
        math::UpdateMeanAndCov(num_pts_, static_cast<int>(pts_.size()), mu_, sigma_, cur_mu, cur_var, new_mu, new_var);

        mu_ = new_mu;
        sigma_ = new_var;
        num_pts_ += pts_.size();
        pts_.clear();

        // Regularize information matrix via SVD
        // Eigen::JacobiSVD svd(sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // Vec3d lambda = svd.singularValues();
        // if (lambda[1] < lambda[0] * 1e-3) {
        //     lambda[1] = lambda[0] * 1e-3;
        // }
        // if (lambda[2] < lambda[0] * 1e-3) {
        //     lambda[2] = lambda[0] * 1e-3;
        // }

        // Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        // info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        // Regularize information matrix via SVD
        // Eigen::SelfAdjointEigenSolver<Mat3d> eig(sigma_);
        // Vec3d lambda = eig.eigenvalues().cwiseMax(eig.eigenvalues()[2] * 1e-3);
        // info_ = eig.eigenvectors() * lambda.cwiseInverse().asDiagonal() * eig.eigenvectors().transpose();        
        Eigen::JacobiSVD svd(sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3d lambda_ = svd.singularValues();
        if (lambda_[1] < lambda_[0] * 1e-3) {
            lambda_[1] = lambda_[0] * 1e-3;
        }
        if (lambda_[2] < lambda_[0] * 1e-3) {
            lambda_[2] = lambda_[0] * 1e-3;
        }

        Mat3d inv_lambda = Vec3d(1.0 / lambda_[0], 1.0 / lambda_[1], 1.0 / lambda_[2]).asDiagonal();
        info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();


    }
}

} // namespace IncLIO
