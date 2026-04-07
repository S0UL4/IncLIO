#ifndef INCLIO_NDT_REGISTRATION_HPP
#define INCLIO_NDT_REGISTRATION_HPP

#include "utils/eigen_types.hpp"
#include "utils/point_types.hpp"
#include "ndt/ndt_map.hpp"

namespace IncLIO {

/// NDT scan-to-map registration.
///
/// Provides two modes:
///   1. AlignNdt()  — standalone Gauss-Newton alignment (for testing / offline use)
///   2. ComputeResidualAndJacobians()  — computes H^T V H and H^T V r for the
///      IESKF update step (the IESKF drives the iteration, not this class)
class NdtRegistration {
   public:
    struct Options {
        int max_iteration = 4;        // Max Gauss-Newton iterations (for AlignNdt)
        int min_effective_pts = 10;   // Min effective point matches
        double eps = 1e-3;            // Convergence threshold on dx norm
        double res_outlier_th = 5.0;  // Chi-squared outlier rejection threshold
    };

    NdtRegistration() = default;
    explicit NdtRegistration(const Options& options) : options_(options) {}

    /// Set the source scan to align
    void SetSource(CloudPtr source) { source_ = source; }

    /// Set the NDT map to align against
    void SetNdtMap(NdtMap* ndt_map) { ndt_map_ = ndt_map; }

    /// Standalone Gauss-Newton NDT alignment.
    /// @param init_pose  initial guess (updated in-place on success)
    /// @return true if converged with enough effective points
    bool AlignNdt(SE3& init_pose);

    /// Compute Jacobians and residuals for IESKF update (18-dim state).
    /// The IESKF state is [p(3), v(3), R(3), ba(3), bg(3), g(3)].
    /// Jacobian layout: col 0-2 = position, col 6-8 = rotation.
    /// @param pose    current pose estimate from IESKF
    /// @param HTVH    output: accumulated H^T * V * H  (18x18)
    /// @param HTVr    output: accumulated H^T * V * r  (18x1)
    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr);

   private:
    Options options_;
    CloudPtr source_ = nullptr;
    NdtMap* ndt_map_ = nullptr;
};

} // namespace IncLIO

#endif // INCLIO_NDT_REGISTRATION_HPP
