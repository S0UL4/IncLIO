#include "inclio/ct_undistort.hpp"

namespace IncLIO {

std::vector<CTSegment> BuildCTSegments(
    const std::vector<Stated>& states,
    const std::vector<Vec3d>&  a_world,
    const std::vector<Vec3d>&  w_body)
{
    const size_t M = a_world.size();
    std::vector<CTSegment> segs;
    segs.reserve(M);

    for (size_t k = 0; k < M; ++k) {
        CTSegment seg;
        seg.t0 = states[k].timestamp_;
        seg.p0 = states[k].p_;
        seg.v0 = states[k].v_;
        seg.R0 = states[k].R_;
        seg.a0 = a_world[k];
        seg.w0 = w_body[k];

        // Jerk and angular acceleration as finite differences toward the next sample.
        // Last segment gets j=0, alpha=0 (constant-acceleration fallback).
        if (k + 1 < M) {
            double dt = states[k + 1].timestamp_ - states[k].timestamp_;
            if (dt > 1e-9) {
                seg.j     = (a_world[k + 1] - a_world[k]) / dt;
                seg.alpha = (w_body[k + 1]  - w_body[k])  / dt;
            }
        }

        segs.push_back(seg);
    }
    return segs;
}

SE3 CTQueryPose(double t_pt, const std::vector<CTSegment>& segs)
{
    if (segs.empty()) return SE3();

    // Find the last segment whose t0 <= t_pt (segments are ordered by time).
    const CTSegment* seg = &segs.front();
    for (size_t i = 1; i < segs.size(); ++i) {
        if (segs[i].t0 <= t_pt)
            seg = &segs[i];
        else
            break;
    }

    double t = t_pt - seg->t0;
    if (t < 0.0) t = 0.0;  // clamp for points before the first IMU sample

    // DLIO eq. (5) — position (cubic)
    Vec3d p_star = seg->p0
                 + seg->v0 * t
                 + 0.5       * seg->a0 * t * t
                 + (1.0/6.0) * seg->j  * t * t * t;

    // DLIO eq. (5) — orientation (quadratic in quaternion coefficients)
    // q*(t) = q0 + 0.5*(q0 ⊗ ω0)*t + 0.25*(q0 ⊗ α)*t²   then normalize
    Eigen::Quaterniond q0      = seg->R0.unit_quaternion();
    Eigen::Quaterniond omega_q(0.0, seg->w0.x(),    seg->w0.y(),    seg->w0.z());
    Eigen::Quaterniond alpha_q(0.0, seg->alpha.x(), seg->alpha.y(), seg->alpha.z());

    Eigen::Quaterniond q_star;
    q_star.coeffs() = q0.coeffs()
                    + 0.5  * (q0 * omega_q).coeffs() * t
                    + 0.25 * (q0 * alpha_q).coeffs() * t * t;
    q_star.normalize();

    return SE3(q_star, p_star);
}

} // namespace IncLIO
