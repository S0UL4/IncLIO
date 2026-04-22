#ifndef INCLIO_CT_UNDISTORT_HPP
#define INCLIO_CT_UNDISTORT_HPP

#include "inclio/state.hpp"
#include "utils/eigen_types.hpp"

#include <vector>

namespace IncLIO {

/// One IMU-to-IMU interval precomputed for DLIO continuous-time queries.
/// Built once per scan by BuildCTSegments(); queried once per point by CTQueryPose().
struct CTSegment {
    double t0    = 0.0;
    Vec3d  p0    = Vec3d::Zero();    // world-frame position at t0
    Vec3d  v0    = Vec3d::Zero();    // world-frame velocity at t0
    SO3    R0;                        // rotation at t0
    Vec3d  a0    = Vec3d::Zero();    // world-frame acceleration at t0: R*(a-ba)+g
    Vec3d  j     = Vec3d::Zero();    // linear jerk: (a1-a0)/dt
    Vec3d  w0    = Vec3d::Zero();    // body-frame bias-corrected angular velocity at t0
    Vec3d  alpha = Vec3d::Zero();    // angular acceleration: (w1-w0)/dt
};

/// Build one CTSegment per IMU interval from the outputs of Predict().
/// states  — size M+1 (state before first IMU, then after each IMU)
/// a_world — size M   (world-frame bias-corrected accel at each IMU sample: R*(a-ba)+g)
/// w_body  — size M   (body-frame bias-corrected gyro at each IMU sample: ω-bg)
std::vector<CTSegment> BuildCTSegments(
    const std::vector<Stated>& states,
    const std::vector<Vec3d>&  a_world,
    const std::vector<Vec3d>&  w_body);

/// Evaluate DLIO eq. (5) to get the sensor pose at timestamp t_pt.
/// segs must have been built by BuildCTSegments() for the enclosing scan.
SE3 CTQueryPose(double t_pt, const std::vector<CTSegment>& segs);

} // namespace IncLIO

#endif // INCLIO_CT_UNDISTORT_HPP
