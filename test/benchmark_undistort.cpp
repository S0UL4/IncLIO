// Benchmark: DLIO continuous-time undistortion vs original slerp/lerp.
//
// Section 1 — Speed: full Undistort() equivalent wall time, varying N
// Section 2 — BuildCTSegments per-scan overhead across IMU counts
// Section 3 — Accuracy: max pos/angle deviation (CT vs Slerp) at varying IMU rates
//
// Build:  colcon build --packages-select inclio_ros2 --cmake-args -DBUILD_BENCHMARKS=ON
// Run:    ./install/inclio_ros2/lib/inclio_ros2/benchmark_undistort

#include "inclio/ct_undistort.hpp"
#include "inclio/state.hpp"
#include "utils/math_utils.hpp"
#include "utils/point_types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <execution>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace IncLIO;
using Clock = std::chrono::high_resolution_clock;
using Ms    = std::chrono::duration<double, std::milli>;

// ─── helpers ──────────────────────────────────────────────────────────────────

// Returns {mean_ms, stddev_ms} over `reps` runs, with 5-run warmup.
template <typename Fn>
static std::pair<double, double> Time(int reps, Fn&& fn) {
    for (int i = 0; i < 5; ++i) fn();  // warmup — prime thread pool + caches
    std::vector<double> t(reps);
    for (int i = 0; i < reps; ++i) {
        auto t0 = Clock::now();
        fn();
        t[i] = Ms(Clock::now() - t0).count();
    }
    double mean = std::accumulate(t.begin(), t.end(), 0.0) / reps;
    double var  = 0.0;
    for (double x : t) var += (x - mean) * (x - mean);
    return {mean, std::sqrt(var / reps)};
}

static void PrintSep() { std::cout << std::string(72, '-') << "\n"; }

static void PrintHeader(const std::string& title) {
    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  " << title << "\n"
              << std::string(72, '=') << "\n"
              << std::left  << std::setw(22) << "  Method"
              << std::right << std::setw(9)  << "Points"
              << std::setw(11) << "Mean ms"
              << std::setw(10) << "Std ms"
              << std::setw(11) << "Mpts/s"
              << "\n";
    PrintSep();
}

static void PrintRow(const std::string& label, int n, double mean, double std) {
    double throughput = static_cast<double>(n) / 1e6 / (mean / 1e3);
    std::cout << std::left  << std::setw(22) << ("  " + label)
              << std::right << std::setw(9)  << n
              << std::setw(11) << std::fixed << std::setprecision(3) << mean
              << std::setw(10) << std::fixed << std::setprecision(3) << std
              << std::setw(11) << std::fixed << std::setprecision(2) << throughput
              << "\n";
}

// ─── trajectory builder ────────────────────────────────────────────────────────

struct Trajectory {
    std::vector<Stated> states;   // M+1 entries
    std::vector<Vec3d>  a_world;  // M entries
    std::vector<Vec3d>  w_body;   // M entries
    double              t_begin;
    double              t_end;
};

// Builds a physically realistic trajectory: forward motion + lateral slalom.
// dt controls IMU rate (e.g. 0.005 = 200 Hz, 0.02 = 50 Hz, 0.04 = 25 Hz).
// num_imu is always matched to fill a 100 ms scan (20 @ 200Hz, 5 @ 50Hz, etc.).
static Trajectory MakeTrajectory(int num_imu, double dt) {
    Trajectory traj;
    traj.t_begin = 1000.0;

    Stated s0;
    s0.timestamp_ = traj.t_begin;
    s0.p_  = Vec3d::Zero();
    s0.v_  = Vec3d(3.0, 0.0, 0.0);   // 3 m/s forward
    s0.R_  = SO3();
    s0.ba_ = Vec3d::Zero();
    s0.bg_ = Vec3d::Zero();
    traj.states.push_back(s0);

    const Vec3d g(0.0, 0.0, -9.81);

    for (int k = 0; k < num_imu; ++k) {
        const Stated& prev = traj.states.back();

        // Sinusoidal lateral maneuver — produces significant jerk between samples
        double phase = 2.0 * M_PI * k / num_imu;
        Vec3d a_body(2.0,
                     4.0 * std::sin(phase),    // lateral: peaks ±4 m/s²
                     0.5 * std::cos(phase));

        Vec3d w(0.3 * std::sin(phase),         // roll  oscillation
                0.1 * std::cos(phase),         // pitch oscillation
                0.8 * std::sin(2.0 * phase));  // yaw   oscillation

        Vec3d a_w = prev.R_ * a_body + g;
        traj.a_world.push_back(a_w);
        traj.w_body.push_back(w);

        Stated next;
        next.timestamp_ = prev.timestamp_ + dt;
        next.v_  = prev.v_  + a_w * dt;
        next.p_  = prev.p_  + prev.v_ * dt + 0.5 * a_w * dt * dt;
        next.R_  = prev.R_  * SO3::exp(w * dt);
        next.ba_ = Vec3d::Zero();
        next.bg_ = Vec3d::Zero();
        traj.states.push_back(next);
    }
    traj.t_end = traj.states.back().timestamp_;
    return traj;
}

// Uniform point timestamps over the scan window (offset from begin, in ms).
static FullPointCloudType MakePoints(int n, double scan_dur_s, unsigned seed = 42) {
    FullPointCloudType cloud;
    cloud.resize(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> xyz(-20.f, 20.f);
    for (int i = 0; i < n; ++i) {
        cloud.points[i].x    = xyz(rng);
        cloud.points[i].y    = xyz(rng);
        cloud.points[i].z    = xyz(rng);
        // time = offset from scan begin in milliseconds
        cloud.points[i].time = static_cast<double>(scan_dur_s * 1e3) * i / (n - 1);
    }
    return cloud;
}

// ─── section 1: speed ─────────────────────────────────────────────────────────

static void BenchSpeed(int reps) {
    PrintHeader("Speed: full Undistort loop (std::execution::par_unseq)  |  200 Hz IMU");

    // 200 Hz IMU, 100 ms scan → 20 samples
    const Trajectory traj = MakeTrajectory(20, 0.005);
    const SE3  T_end  = traj.states.back().GetSE3();
    const SE3  TIL    = SE3();  // identity extrinsics (isolates undistort math)

    for (int n : {10000, 25000, 50000, 100000}) {
        FullPointCloudType cloud = MakePoints(n, 0.1);

        // CT: BuildCTSegments once + CTQueryPose per point
        auto [m_ct, s_ct] = Time(reps, [&] {
            auto segs = BuildCTSegments(traj.states, traj.a_world, traj.w_body);
            std::for_each(std::execution::par_unseq,
                cloud.points.begin(), cloud.points.end(), [&](FullPointType& pt) {
                    double t_pt = traj.t_begin + pt.time * 1e-3;
                    SE3    Ti   = CTQueryPose(t_pt, segs);
                    Vec3d  pi   = ToVec3d(pt);
                    Vec3d  pc   = T_end.inverse() * Ti * TIL * pi;
                    pt.x = static_cast<float>(pc(0));
                    pt.y = static_cast<float>(pc(1));
                    pt.z = static_cast<float>(pc(2));
                });
        });
        PrintRow("CT (DLIO)", n, m_ct, s_ct);

        cloud = MakePoints(n, 0.1);  // reset to unmodified input

        // Slerp/lerp: PoseInterp per point (no pre-build step)
        auto [m_sl, s_sl] = Time(reps, [&] {
            std::for_each(std::execution::par_unseq,
                cloud.points.begin(), cloud.points.end(), [&](FullPointType& pt) {
                    double t_pt = traj.t_begin + pt.time * 1e-3;
                    SE3    Ti   = T_end;
                    Stated match;
                    math::PoseInterp<Stated>(t_pt, traj.states,
                        [](const Stated& s) { return s.timestamp_; },
                        [](const Stated& s) { return s.GetSE3(); },
                        Ti, match);
                    Vec3d pi = ToVec3d(pt);
                    Vec3d pc = T_end.inverse() * Ti * TIL * pi;
                    pt.x = static_cast<float>(pc(0));
                    pt.y = static_cast<float>(pc(1));
                    pt.z = static_cast<float>(pc(2));
                });
        });
        PrintRow("Slerp/Lerp", n, m_sl, s_sl);

        double overhead_pct = (m_ct - m_sl) / m_sl * 100.0;
        std::cout << "    CT overhead: " << std::showpos << std::fixed
                  << std::setprecision(1) << overhead_pct << std::noshowpos << "%\n";
        PrintSep();
    }
}

// ─── section 2: BuildCTSegments per-scan cost ─────────────────────────────────

static void BenchBuildCost(int reps) {
    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  BuildCTSegments per-scan overhead (no points involved)\n"
              << std::string(72, '=') << "\n"
              << std::left  << std::setw(20) << "  IMU rate"
              << std::right << std::setw(10) << "Samples"
              << std::setw(13) << "Mean µs"
              << std::setw(12) << "Std µs"
              << "\n";
    PrintSep();

    // (imu_rate_hz, num_imu_in_100ms_scan)
    for (auto [hz, num_imu] : std::vector<std::pair<int,int>>{{25,3},{50,5},{100,10},{200,20},{400,40}}) {
        double dt = 1.0 / hz;
        Trajectory traj = MakeTrajectory(num_imu, dt);

        auto [m, s] = Time(reps, [&] {
            volatile auto segs = BuildCTSegments(traj.states, traj.a_world, traj.w_body);
            (void)segs;
        });

        std::cout << std::left  << std::setw(20) << ("  " + std::to_string(hz) + " Hz")
                  << std::right << std::setw(10) << num_imu
                  << std::setw(13) << std::fixed << std::setprecision(2) << (m * 1e3)
                  << std::setw(12) << std::fixed << std::setprecision(2) << (s * 1e3)
                  << "\n";
    }
}

// ─── section 3: accuracy ──────────────────────────────────────────────────────

static void BenchAccuracy() {
    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Accuracy: max pose deviation (CT vs Slerp)  |  50 000 pts\n"
              << "  Trajectory: 3 m/s forward + lateral slalom (4 m/s² peak lateral)\n"
              << std::string(72, '=') << "\n"
              << std::left  << std::setw(16) << "  IMU rate"
              << std::right << std::setw(10) << "Samples"
              << std::setw(16) << "max Δp (mm)"
              << std::setw(18) << "max Δθ (mdeg)"
              << "\n";
    PrintSep();

    const int n_pts = 50000;

    for (auto [hz, num_imu] : std::vector<std::pair<int,int>>{{200,20},{100,10},{50,5},{25,3}}) {
        double dt = 1.0 / hz;
        Trajectory traj = MakeTrajectory(num_imu, dt);
        FullPointCloudType cloud = MakePoints(n_pts, num_imu * dt);

        auto segs = BuildCTSegments(traj.states, traj.a_world, traj.w_body);

        double max_dp    = 0.0;   // mm
        double max_dtheta = 0.0;  // millidegrees

        for (const auto& pt : cloud.points) {
            double t_pt = traj.t_begin + pt.time * 1e-3;

            SE3 Ti_ct = CTQueryPose(t_pt, segs);

            SE3    Ti_sl;
            Stated match;
            math::PoseInterp<Stated>(t_pt, traj.states,
                [](const Stated& s) { return s.timestamp_; },
                [](const Stated& s) { return s.GetSE3(); },
                Ti_sl, match);

            double dp     = (Ti_ct.translation() - Ti_sl.translation()).norm() * 1e3;
            double dtheta = (Ti_ct.so3() * Ti_sl.so3().inverse()).log().norm()
                            * (180.0 / M_PI) * 1e3;

            if (dp     > max_dp)     max_dp     = dp;
            if (dtheta > max_dtheta) max_dtheta = dtheta;
        }

        std::cout << std::left  << std::setw(16) << ("  " + std::to_string(hz) + " Hz")
                  << std::right << std::setw(10) << num_imu
                  << std::setw(16) << std::fixed << std::setprecision(4) << max_dp
                  << std::setw(18) << std::fixed << std::setprecision(4) << max_dtheta
                  << "\n";
    }

    std::cout << "\n  NOTE: deviation = how much CT and Slerp disagree on the same\n"
              << "  trajectory. CT is the ground truth here (matches IESKF kinematics).\n";
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n"
              << "  IncLIO Undistortion Benchmark\n"
              << "  CT (DLIO constant-jerk/angular-accel) vs Slerp/Lerp\n"
              << "  Trajectory: 3 m/s forward + sinusoidal lateral slalom\n";

    const int speed_reps = 200;
    const int build_reps = 10000;

    BenchSpeed(speed_reps);
    BenchBuildCost(build_reps);
    BenchAccuracy();

    std::cout << "\n" << std::string(72, '=') << "\n";
    return 0;
}
