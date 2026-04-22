// Benchmark: IncLIO custom NDT/transform vs PCL equivalents.
//
// Four sections:
//   1. transformCloudOMP  vs  pcl::transformPointCloud
//   2. NdtMap::AddCloud   vs  pcl::NdT::setInputTarget  (map build only)
//   3. AlignNdt           vs  pcl::NdT::align            (alignment only, pre-built map)
//   4. Full pipeline      vs  pcl::NdT                   (build + align together)
//
// Build:  colcon build --packages-select inclio_ros2 --cmake-args -DBUILD_BENCHMARKS=ON
// Run:    ./install/inclio_ros2/lib/inclio_ros2/benchmark_ndt

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "ndt/ndt_map.hpp"
#include "ndt/ndt_registration.hpp"
#include "utils/logger.hpp"
#include "utils/point_types.hpp"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

using namespace IncLIO;
using Clock = std::chrono::high_resolution_clock;
using Ms    = std::chrono::duration<double, std::milli>;

// ─── helpers ──────────────────────────────────────────────────────────────────

static CloudPtr MakeCloud(int n, float range = 30.f, unsigned seed = 42) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->reserve(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> xy(-range, range);
    std::uniform_real_distribution<float> z(-2.f, 4.f);
    for (int i = 0; i < n; ++i) {
        pcl::PointXYZI p;
        p.x = xy(rng); p.y = xy(rng); p.z = z(rng);
        p.intensity = 1.f;
        cloud->push_back(p);
    }
    cloud->width   = static_cast<uint32_t>(cloud->size());
    cloud->height  = 1;
    cloud->is_dense = true;
    return cloud;
}

// Slightly displaced copy: 1 m translation + ~5° yaw.
static CloudPtr MakeDisplacedCloud(const CloudPtr& ref) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    const float a = 0.0873f; // ~5 degrees
    T(0,0) = std::cos(a); T(0,1) = -std::sin(a); T(0,3) = 1.0f;
    T(1,0) = std::sin(a); T(1,1) =  std::cos(a); T(1,3) = 0.5f;
    auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*ref, *out, T);
    return out;
}

// Returns {mean_ms, stddev_ms} over `reps` runs.
template <typename Fn>
static std::pair<double,double> Time(int reps, Fn&& fn) {
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
              << std::left  << std::setw(38) << "  Method"
              << std::right << std::setw(9)  << "Points"
              << std::setw(11) << "Mean ms"
              << std::setw(10) << "Std ms"
              << std::setw(10) << "Mpts/s"
              << "\n";
    PrintSep();
}

static void PrintRow(const std::string& label, int n, double mean, double std) {
    double throughput = (n / 1e6) / (mean / 1e3);
    std::cout << std::left  << std::setw(38) << ("  " + label)
              << std::right << std::setw(9)  << n
              << std::setw(11) << std::fixed << std::setprecision(2) << mean
              << std::setw(10) << std::fixed << std::setprecision(2) << std
              << std::setw(10) << std::fixed << std::setprecision(2) << throughput
              << "\n";
}

static void PrintSpeedup(double a_ms, double b_ms) {
    std::cout << "    speedup: " << std::fixed << std::setprecision(2)
              << (b_ms / a_ms) << "x  (IncLIO / PCL)\n";
    PrintSep();
}

// ─── benchmark 1: point cloud transform ───────────────────────────────────────

static void BenchTransform() {
    PrintHeader("Transform: IncLIO::transformCloudOMP  vs  pcl::transformPointCloud");

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0,0) =  0.866f; T(0,1) = -0.5f;   T(0,3) = 3.0f;
    T(1,0) =  0.5f;   T(1,1) =  0.866f; T(1,3) = 1.0f;

    const int reps = 50;
    for (int n : {1000, 5000, 10000, 30000, 100000, 200000}) {
        auto src = MakeCloud(n);
        auto dst = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(n, 1);

        auto [m_omp, s_omp] = Time(reps, [&] { transformCloudOMP(*src, *dst, T); });
        PrintRow("transformCloudOMP", n, m_omp, s_omp);

        auto [m_pcl, s_pcl] = Time(reps, [&] { pcl::transformPointCloud(*src, *dst, T); });
        PrintRow("pcl::transformPointCloud", n, m_pcl, s_pcl);

        PrintSpeedup(m_omp, m_pcl);
    }
}

// ─── benchmark 2: NDT map build ───────────────────────────────────────────────

static void BenchMapBuild() {
    PrintHeader("NDT Map Build: IncLIO NdtMap::AddCloud  vs  pcl::NdT::setInputTarget");

    const int   reps       = 20;
    const float voxel_size = 1.0f;
    // Use a tight range so point density >= 6 pts/m^3 (PCL's minimum per voxel).
    // +-5 m -> ~600 m^3 -> n=5k gives ~8 pts/voxel, n=100k gives ~166.
    const float range      = 5.f;

    for (int n : {5000, 10000, 30000, 100000}) {
        auto cloud = MakeCloud(n, range);

        auto [m_inc, s_inc] = Time(reps, [&] {
            NdtMap::Options opts;
            opts.voxel_size = voxel_size;
            NdtMap map(opts);
            map.AddCloud(cloud);
        });
        PrintRow("NdtMap::AddCloud", n, m_inc, s_inc);

        auto [m_pcl, s_pcl] = Time(reps, [&] {
            pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
            ndt.setResolution(voxel_size);
            ndt.setInputTarget(cloud);
            // Force voxel grid construction (lazy in PCL — trigger it explicitly)
            pcl::PointCloud<pcl::PointXYZI> dummy;
            ndt.setInputSource(cloud);
            ndt.align(dummy, Eigen::Matrix4f::Identity());
        });
        PrintRow("pcl::NdT::setInputTarget (+1 align)", n, m_pcl, s_pcl);

        PrintSpeedup(m_inc, m_pcl);
    }
}

// ─── benchmark 3: NDT alignment only (pre-built map) ─────────────────────────

static void BenchAlign() {
    PrintHeader("NDT Align only (pre-built map): IncLIO AlignNdt  vs  pcl::NdT::align");

    const int   reps       = 10;
    const float voxel_size = 1.0f;
    const int   max_iter   = 4;
    const float range      = 5.f;

    for (int n : {5000, 10000, 30000, 100000}) {
        auto target = MakeCloud(n, range, 42);
        auto source = MakeDisplacedCloud(target);

        // Pre-build IncLIO map once.
        NdtMap::Options map_opts;
        map_opts.voxel_size = voxel_size;
        NdtMap ndt_map(map_opts);
        ndt_map.AddCloud(target);

        NdtRegistration::Options reg_opts;
        reg_opts.max_iteration = max_iter;

        auto [m_inc, s_inc] = Time(reps, [&] {
            NdtRegistration reg(reg_opts);
            reg.SetSource(source);
            reg.SetNdtMap(&ndt_map);
            SE3 pose;
            reg.AlignNdt(pose);
        });
        PrintRow("AlignNdt (4 iters, GN+TBB)", n, m_inc, s_inc);

        // Pre-build PCL target once.
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_pcl;
        ndt_pcl.setResolution(voxel_size);
        ndt_pcl.setMaximumIterations(max_iter);
        ndt_pcl.setInputTarget(target);

        auto [m_pcl, s_pcl] = Time(reps, [&] {
            ndt_pcl.setInputSource(source);
            pcl::PointCloud<pcl::PointXYZI> output;
            ndt_pcl.align(output);
        });
        PrintRow("pcl::NdT::align (4 iters)", n, m_pcl, s_pcl);

        PrintSpeedup(m_inc, m_pcl);
    }
}

// ─── benchmark 4: full pipeline (build + align) ───────────────────────────────

static void BenchFull() {
    PrintHeader("NDT Full Pipeline (build + align): IncLIO  vs  pcl::NdT");

    const int   reps       = 10;
    const float voxel_size = 1.0f;
    const int   max_iter   = 4;
    const float range      = 5.f;

    for (int n : {5000, 10000, 30000, 100000}) {
        auto target = MakeCloud(n, range, 42);
        auto source = MakeDisplacedCloud(target);

        auto [m_inc, s_inc] = Time(reps, [&] {
            NdtMap::Options map_opts;
            map_opts.voxel_size = voxel_size;
            NdtMap ndt_map(map_opts);
            ndt_map.AddCloud(target);

            NdtRegistration::Options reg_opts;
            reg_opts.max_iteration = max_iter;
            NdtRegistration reg(reg_opts);
            reg.SetSource(source);
            reg.SetNdtMap(&ndt_map);
            SE3 pose;
            reg.AlignNdt(pose);
        });
        PrintRow("IncLIO (AddCloud + AlignNdt)", n, m_inc, s_inc);

        auto [m_pcl, s_pcl] = Time(reps, [&] {
            pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
            ndt.setResolution(voxel_size);
            ndt.setMaximumIterations(max_iter);
            ndt.setInputTarget(target);
            ndt.setInputSource(source);
            pcl::PointCloud<pcl::PointXYZI> output;
            ndt.align(output);
        });
        PrintRow("pcl::NdT (setTarget + align)", n, m_pcl, s_pcl);

        PrintSpeedup(m_inc, m_pcl);
    }
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    // Suppress per-iteration NDT logs so they don't pollute timing output.
    InitLogger(spdlog::level::warn);

    std::cout << "IncLIO benchmark — built " __DATE__ " " __TIME__ "\n";

    BenchTransform();
    BenchMapBuild();
    BenchAlign();
    BenchFull();

    std::cout << "\nDone.\n";
    return 0;
}
