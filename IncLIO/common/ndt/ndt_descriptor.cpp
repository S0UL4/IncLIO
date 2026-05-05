#include "ndt/ndt_descriptor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace IncLIO {

namespace {

constexpr int    kHistBins    = 10;
constexpr int    kNumShapes   = 8;
constexpr double kShapeBin    = 0.3;
constexpr double kShapeMax    = kShapeBin * kNumShapes;  // 2.4
constexpr double kHistRange   = 3.0;

// Sectorkey-bounded shift search ratio (NDTMC-LIO-SAM: SEARCH_RATIO=0.3 →
// search ±15% of the sectors around the vkey-aligned shift).
constexpr double kSearchRatio = 0.3;

/// Per-cell tracker of the dominant shape mode across z-layers in one
/// (ring, sector). Mirrors NDTMC::GridCell::addShape: a shape becomes
/// "dominant" only if it is the most-frequent AND covers >= 1/8 of all
/// observations in this cell.
struct GridCell {
    int counts[kNumShapes] = {0};
    int shape_max = -1;
    int count_max = 0;

    void AddShape(int shape) {
        counts[shape - 1]++;
        int total = 0;
        for (int c : counts) total += c;
        const double quarter = static_cast<double>(total) / 8.0;
        if (counts[shape - 1] > count_max && counts[shape - 1] >= quarter) {
            count_max = counts[shape - 1];
            shape_max = shape;
        } else if (count_max < quarter) {
            shape_max = -1;
        }
    }
};

inline float xy2theta(float x, float y) {
    constexpr float kRad2Deg = static_cast<float>(180.0 / M_PI);
    if (x >= 0.f && y >= 0.f) return kRad2Deg * std::atan(y / x);
    if (x <  0.f && y >= 0.f) return 180.f - kRad2Deg * std::atan(y / -x);
    if (x <  0.f && y <  0.f) return 180.f + kRad2Deg * std::atan(y / x);
    if (x >= 0.f && y <  0.f) return 360.f - kRad2Deg * std::atan(-y / x);
    return 0.f;
}

bool CalculateSectorId(const Vec3d& mean,
                       int pc_num_ring, int pc_num_sector, double pc_max_radius,
                       int& ring_idx, int& sector_idx) {
    const double r = std::sqrt(mean.x() * mean.x() + mean.y() * mean.y());
    if (r > pc_max_radius) return false;
    const float theta = xy2theta(static_cast<float>(mean.x()),
                                 static_cast<float>(mean.y()));
    ring_idx = std::max(std::min(pc_num_ring,
                  static_cast<int>(std::ceil((r / pc_max_radius) * pc_num_ring))), 1);
    sector_idx = std::max(std::min(pc_num_sector,
                  static_cast<int>(std::ceil((theta / 360.0) * pc_num_sector))), 1);
    return true;
}

int CalculateLayer(double z, int pc_num_z, double pc_max_z) {
    if (z < 0.0 || z > pc_max_z) return -1;
    return std::max(std::min(pc_num_z,
                  static_cast<int>(std::ceil((z / pc_max_z) * pc_num_z))), 1);
}

/// Column-wise circular right-shift by `n_shift` (NDTMC-LIO-SAM `circshift`).
Eigen::MatrixXd Circshift(const Eigen::MatrixXd& m, int n_shift) {
    if (n_shift == 0) return m;
    const int cols = static_cast<int>(m.cols());
    Eigen::MatrixXd shifted = Eigen::MatrixXd::Zero(m.rows(), cols);
    for (int c = 0; c < cols; ++c) {
        const int dst = ((c + n_shift) % cols + cols) % cols;
        shifted.col(dst) = m.col(c);
    }
    return shifted;
}

/// Sectorkey: 1×S column-mean vector. Shifting a sectorkey by k columns is
/// equivalent to shifting the full descriptor by k columns — used as a cheap
/// pre-alignment to bound the brute-force shift search.
Eigen::MatrixXd MakeSectorkey(const Eigen::MatrixXd& desc) {
    Eigen::MatrixXd vkey(1, desc.cols());
    for (int c = 0; c < desc.cols(); ++c) {
        vkey(0, c) = desc.col(c).mean();
    }
    return vkey;
}

/// Brute-force shift search on the (cheap) 1×S sectorkey to seed the bounded
/// search on the (expensive) full descriptor.
int FastAlignUsingVkey(const Eigen::MatrixXd& vkey1, const Eigen::MatrixXd& vkey2) {
    int    argmin_shift = 0;
    double min_norm     = std::numeric_limits<double>::infinity();
    for (int s = 0; s < vkey1.cols(); ++s) {
        const Eigen::MatrixXd v2_shifted = Circshift(vkey2, s);
        const double n = (vkey1 - v2_shifted).norm();
        if (n < min_norm) {
            min_norm     = n;
            argmin_shift = s;
        }
    }
    return argmin_shift;
}

/// Global Pearson-style correlation distance.
/// Cells that were zero in the original (i.e. empty bins) are preserved
/// as zero after mean-centering — empty-bin sparsity carries information
/// about the local geometry, not just absence.
double DistDirect(const Eigen::MatrixXd& frame, const Eigen::MatrixXd& submap) {
    const double a_mean = frame.mean();
    const double b_mean = submap.mean();
    Eigen::MatrixXd a = frame.array() - a_mean;
    a = (a.array() == -a_mean).select(0.0, a);
    Eigen::MatrixXd b = submap.array() - b_mean;
    b = (b.array() == -b_mean).select(0.0, b);
    const double num   = (a.cwiseProduct(b).colwise().sum()).sum();
    const double denom = std::sqrt((a.cwiseProduct(a).colwise().sum()).sum() * (b.cwiseProduct(b).colwise().sum()).sum());
    if (denom == 0.0) return 1.0;
    return 1.0 - std::abs(num / denom);
}

}  // namespace

void NdtDescriptor::Build(const std::vector<NdtSnapshot>& voxels,
                          Eigen::MatrixXd& desc_out,
                          Eigen::VectorXd& hist_out) const {
    const int R = cfg_.pc_num_ring;
    const int S = cfg_.pc_num_sector;
    const int Z = cfg_.pc_num_z;

    Eigen::MatrixXd desc_1 = Eigen::MatrixXd::Zero(R, S);
    Eigen::MatrixXd desc_2 = Eigen::MatrixXd::Zero(R, S);   
    hist_out = Eigen::VectorXd::Zero(kHistBins);

    std::vector<std::vector<std::vector<GridCell>>> grid(
        R, std::vector<std::vector<GridCell>>(
               S, std::vector<GridCell>(Z)));

    constexpr double kHistInterval = kHistRange / kHistBins;

    for (const auto& v : voxels) {
        if (v.n_pts < 5 || v.evals(2) <= 0.0) continue;

        int ring_idx = 0, sector_idx = 0;
        if (!CalculateSectorId(v.mean, R, S, cfg_.pc_max_radius, ring_idx, sector_idx))
            continue;

        const int z_idx = CalculateLayer(v.mean.z(), Z, cfg_.pc_max_z);
        if (z_idx == -1) continue;

        const double mid2 = v.evals(1) * v.evals(1);
        if (mid2 <= 0.0) continue;
        const double value = v.evals(2) * v.evals(0) / mid2;

        if (value < kHistRange) {
            const int h_idx = static_cast<int>(std::floor(value / kHistInterval));
            if (h_idx >= 0 && h_idx < kHistBins) hist_out(h_idx) += 1.0;
        }

        if (value > 0.0 && value < kShapeMax) {
            const int shape_id = static_cast<int>(std::floor(value / kShapeBin));
            grid[ring_idx - 1][sector_idx - 1][z_idx - 1].AddShape(shape_id + 1);

            // Differential entropy of a 3D Gaussian (constant terms + 0.5·log|Σ|),
            // weighted by layer index so taller features count more.
            constexpr double N = 3.0;
            const double det = v.cov.determinant();
            if (det > 0.0) {
                const double entropy = (N * 0.5) * (1.0 + std::log(2.0 * M_PI))
                                     + 0.5 * std::log(det);
                desc_1(ring_idx - 1, sector_idx - 1) +=
                    entropy * z_idx / static_cast<double>(Z);
            }
        }
    }

    for (int i = 0; i < R; ++i) {
        for (int j = 0; j < S; ++j) {
            double w = 0.0;
            for (int k = 0; k < Z; ++k) {
                if (grid[i][j][k].shape_max == -1) continue;
                w += static_cast<double>(grid[i][j][k].shape_max * (k + 1)) / Z;
            }
            desc_2(i, j) = w;
        }
    }

    desc_out.resize(2 * R, S);
    desc_out.topRows(R)    = desc_1;
    desc_out.bottomRows(R) = desc_2;
}

std::pair<double, int> NdtDescriptor::Distance(const Eigen::MatrixXd& a,
                                               const Eigen::MatrixXd& b) {
    if (a.rows() == 0 || a.cols() == 0 ||
        a.rows() != b.rows() || a.cols() != b.cols()) {
        return {1.0, -1};
    }
    const int S = static_cast<int>(a.cols());

    // 1. Cheap pre-alignment via sectorkey (NDTMC-LIO-SAM fastAlignUsingVkey).
    const Eigen::MatrixXd vkey_a = MakeSectorkey(a);
    const Eigen::MatrixXd vkey_b = MakeSectorkey(b);
    const int argmin_vkey = FastAlignUsingVkey(vkey_a, vkey_b);

    // 2. Bounded shift search around the vkey-aligned offset.
    const int search_radius = static_cast<int>(std::round(0.5 * kSearchRatio * S));
    std::vector<int> shift_idx_search_space;
    shift_idx_search_space.reserve(2 * search_radius + 1);
    shift_idx_search_space.push_back(argmin_vkey);
    for (int ii = 1; ii <= search_radius; ++ii) {
        shift_idx_search_space.push_back(((argmin_vkey + ii) % S + S) % S);
        shift_idx_search_space.push_back(((argmin_vkey - ii) % S + S) % S);
    }

    double best_dist  = std::numeric_limits<double>::infinity();
    int    best_shift = 0;
    for (const int n_shift : shift_idx_search_space) {
        const Eigen::MatrixXd b_shifted = Circshift(b, n_shift);
        const double d = DistDirect(a, b_shifted);
        if (std::isnan(d)) continue;
        if (d < best_dist) {
            best_dist  = d;
            best_shift = n_shift;
        }
    }
    if (!std::isfinite(best_dist)) return {1.0, -1};
    return {best_dist, best_shift};
}

}  // namespace IncLIO
