#ifndef INCLIO_NDT_MAP_HPP
#define INCLIO_NDT_MAP_HPP

#include "utils/eigen_types.hpp"
#include "utils/point_types.hpp"
#include "ndt/ndt_voxel.hpp"

#include <list>
#include <unordered_map>
#include <vector>

namespace IncLIO {

/// Incremental NDT map using spatial hashing with LRU eviction.
///
/// Maintains a voxelized Normal Distribution map that supports:
///   - Incremental point cloud insertion
///   - LRU-based capacity limit for real-time operation
///   - Nearby voxel lookup (CENTER or NEARBY6 patterns)
class NdtMap {
   public:
    enum class NearbyType {
        CENTER,   // Only the center voxel
        NEARBY6,  // 6-connected neighbors (up/down/left/right/front/back) + center
    };

    struct Options {
        double voxel_size = 1.0;
        double inv_voxel_size = 1.0;
        int min_pts_in_voxel = 5;   // Min points before estimating distribution
        int max_pts_in_voxel = 50;  // Stop accumulating after this count
        size_t capacity = 100000;   // Max cached voxels (LRU eviction)
        NearbyType nearby_type = NearbyType::NEARBY6;
    };

    using KeyType = Vec3i;

    NdtMap() { Init(); }
    explicit NdtMap(const Options& options) : options_(options) { Init(); }

    /// Insert a world-frame point cloud into the map
    void AddCloud(CloudPtr cloud_world);

    /// Number of active voxels
    size_t NumVoxels() const { return grids_.size(); }

    /// Get voxel by grid index (nullptr if not found)
    NdtVoxel* GetVoxel(const KeyType& key);

    /// Get valid nearby voxels for a query point (in world coordinates)
    std::vector<NdtVoxel*> GetNearbyVoxels(const Vec3d& point);

    /// Convert a world-frame point to its voxel grid index
    KeyType PointToKey(const Vec3d& point) const {
        return CastToInt(point * options_.inv_voxel_size);
    }

    /// Access nearby grid offsets
    const std::vector<KeyType>& GetNearbyGrids() const { return nearby_grids_; }

    const Options& GetOptions() const { return options_; }

    void Clear() {
        data_.clear();
        grids_.clear();
    }

   private:
    void Init() {
        options_.inv_voxel_size = 1.0 / options_.voxel_size;
        GenerateNearbyGrids();
    }

    void GenerateNearbyGrids();

    Options options_;

    using KeyAndData = std::pair<KeyType, NdtVoxel>;
    std::list<KeyAndData> data_;  // LRU list: front = most recently used
    std::unordered_map<KeyType, std::list<KeyAndData>::iterator, hash_vec<3>> grids_;
    std::vector<KeyType> nearby_grids_;

    bool flag_first_scan_ = true;
};

} // namespace IncLIO

#endif // INCLIO_NDT_MAP_HPP
