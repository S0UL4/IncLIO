#include "ndt/ndt_map.hpp"

#include <execution>
#include <set>

namespace IncLIO {

void NdtMap::AddCloud(CloudPtr cloud_world) {
    std::set<KeyType, less_vec<3>> active_voxels;

    for (const auto& p : cloud_world->points) {
        auto pt = ToVec3d(p);
        auto key = PointToKey(pt);
        auto iter = grids_.find(key);

        if (iter == grids_.end()) {
            // New voxel
            data_.push_front({key, NdtVoxel(pt)});
            grids_.insert({key, data_.begin()});

            // LRU eviction
            if (data_.size() >= options_.capacity) {
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } else {
            // Existing voxel — add point and move to front (most recently used)
            iter->second->second.AddPoint(pt);
            data_.splice(data_.begin(), data_, iter->second);
            iter->second = data_.begin();
        }

        active_voxels.emplace(key);
    }

    // Update distributions for all modified voxels (parallel)
    std::for_each(std::execution::par_unseq, active_voxels.begin(), active_voxels.end(),
                  [this](const auto& key) {
                      grids_[key]->second.UpdateDistribution(
                          flag_first_scan_,
                          options_.min_pts_in_voxel,
                          options_.max_pts_in_voxel);
                  });

    flag_first_scan_ = false;
}

void NdtMap::GenerateNearbyGrids() {
    if (options_.nearby_type == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),
                         KeyType(0, 1, 0),  KeyType(0, -1, 0), KeyType(0, 0, -1),
                         KeyType(0, 0, 1)};
    }
}

NdtVoxel* NdtMap::GetVoxel(const KeyType& key) {
    auto it = grids_.find(key);
    if (it != grids_.end()) {
        return &(it->second->second);
    }
    return nullptr;
}

std::vector<NdtVoxel*> NdtMap::GetNearbyVoxels(const Vec3d& point) {
    std::vector<NdtVoxel*> result;
    KeyType center = PointToKey(point);
    for (const auto& offset : nearby_grids_) {
        KeyType key = center + offset;
        auto* v = GetVoxel(key);
        if (v && v->IsValid()) {
            result.push_back(v);
        }
    }
    return result;
}

} // namespace IncLIO
