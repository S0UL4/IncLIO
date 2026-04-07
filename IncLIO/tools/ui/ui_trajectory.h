#ifndef INCLIO_UI_TRAJECTORY_H
#define INCLIO_UI_TRAJECTORY_H

#include "common/utils/eigen_types.hpp"

#include <pangolin/gl/glvbo.h>

namespace IncLIO::ui {

/// Trajectory rendering in UI
class UiTrajectory {
   public:
    UiTrajectory(const Vec3f& color) : color_(color) { pos_.reserve(max_size_); }

    /// Add a trajectory point
    void AddPt(const SE3& pose);

    /// Render this trajectory
    void Render();

    void Clear() {
        pos_.clear();
        pos_.reserve(max_size_);
        vbo_.Free();
    }

   private:
    int max_size_ = 1e6;           // Maximum number of recorded points
    std::vector<Vec3f> pos_;       // Trajectory position data
    Vec3f color_ = Vec3f::Zero();  // Trajectory display color
    pangolin::GlBuffer vbo_;       // GPU vertex buffer
};

}  // namespace IncLIO::ui

#endif  // INCLIO_UI_TRAJECTORY_H
