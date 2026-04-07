#ifndef INCLIO_UI_CAR_H
#define INCLIO_UI_CAR_H

#include <pangolin/gl/glvbo.h>

#include "common/utils/eigen_types.hpp"

namespace IncLIO::ui {

/// Car displayed in the UI
class UiCar {
   public:
    UiCar(const Vec3f& color) : color_(color) {}

    /// Set car pose, update GPU buffer vertices
    void SetPose(const SE3& pose);

    /// Render the car
    void Render();

   private:
    Vec3f color_;
    pangolin::GlBuffer vbo_;

    static std::vector<Vec3f> car_vertices_;  // Car vertices
};

}  // namespace IncLIO::ui

#endif
