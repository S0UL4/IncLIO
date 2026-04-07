#ifndef INCLIO_UI_CLOUD_H
#define INCLIO_UI_CLOUD_H

#include "common/utils/eigen_types.hpp"
#include "common/utils/point_types.hpp"

#include <pangolin/gl/glvbo.h>

namespace IncLIO::ui {

/// Point cloud for UI rendering.
/// Static point clouds can use this for rendering.
class UiCloud {
   public:
    /// Color mode for rendering this point cloud
    enum UseColor {
        PCL_COLOR,        // PCL color (reddish)
        INTENSITY_COLOR,  // Intensity
        HEIGHT_COLOR,     // Height
        GRAY_COLOR,       // Gray
    };

    UiCloud() {}
    UiCloud(CloudPtr cloud);

    /// Set a UI cloud from a PCL point cloud
    /// @param cloud  PCL point cloud
    /// @param pose   Point cloud pose, transforms to global frame
    void SetCloud(CloudPtr cloud, const SE3& pose);

    /// Render this point cloud
    void Render();

    void SetRenderColor(UseColor use_color);

   private:
    Vec4f IntensityToRgbPCL(const float& intensity) const {
        int index = int(intensity * 6);
        index = index % intensity_color_table_pcl_.size();
        return intensity_color_table_pcl_[index];
    }

    UseColor use_color_ = UseColor::PCL_COLOR;

    std::vector<Vec3f> xyz_data_;              // XYZ buffer
    std::vector<Vec4f> color_data_pcl_;        // PCL color buffer
    std::vector<Vec4f> color_data_intensity_;  // Intensity color buffer
    std::vector<Vec4f> color_data_height_;     // Height color buffer
    std::vector<Vec4f> color_data_gray_;       // Gray color buffer

    pangolin::GlBuffer vbo_;  // GPU vertex buffer
    pangolin::GlBuffer cbo_;  // GPU color buffer

    /// Build PCL intensity color table
    void BuildIntensityTable();
    static std::vector<Vec4f> intensity_color_table_pcl_;
};

}  // namespace IncLIO::ui
#endif
