#ifndef INCLIO_UI_PANGOLIN_WINDOW_H
#define INCLIO_UI_PANGOLIN_WINDOW_H

#include "common/utils/eigen_types.hpp"
#include "common/utils/point_types.hpp"

#include <map>
#include <memory>

namespace IncLIO::ui {

class PangolinWindowImpl;

/// @note This class does not directly involve any OpenGL or Pangolin operations;
///       those should be placed in PangolinWindowImpl.
class PangolinWindow {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PangolinWindow();
    ~PangolinWindow();

    /// @brief Initialize the window and start the render thread in the background.
    /// @note Non-OpenGL/Pangolin initialization should go in this function body;
    ///       OpenGL/Pangolin-related setup should go in PangolinWindowImpl::Init.
    bool Init();

    /// Update the global lidar map point cloud
    void UpdatePointCloudGlobal(const std::map<Vec2i, CloudPtr, less_vec<2>>& cloud);

    /// Update Kalman filter state
    void UpdateNavState(const SE3& pose, const Vec3d& vel, const Vec3d& ba, const Vec3d& bg);

    /// Update a scan and its corresponding pose
    void UpdateScan(CloudPtr cloud, const SE3& pose);

    /// Wait for the display thread to finish and release resources
    void Quit();

    /// Whether the user has quit the UI
    bool ShouldQuit();

    /// Set the IMU-to-LiDAR extrinsic transform
    void SetTImuLidar(const SE3& T_imu_lidar);

    /// Set how many scan frames to retain
    void SetCurrentScanSize(int current_scan_size);

   private:
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};
}  // namespace IncLIO::ui

#endif
