#ifndef INCLIO_UI_PANGOLIN_WINDOW_IMPL_H
#define INCLIO_UI_PANGOLIN_WINDOW_IMPL_H

// Include Pangolin before PCL to suppress HAVE_OPENNI compilation warnings
#include <pangolin/pangolin.h>

#include "common/utils/point_types.hpp"
#include "tools/ui/pangolin_window.h"
#include "tools/ui/ui_car.h"
#include "tools/ui/ui_cloud.h"
#include "tools/ui/ui_trajectory.h"

#include <pcl/filters/voxel_grid.h>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace IncLIO::ui {

struct UiFrame;

class PangolinWindowImpl {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PangolinWindowImpl() = default;
    ~PangolinWindowImpl() = default;

    PangolinWindowImpl(const PangolinWindowImpl &) = delete;
    PangolinWindowImpl &operator=(const PangolinWindowImpl &) = delete;
    PangolinWindowImpl(PangolinWindowImpl &&) = delete;
    PangolinWindowImpl &operator=(PangolinWindowImpl &&) = delete;

    /// Initialize, create point cloud and car entities
    bool Init();

    /// Deinitialize
    bool DeInit();

    /// Render all information
    void Render();

   public:
    /// Background render thread
    std::thread render_thread_;

    /// Helper mutexes and atomic variables
    std::mutex mtx_map_cloud_;
    std::mutex mtx_current_scan_;
    std::mutex mtx_nav_state_;
    std::mutex mtx_gps_pose_;

    std::atomic<bool> exit_flag_;

    std::atomic<bool> cloud_global_need_update_;
    std::atomic<bool> kf_result_need_update_;
    std::atomic<bool> current_scan_need_update_;
    std::atomic<bool> lidarloc_need_update_;
    std::atomic<bool> pgoloc_need_update_;
    std::atomic<bool> gps_need_update_;

    CloudPtr current_scan_ = nullptr;  // Current scan
    SE3 current_pose_;                 // Pose of the current scan

    // Map point cloud
    std::map<Vec2i, CloudPtr, less_vec<2>> cloud_global_map_;

    /// GPS pose
    SE3 gps_pose_;

    /// Filter state
    SE3 pose_;
    Vec3d vel_;
    Vec3d bias_acc_;
    Vec3d bias_gyr_;
    Vec3d grav_;

    SE3 T_imu_lidar_;
    int max_size_of_current_scan_ = 2000;  // Number of current scan frames to retain

    //////////////////////////////// Rendering-related members below ///////////////////////////
   private:
    /// Create/release OpenGL buffers
    void AllocateBuffer();
    void ReleaseBuffer();

    void CreateDisplayLayout();

    void DrawAll();  // Draw the localization window

    /// Render point clouds, call various Update functions
    void RenderClouds();
    bool UpdateGps();
    bool UpdateGlobalMap();
    bool UpdateState();
    bool UpdateCurrentScan();

    void RenderLabels();

   private:
    /// Window layout parameters
    int win_width_ = 1920;
    int win_height_ = 1080;
    static constexpr float cam_focus_ = 5000;
    static constexpr float cam_z_near_ = 1.0;
    static constexpr float cam_z_far_ = 1e10;
    static constexpr int menu_width_ = 200;
    const std::string win_name_ = "IncLIO.UI";
    const std::string dis_main_name_ = "main";
    const std::string dis_3d_name_ = "Cam 3D";
    const std::string dis_3d_main_name_ = "Cam 3D Main";
    const std::string dis_plot_name_ = "Plot";
    const std::string dis_imgs_name = "Images";

    bool following_loc_ = true;  // Whether the camera follows the localization result

    // Text label
    pangolin::GlText gltext_label_global_;

    // Camera
    pangolin::OpenGlRenderState s_cam_main_;

    /// Cloud rendering
    ui::UiCar car_{Vec3f(0.2, 0.2, 0.8)};                                      // Blue car
    std::map<Vec2i, std::shared_ptr<ui::UiCloud>, less_vec<2>> cloud_map_ui_;   // Point cloud map for rendering
    std::shared_ptr<ui::UiCloud> current_scan_ui_;                              // Current scan
    std::deque<std::shared_ptr<ui::UiCloud>> scans_;                            // Queue of retained current scans

    /// Intermediate variables used during UI rendering
    SE3 T_map_odom_for_lio_traj_ui_;      // For displaying the LIO trajectory
    SE3 T_map_baselink_for_lio_traj_ui_;  // For displaying the LIO trajectory

    // Trajectory
    std::shared_ptr<ui::UiTrajectory> traj_lidarloc_ui_ = nullptr;
    std::shared_ptr<ui::UiTrajectory> traj_gps_ui_ = nullptr;

    // Filter state data loggers
    pangolin::DataLog log_vel_;           // Velocity in odom frame
    pangolin::DataLog log_vel_baselink_;  // Velocity in baselink frame
    pangolin::DataLog log_bias_acc_;
    pangolin::DataLog log_bias_gyr_;

    std::unique_ptr<pangolin::Plotter> plotter_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_vel_baselink_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_gyr_ = nullptr;
};

}  // namespace IncLIO::ui

#endif
