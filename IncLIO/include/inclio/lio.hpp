#ifndef INCLIO_LIO_HPP
#define INCLIO_LIO_HPP

#include "inclio/state.hpp"
#include "inclio/imu.hpp"
#include "inclio/imu_processor.hpp"
#include "inclio/measurement_sync.hpp"
#include "ieskf/ieskf.hpp"
#include "ndt/ndt_map.hpp"
#include "ndt/ndt_registration.hpp"
#include "utils/point_types.hpp"
#include "utils/math_utils.hpp"
#include "utils/logger.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <memory>
#include <vector>
#include <string>

namespace IncLIO {

// Forward declaration for optional UI
namespace ui { class PangolinWindow; }

/// Main LIO odometry pipeline
///
/// Data flow per scan:
///   1. Feed IMU and LiDAR data via AddIMU() and AddCloud()
///   2. MessageSync buffers and aligns them into MeasureGroups
///   3. On each synced group:
///      a. IMU forward propagation (predict)
///      b. Undistort point cloud using propagated poses
///      c. Downsample the cloud
///      d. IESKF iterated update using NDT observation model
///      e. Insert corrected cloud into NDT map
///      f. Output current pose

struct LIOConfig {
    IMUProcessorConfig imu_config;

    // NDT map options
    double ndt_voxel_size = 1.0;
    int ndt_min_pts_in_voxel = 5;
    int ndt_max_pts_in_voxel = 50;
    size_t ndt_capacity = 100000;

    // NDT registration options
    int ndt_max_iteration = 4;
    int ndt_min_effective_pts = 10;
    double ndt_res_outlier_th = 5.0;

    // IESKF options
    int ieskf_num_iterations = 3;
    double ieskf_quit_eps = 1e-3;

    // Scan processing
    double scan_voxel_size = 0.5;          // Downsample leaf size for scan
    double map_update_dist_th = 1.0;       // Min translation to add scan to map
    double map_update_angle_th_deg = 10.0; // Min rotation (deg) to add scan to map

    // IMU-LiDAR extrinsics
    SE3 T_imu_lidar;  // Transform from LiDAR frame to IMU frame

    // UI
    bool with_ui = false;
};

class LIO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LIO() = default;
    explicit LIO(const LIOConfig& config);
    ~LIO() = default;

    /// Initialize from a YAML config file
    bool Init(const std::string& config_yaml);

    /// Feed an IMU measurement (buffered by MessageSync)
    void AddIMU(IMUPtr imu);

    /// Feed a LiDAR scan (buffered by MessageSync, triggers processing when synced)
    void AddCloud(FullCloudPtr cloud, double timestamp);

    /// Get current pose estimate
    SE3 GetCurrentPose() const { return ieskf_.GetNominalSE3(); }

    /// Get full state
    Stated GetCurrentState() const { return ieskf_.GetNominalState(); }

    /// Get the current aligned scan (in world frame)
    CloudPtr GetCurrentScan() const { return current_scan_; }

    /// Whether IMU initialization is done
    bool IsInitialized() const { return !imu_need_init_; }

    /// Shut down (clean up UI if active)
    void Finish();

    // is Keyframe or not ?
    bool IsKeyframe(const SE3& current_pose);

   private:
    bool LoadFromYAML(const std::string& yaml_file);

    /// Called by MessageSync when a synced MeasureGroup is ready
    void ProcessMeasurements(const MeasureGroup& meas);

    /// Try IMU initialization from buffered measurements
    void TryInitIMU();

    /// Propagate IESKF state using buffered IMU data
    void Predict();

    /// Undistort the current scan using IMU-propagated poses
    void Undistort();

    /// Run NDT alignment and map update
    void Align();

    LIOConfig config_;

    // Modules
    std::shared_ptr<MessageSync> sync_ = nullptr;
    IMUProcessor imu_processor_;
    IESKFD ieskf_;
    NdtMap ndt_map_;
    NdtRegistration ndt_reg_;

    // Scan data
    FullCloudPtr scan_undistort_{new FullPointCloudType()};
    CloudPtr current_scan_ = nullptr;
    CloudPtr current_scan_world_ = nullptr;

    // State
    MeasureGroup measures_;
    std::vector<Stated> imu_states_;
    SE3 last_pose_;
    SE3 last_kf_pose_;
    bool imu_need_init_ = true;
    bool first_scan_ = true;
    int frame_num_ = 0;

    // Optional UI
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

} // namespace IncLIO

#endif // INCLIO_LIO_HPP
