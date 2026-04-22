#include "inclio/lio.hpp"

#include <chrono>
#include <numeric>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <execution>

#ifdef BUILD_TOOLS
#include "tools/ui/pangolin_window.h"
#endif

namespace IncLIO {

LIO::LIO(const LIOConfig& config) : config_(config) {
    IMUProcessorConfig imu_cfg = config.imu_config;
    imu_cfg.use_speed_for_static_checking = false;
    imu_processor_ = IMUProcessor(imu_cfg);

    // NDT map
    NdtMap::Options map_opts;
    map_opts.voxel_size = config.ndt_voxel_size;
    map_opts.min_pts_in_voxel = config.ndt_min_pts_in_voxel;
    map_opts.max_pts_in_voxel = config.ndt_max_pts_in_voxel;
    map_opts.capacity = config.ndt_capacity;
    ndt_map_ = NdtMap(map_opts);

    // NDT registration
    NdtRegistration::Options reg_opts;
    reg_opts.max_iteration = config.ndt_max_iteration;
    reg_opts.min_effective_pts = config.ndt_min_effective_pts;
    reg_opts.res_outlier_th = config.ndt_res_outlier_th;
    ndt_reg_ = NdtRegistration(reg_opts);
    ndt_reg_.SetNdtMap(&ndt_map_);

    current_scan_world_ = CloudPtr(new PointCloudType);

    // MessageSync — fires ProcessMeasurements when a synced group is ready
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup& m) { ProcessMeasurements(m); });
    if(SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG) {
        InitLogger(spdlog::level::debug);
    }
    else if (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_INFO) {
        InitLogger(spdlog::level::info);
    }
}

bool LIO::Init(const std::string& config_yaml) {
    if (!LoadFromYAML(config_yaml)) {
        INCLIO_ERROR("LIO init failed");
        return false;
    }

    sync_->Init(config_yaml);
#ifdef BUILD_TOOLS
    if (config_.with_ui) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();
    }
#endif
    return true;
}

bool LIO::LoadFromYAML(const std::string& yaml_file) {
    auto yaml = YAML::LoadFile(yaml_file);

    // IMU-LiDAR extrinsics
    if (yaml["mapping"]) {
        auto ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        auto ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
        Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
        Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
        config_.T_imu_lidar = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    }

    return true;
}

void LIO::AddIMU(IMUPtr imu) {
    sync_->ProcessIMU(imu);

    // High-rate forward propagation (kinematics only, no covariance).
    // This runs on every IMU arrival so GetPropagatedState() returns an
    // up-to-date pose at IMU rate. The real IESKF is not touched here.
    if (!imu_need_init_) {
        std::lock_guard<std::mutex> lock(prop_state_mutex_);
        double dt = imu->timestamp_ - prop_state_.timestamp_;
        if (dt > 0 && dt < 0.1) {
            Vec3d acce = imu->acce_;
            Vec3d gyro = imu->gyro_;
            SO3 R = prop_state_.R_;

            prop_state_.p_ += prop_state_.v_ * dt
                            + 0.5 * (R * (acce - prop_state_.ba_)) * dt * dt
                            + 0.5 * prop_gravity_ * dt * dt;
            prop_state_.v_ += R * (acce - prop_state_.ba_) * dt
                            + prop_gravity_ * dt;
            prop_state_.R_ = R * SO3::exp((gyro - prop_state_.bg_) * dt);
            prop_state_.timestamp_ = imu->timestamp_;
        }
    }
}

void LIO::AddCloud(FullCloudPtr cloud, double timestamp) {
    // MessageSync handles buffering and calls ProcessMeasurements when synced
    sync_->ProcessCloud(cloud, timestamp);
}

void LIO::ProcessMeasurements(const MeasureGroup& meas) {
    INCLIO_DEBUG("ProcessMeasurements: imu={}, lidar pts={}", meas.imu_.size(), meas.lidar_->size());
    measures_ = meas;

    if (imu_need_init_) {
        TryInitIMU();
        return;
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    Predict();
    auto t1 = std::chrono::high_resolution_clock::now();
    Undistort();
    auto t2 = std::chrono::high_resolution_clock::now();
    Align();
    auto t3 = std::chrono::high_resolution_clock::now();

    // Reset the high-rate propagated state to the NDT-corrected estimate.
    // From here, each incoming IMU in AddIMU() will propagate forward from this anchor.
    {
        std::lock_guard<std::mutex> lock(prop_state_mutex_);
        prop_state_ = ieskf_.GetNominalState();
        prop_gravity_ = ieskf_.GetGravity();
    }

    INCLIO_DEBUG("Frame {}: Predict {:.3f} ms | Undistort {:.3f} ms | Align {:.3f} ms | Total {:.3f} ms",
        frame_num_,
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        std::chrono::duration<double, std::milli>(t2 - t1).count(),
        std::chrono::duration<double, std::milli>(t3 - t2).count(),
        std::chrono::duration<double, std::milli>(t3 - t0).count());
}

void LIO::TryInitIMU() {
    for (const auto& imu : measures_.imu_) {
        imu_processor_.AddIMUForInit(*imu);
    }

    if (imu_processor_.InitSuccess()) {
        IESKFD::Options ieskf_opts;
        ieskf_opts.num_iterations_ = config_.ieskf_num_iterations;
        ieskf_opts.quit_eps_ = config_.ieskf_quit_eps;
        ieskf_opts.gyro_var_ = std::sqrt(imu_processor_.GetCovGyro()[0]);
        ieskf_opts.acce_var_ = std::sqrt(imu_processor_.GetCovAcce()[0]);
        ieskf_opts.imu_dt_ = imu_processor_.GetIMUDt();

        // Compute initial rotation to align body frame with gravity-aligned world frame.
        // The IMU processor gives gravity in the body frame; we need R_ such that
        // R_ * body_gravity = world_gravity, i.e. R_ rotates body to world.
        Vec3d body_gravity = imu_processor_.GetGravity();
        Vec3d world_gravity(0, 0, -imu_processor_.GetGravity().norm());
        Eigen::Quaterniond q_init = Eigen::Quaterniond::FromTwoVectors(
            body_gravity, world_gravity);
        SO3 init_R(q_init);

        ieskf_.SetInitialConditions(ieskf_opts,
                                    imu_processor_.GetInitBg(),
                                    imu_processor_.GetInitBa(),
                                    world_gravity);
        ieskf_.SetR(init_R);

        // Seed current_time_ so the first Predict() dt is ~imu_dt_, not ~1.775e9 s.
        if (!measures_.imu_.empty())
            ieskf_.SetCurrentTime(measures_.imu_.back()->timestamp_);

        imu_need_init_ = false;
        INCLIO_INFO("IMU initialization successful, init_R ypr: [{:.2f}, {:.2f}, {:.2f}] deg",
                    init_R.log()[2] * 180.0 / M_PI,
                    init_R.log()[1] * 180.0 / M_PI,
                    init_R.log()[0] * 180.0 / M_PI);
    }
}

void LIO::Predict() {
    imu_states_.clear();
    imu_a_world_.clear();
    imu_w_body_.clear();
    imu_states_.emplace_back(ieskf_.GetNominalState());

    const Vec3d g = ieskf_.GetGravity();
    for (const auto& imu : measures_.imu_) {
        const Stated& s = imu_states_.back();
        imu_a_world_.push_back(s.R_ * (imu->acce_ - s.ba_) + g);
        imu_w_body_.push_back(imu->gyro_ - s.bg_);
        ieskf_.Predict(*imu);
        imu_states_.emplace_back(ieskf_.GetNominalState());
    }
}

void LIO::Undistort() {
    auto cloud = measures_.lidar_;
    auto imu_state = ieskf_.GetNominalState();
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);
    const SE3& TIL = config_.T_imu_lidar;

    // Build CT trajectory once outside the parallel loop (read-only inside).
    std::vector<CTSegment> ct_segs;
    if (config_.use_ct_undistort)
        ct_segs = BuildCTSegments(imu_states_, imu_a_world_, imu_w_body_);

    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto& pt) {
        const double t_pt = measures_.lidar_begin_time_ + pt.time * 1e-3;
        SE3 Ti;

        if (config_.use_ct_undistort) {
            Ti = CTQueryPose(t_pt, ct_segs);
        } else {
            // Original slerp/lerp interpolation — kept as fallback
            Stated match;
            math::PoseInterp<Stated>(
                t_pt, imu_states_,
                [](const Stated& s) { return s.timestamp_; },
                [](const Stated& s) { return s.GetSE3(); },
                Ti, match);
        }

        Vec3d pi = ToVec3d(pt);
        Vec3d p_compensate = T_end.inverse() * Ti * TIL * pi;

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);
    });

    scan_undistort_ = cloud;
}

void LIO::Align() {
    const SE3& TIL = config_.T_imu_lidar;

    // Transform undistorted scan from LiDAR frame to IMU frame
    // FullCloudPtr scan_imu_frame(new FullPointCloudType);
    // pcl::transformPointCloud(*scan_undistort_, *scan_imu_frame, TIL.matrix().cast<float>());

    // Convert to simple point cloud and downsample
    // #pragma omp parallel for
    // current_scan_.reset(new PointCloudType);
    // for (const auto& pt : scan_undistort_->points) {
    //     PointType p;
    //     p.x = pt.x; p.y = pt.y; p.z = pt.z;
    //     p.intensity = pt.intensity;
    //     current_scan_->push_back(p);
    // }
    // current_scan_->width = current_scan_->size();
    // current_scan_->height = 1;
    // current_scan_->is_dense = true;

    current_scan_.reset(new PointCloudType);

    // Pre-allocate memory to avoid push_back race conditions
    current_scan_->points.resize(scan_undistort_->points.size());

    #pragma omp parallel for
    for (size_t i = 0; i < scan_undistort_->points.size(); ++i) {
        const auto& pt = scan_undistort_->points[i];  // read-only, safe
        auto& p = current_scan_->points[i];          // each thread writes to a unique element
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;              
    }

    current_scan_->width = current_scan_->size();
    current_scan_->height = 1;
    current_scan_->is_dense = true;

    // Parallel voxel grid downsampling (replaces single-threaded pcl::VoxelGrid)
    std::vector<int> pt_index(current_scan_->points.size());
    std::iota(pt_index.begin(), pt_index.end(), 0);
    VoxelHashMap voxel_map;
    double inv_voxel = 1.0 / config_.scan_voxel_size;

    // Pass 1: bin all points concurrently
    std::for_each(std::execution::par_unseq, pt_index.begin(), pt_index.end(), [&](int i) {
        const auto& pt = current_scan_->points[i];
        Vec3i key = CastToInt(Vec3d(pt.x * inv_voxel, pt.y * inv_voxel, pt.z * inv_voxel));

        VoxelHashMap::accessor acc;
        voxel_map.insert(acc, key);
        acc->second.sx += pt.x;
        acc->second.sy += pt.y;
        acc->second.sz += pt.z;
        acc->second.si += pt.intensity;
        acc->second.count++;
    });

    // Pass 2: extract centroids in parallel
    std::vector<VoxelHashMap::const_iterator> voxel_iters;
    voxel_iters.reserve(voxel_map.size());
    for (auto it = voxel_map.begin(); it != voxel_map.end(); ++it) {
        voxel_iters.push_back(it);
    }

    CloudPtr current_scan_filtered(new PointCloudType);
    current_scan_filtered->points.resize(voxel_iters.size());

    std::vector<int> vox_index(voxel_iters.size());
    std::iota(vox_index.begin(), vox_index.end(), 0);

    std::for_each(std::execution::par_unseq, vox_index.begin(), vox_index.end(), [&](int i) {
        const auto& v = voxel_iters[i]->second;
        double inv_n = 1.0 / v.count;
        auto& p = current_scan_filtered->points[i];
        p.x = static_cast<float>(v.sx * inv_n);
        p.y = static_cast<float>(v.sy * inv_n);
        p.z = static_cast<float>(v.sz * inv_n);
        p.intensity = static_cast<float>(v.si * inv_n);
    });

    current_scan_filtered->width = current_scan_filtered->size();
    current_scan_filtered->height = 1;
    current_scan_filtered->is_dense = true;

    // First scan — just add to map
    if (first_scan_) {
        ndt_map_.AddCloud(current_scan_);
        first_scan_ = false;
        return;
    }

    INCLIO_DEBUG("=== frame {}", frame_num_);

    // IESKF update using NDT observation model
    ndt_reg_.SetSource(current_scan_filtered);
    ieskf_.UpdateUsingCustomObserve([this](const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
        ndt_reg_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
    });

    // Add scan to NDT map if moved enough
    SE3 current_pose = ieskf_.GetNominalSE3();
    SE3 delta_pose = last_pose_.inverse() * current_pose;

    last_was_keyframe_ = false;
    if (delta_pose.translation().norm() > config_.map_update_dist_th ||
        delta_pose.so3().log().norm() > math::deg2rad(config_.map_update_angle_th_deg)) {
        INCLIO_DEBUG("=== delta_pose.translation().norm() {}", delta_pose.translation().norm());
        IncLIO::transformCloudOMP(*current_scan_filtered, *current_scan_world_, current_pose.matrix().cast<float>());
        ndt_map_.AddCloud(current_scan_world_);
        last_pose_ = current_pose;
        last_was_keyframe_ = true;
    }

#ifdef BUILD_TOOLS
    if (ui_) {
        auto nav_state = ieskf_.GetNominalState();
        ui_->UpdateScan(current_scan_, nav_state.GetSE3());
        ui_->UpdateNavState(nav_state.GetSE3(), nav_state.v_, nav_state.ba_, nav_state.bg_);
    }
#endif

    frame_num_++;
}

bool LIO::IsKeyframe(const SE3& current_pose) {
    SE3 delta = last_kf_pose_.inverse() * current_pose;
    bool is_kf = delta.translation().norm() > config_.map_update_dist_th ||
                  delta.so3().log().norm() > math::deg2rad(config_.map_update_angle_th_deg);
    if(is_kf)
    {
        INCLIO_DEBUG("=== keyframe detected, frame_num_ {}", frame_num_);
        last_kf_pose_ = current_pose;
        return true;
    }
    return is_kf;
}



void LIO::Finish() {
#ifdef BUILD_TOOLS
    if (ui_) {
        ui_->Quit();
    }
#endif
    INCLIO_INFO("LIO finished, total frames: {}", frame_num_);
}

} // namespace IncLIO
