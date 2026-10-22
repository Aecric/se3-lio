#pragma once

// C++ standard libraries
#include <Eigen/Core>

#include <memory>

// Common
#include "common/data_type.h"
#include "common/utils.h"

// Core
#include "core/map_management.h"
#include "core/state_predict.h"
#include "core/state_update.h"

namespace se3_lio::pipeline {

struct SE3_LIO_Config {
    double acc_noise = 0.1;
    double gyr_noise = 0.1;
    double bg_noise = 0.0001;
    double ba_noise = 0.0001;

    double lidar_range_noise = 0.02;
    double lidar_angle_noise = 0.15;

    double downsample_resolution = 0.5;

    int max_iter = 4;

    double voxel_map_resolution = 1.0;
    int voxel_map_max_layer = 2;
    std::vector<int> voxel_map_layer_size = {5, 5, 5, 5, 5};
    int voxel_map_max_point_size = 1000;
    float voxel_map_plane_thres = 0.01;

    bool voxel_map_sliding_en = false;
    double voxel_map_sliding_thresh = 8.0;
    int voxel_map_half_size = 50;

    bool verbose = false;
};

class SE3_LIO {
public:
    explicit SE3_LIO(const SE3_LIO_Config &_config);
    SE3_LIO() : config_(){};

    void estimatePose(MeasurementPtr &_measurement_ptr);
    void estimatePoseWithGPS_v2(MeasurementPtr &_measurement_ptr);

    State getState() const { return state_; }

    /// False until the IMU is initialized and a scan has been registered.
    bool isValid() const { return state_predictor_valid_; }

    // ---- Fixed-map localization (run_loclite_online) ----
    /// Build the voxel map once from world-frame points; it is never updated.
    void setFixedMap(const std::vector<Eigen::Vector3d> &_points, double _point_cov);

    /// Restart the filter (IMU initialization included). A fixed map is kept,
    /// any other map is discarded.
    void resetOdometry();

    /// Start the odometry at T_world_imu instead of the identity. Takes effect
    /// once the IMU is initialized; gravity is rotated into the world frame.
    void setInitialPose(const Eigen::Matrix4d &_T_world_imu);

    size_t getMapSize() const { return map_manager_ ? map_manager_->mapSize() : 0; }

private:
    void setupFilter();

    SE3_LIO_Config config_;

    bool state_predictor_valid_ = false;
    bool has_initial_pose_ = false;
    bool initial_pose_applied_ = false;
    Eigen::Matrix4d initial_pose_ = Eigen::Matrix4d::Identity();

    MeasurementPtr meas_;
    State state_;

    se3_lio::StatePredict state_predictor_;
    se3_lio::StateUpdate state_updater_;
    std::shared_ptr<se3_lio::ManageMap> map_manager_;
};

}  // namespace se3_lio::pipeline