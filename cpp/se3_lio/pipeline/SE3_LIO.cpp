#include "SE3_LIO.h"

#include <memory>

namespace se3_lio::pipeline {

SE3_LIO::SE3_LIO(const SE3_LIO_Config &_config) : config_(_config) { setupFilter(); }

void SE3_LIO::setupFilter() {
    state_predictor_ = se3_lio::StatePredict();
    state_predictor_.setIMUNoise(config_.gyr_noise, config_.acc_noise, config_.bg_noise,
                                 config_.ba_noise);
    state_predictor_.setLiDARNoise(config_.lidar_range_noise, config_.lidar_angle_noise);
    state_predictor_.setVerbose(config_.verbose);

    se3_lio::ManageMapConfig manage_map_config;
    manage_map_config.resolution = config_.voxel_map_resolution;
    manage_map_config.max_layer = config_.voxel_map_max_layer;
    manage_map_config.layer_size = config_.voxel_map_layer_size;
    manage_map_config.max_point_size = config_.voxel_map_max_point_size;
    manage_map_config.plane_thres = config_.voxel_map_plane_thres;
    manage_map_config.map_sliding_en = config_.voxel_map_sliding_en;
    manage_map_config.sliding_thresh = config_.voxel_map_sliding_thresh;
    manage_map_config.half_map_size = config_.voxel_map_half_size;
    manage_map_config.verbose = config_.verbose;

    if (!map_manager_ || !map_manager_->isFixed()) {
        map_manager_ = std::make_shared<se3_lio::ManageMap>(manage_map_config);
    }

    state_updater_ = se3_lio::StateUpdate(config_.max_iter);
    state_updater_.setManageMapConfig(manage_map_config);
    state_updater_.setMap(map_manager_);

    state_ = State();
    state_predictor_valid_ = false;
    initial_pose_applied_ = false;
}

void SE3_LIO::setFixedMap(const std::vector<Eigen::Vector3d> &_points, double _point_cov) {
    // A slide would evict the fixed map around the body.
    config_.voxel_map_sliding_en = false;
    map_manager_.reset();
    setupFilter();
    map_manager_->initFixedMap(_points, _point_cov);
}

void SE3_LIO::resetOdometry() {
    has_initial_pose_ = false;
    setupFilter();
}

void SE3_LIO::setInitialPose(const Eigen::Matrix4d &_T_world_imu) {
    initial_pose_ = _T_world_imu;
    has_initial_pose_ = true;
    initial_pose_applied_ = false;
}

void SE3_LIO::estimatePose(MeasurementPtr &_measurement_ptr) {
    if (has_initial_pose_ && !initial_pose_applied_ && state_predictor_.isInitialized()) {
        // The initial state lives in the IMU frame (gravity from the static
        // accelerometer mean); move it to the requested world pose before the
        // first propagation.
        state_.grav = initial_pose_.block<3, 3>(0, 0) * state_.grav;
        state_.pose = initial_pose_;
        initial_pose_applied_ = true;
    }

    state_predictor_.setState(state_);
    state_predictor_.setMeasurement(_measurement_ptr);
    state_ = state_predictor_.predictState();
    state_predictor_.undistortCloud();

    state_predictor_valid_ = state_predictor_.isValid();
    if (!state_predictor_valid_) return;

    _measurement_ptr->raw_lidar = _measurement_ptr->lidar;
    downsampleCloud(_measurement_ptr->lidar, config_.downsample_resolution);

    state_predictor_.calculateUndistCloudCov(_measurement_ptr->lidar);

    state_.num_inliers = 0;
    state_updater_.setState(state_);
    state_updater_.setMeasurement(_measurement_ptr);
    state_ = state_updater_.updateState();

    map_manager_->setState(state_);
    map_manager_->setMeasurement(_measurement_ptr->lidar);
    map_manager_->updateMap();
}

void SE3_LIO::estimatePoseWithGPS_v2(MeasurementPtr &_measurement_ptr) {
    state_predictor_.setState(state_);
    state_predictor_.setMeasurement(_measurement_ptr);
    state_ = state_predictor_.predictState();
    state_predictor_.undistortCloud();

    if (!state_predictor_.isValid()) return;

    downsampleCloud(_measurement_ptr->lidar, config_.downsample_resolution);

    state_predictor_.calculateUndistCloudCov(_measurement_ptr->lidar);

    state_updater_.setState(state_);
    state_updater_.setMeasurement(_measurement_ptr);
    state_ = state_updater_.updateState_v2();

    map_manager_->setState(state_);
    map_manager_->setMeasurement(_measurement_ptr->lidar);
    map_manager_->updateMap();
}

}  // namespace se3_lio::pipeline