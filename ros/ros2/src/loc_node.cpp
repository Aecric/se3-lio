#include "loc_node.h"

#include <pcl/filters/voxel_grid.h>

namespace se3_lio::ros2_node {

LocNode::LocNode(const rclcpp::NodeOptions &options) : Node("se3_lio_loc", options) {
    declareCoreParameters();

    // Localization surface (map, frames, outputs, reloc.*), same parameters as Super-LIO.
    auto params = loclite::LoadLocParams([this](const std::string &name, auto value) {
        return this->declare_parameter<decltype(value)>(name, value);
    });
    const double map_voxel = this->declare_parameter<double>("fixed_map.voxel_size", 0.1);
    const double map_point_cov = this->declare_parameter<double>("fixed_map.point_cov", 0.0004);
    if (se3_lio_config_.downsample_resolution <= 0 || se3_lio_config_.voxel_map_resolution <= 0 ||
        se3_lio_config_.max_iter < 1 || map_point_cov <= 0)
        throw std::runtime_error("Invalid localization sensor/filter parameters");

    const auto map = loclite::LoadFixedMap(params.map_file);
    const auto started = std::chrono::steady_clock::now();
    {
        // The fixed map is the one SE(3)-LIO tracks against: no update, no slide.
        loclite::CloudPtr ds = map;
        if (map_voxel > 0) {
            const float leaf = static_cast<float>(map_voxel);
            pcl::VoxelGrid<loclite::PointType> grid;
            grid.setLeafSize(leaf, leaf, leaf);
            grid.setInputCloud(map);
            ds.reset(new loclite::Cloud());
            grid.filter(*ds);
        }
        std::vector<Eigen::Vector3d> points;
        points.reserve(ds->size());
        for (const auto &p : ds->points) points.emplace_back(p.x, p.y, p.z);
        se3_lio_pipeline_ = se3_lio::pipeline::SE3_LIO(se3_lio_config_);
        se3_lio_pipeline_.setFixedMap(points, map_point_cov);
        RCLCPP_INFO(this->get_logger(), "Map init success: %zu points -> %zu voxel-map roots in %ld ms",
                    ds->size(), se3_lio_pipeline_.getMapSize(),
                    static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now() - started)
                                          .count()));
    }

    relocalizer_ = std::make_unique<loclite::Relocalizer>(params.reloc);
    if (!relocalizer_->SetMap(map)) throw std::runtime_error("Reloc target init failed");
    RCLCPP_INFO(this->get_logger(), "[Reloc]: targets ready; cold start %s",
                relocalizer_->ColdStartMode());

    synchronizer_ = std::make_unique<se3_lio::synchronizer::MeasurementSynchronizer>();
    loc_ = std::make_unique<loclite::LocInterface>(*this, std::move(params), [this](bool active) {
        if (active) {
            createSensorSubscriptions();
        } else {
            destroySensorSubscriptions();
        }
        // Each activation starts a fresh session on the same fixed map.
        restart_ = true;
    });
    loc_->PublishMap(*map);
    createSensorSubscriptions();
}

LocNode::~LocNode() {
    keep_process_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void LocNode::declareCoreParameters() {
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
    lidar_topic_ = this->declare_parameter<std::string>("lidar_topic", "/lidar/points");

    se3_lio_config_.acc_noise = this->declare_parameter<double>("sensors.imu.acc_cov", 0.1);
    se3_lio_config_.gyr_noise = this->declare_parameter<double>("sensors.imu.gyr_cov", 0.1);
    se3_lio_config_.bg_noise = this->declare_parameter<double>("sensors.imu.b_acc_cov", 0.0001);
    se3_lio_config_.ba_noise = this->declare_parameter<double>("sensors.imu.b_gyr_cov", 0.0001);

    lidar_min_range_ = this->declare_parameter<double>("sensors.lidar.min_range", 0.1);
    se3_lio_config_.lidar_range_noise = this->declare_parameter<double>("sensors.lidar.range_cov", 0.001);
    se3_lio_config_.lidar_angle_noise = this->declare_parameter<double>("sensors.lidar.angle_cov", 0.01);
    lidar_point_filter_num_ = this->declare_parameter<int>("sensors.lidar.point_filter_num", 1);
    if (lidar_point_filter_num_ < 1) lidar_point_filter_num_ = 1;

    const auto lidar_extrinsic_t =
        this->declare_parameter<std::vector<double>>("sensors.t_exts", {0.0, 0.0, 0.0});
    const auto lidar_extrinsic_q =
        this->declare_parameter<std::vector<double>>("sensors.q_exts", {1.0, 0.0, 0.0, 0.0});
    if (lidar_extrinsic_t.size() != 3 || lidar_extrinsic_q.size() != 4)
        throw std::runtime_error("sensors.t_exts needs 3 values and sensors.q_exts 4 [w, x, y, z]");

    se3_lio_config_.downsample_resolution = this->declare_parameter<double>("downsample.resolution", 0.5);
    se3_lio_config_.max_iter = this->declare_parameter<int>("max_iter", 4);
    se3_lio_config_.verbose = this->declare_parameter<bool>("verbose", false);

    se3_lio_config_.voxel_map_resolution = this->declare_parameter<double>("voxel_map.resolution", 1.0);
    se3_lio_config_.voxel_map_max_layer = this->declare_parameter<int>("voxel_map.max_layer", 2);
    const auto layer_size =
        this->declare_parameter<std::vector<int64_t>>("voxel_map.layer_size", {5, 5, 5, 5, 5});
    se3_lio_config_.voxel_map_max_point_size = this->declare_parameter<int>("voxel_map.max_point_size", 1000);
    se3_lio_config_.voxel_map_plane_thres =
        static_cast<float>(this->declare_parameter<double>("voxel_map.plane_threshold", 0.01));
    se3_lio_config_.voxel_map_layer_size.clear();
    for (const auto &size : layer_size) {
        se3_lio_config_.voxel_map_layer_size.push_back(static_cast<int>(size));
    }
    // A sliding window would evict the fixed map; localization never slides.
    se3_lio_config_.voxel_map_sliding_en = false;

    lidar_extrinsic_.block<3, 1>(0, 3) =
        Eigen::Vector3d(lidar_extrinsic_t[0], lidar_extrinsic_t[1], lidar_extrinsic_t[2]);
    Eigen::Quaterniond q_lidar_extrinsic(lidar_extrinsic_q[0], lidar_extrinsic_q[1],
                                         lidar_extrinsic_q[2], lidar_extrinsic_q[3]);
    lidar_extrinsic_.block<3, 3>(0, 0) = q_lidar_extrinsic.normalized().toRotationMatrix();
    std::cout << "LiDAR extrinsic: \n" << lidar_extrinsic_ << std::endl;
}

void LocNode::createSensorSubscriptions() {
    if (sub_imu_) return;
    // Same QoS as lio_node: sensor streams and `ros2 bag play` are best_effort.
    auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(10000)).best_effort();
    auto qos_lidar = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_imu_ = this->create_subscription<IMU_MSG_TYPE>(
        imu_topic_, qos_imu, std::bind(&LocNode::imuCallback, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<LIDAR_MSG_TYPE>(
        lidar_topic_, qos_lidar, std::bind(&LocNode::lidarCallback, this, std::placeholders::_1));
}

void LocNode::destroySensorSubscriptions() {
    sub_imu_.reset();
    sub_lidar_.reset();
}

void LocNode::imuCallback(const IMU_MSG_TYPE::SharedPtr _msg) {
    loc_->ImuReceived();
    std::lock_guard<std::mutex> lock(mutex_);
    imu_queue_.push(_msg);
}

void LocNode::lidarCallback(const LIDAR_MSG_TYPE::SharedPtr _msg) {
    loc_->LidarReceived();
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_queue_.push(_msg);
}

void LocNode::pushAllROSMessages() {
    std::lock_guard<std::mutex> lock(mutex_);
    synchronizer_->pause();
    while (!imu_queue_.empty()) {
        IMU imu = convertIMUMessage(imu_queue_.front());
        synchronizer_->addIMU(imu);
        imu_queue_.pop();
    }

    while (!lidar_queue_.empty()) {
#if defined(LIDAR_LIVOX)
        LiDAR lidar = convertLivoxMessage(lidar_queue_.front(), lidar_min_range_, lidar_point_filter_num_);
#else  // LIDAR_OUSTER
        LiDAR lidar = convertOusterMessage(lidar_queue_.front(), lidar_min_range_, lidar_point_filter_num_);
#endif
        if (!lidar.points.empty()) synchronizer_->addLiDAR(lidar);
        lidar_queue_.pop();
    }
    synchronizer_->resume();
}

void LocNode::run() {
    keep_process_ = true;
    thread_ = std::thread(&LocNode::process, this);
}

void LocNode::resetLocalization() {
    tracking_ = false;
    relocalizer_->Reset();
    se3_lio_pipeline_.resetOdometry();
    loc_->ClearPath();
    loc_->SetState(loclite::kWaitForInitialPose);
}

void LocNode::process() {
    rclcpp::Rate rate(2000);

    while (rclcpp::ok() && keep_process_) {
        if (restart_.exchange(false)) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                std::queue<IMU_MSG_TYPE::SharedPtr>().swap(imu_queue_);
                std::queue<LIDAR_MSG_TYPE::SharedPtr>().swap(lidar_queue_);
            }
            synchronizer_ = std::make_unique<se3_lio::synchronizer::MeasurementSynchronizer>();
            resetLocalization();
        }
        if (!loc_->Active()) {
            rate.sleep();
            continue;
        }

        // /initialpose (map <- lidar) interrupts a cold start or the current track.
        Eigen::Matrix4d T_map_lidar;
        if (loc_->TakeInitialPose(T_map_lidar)) {
            resetLocalization();
            relocalizer_->RequestManual(T_map_lidar * lidar_extrinsic_.inverse());
            loc_->SetState(loclite::kInitializing);
        }
        // Once tracking is lost, require a validated new initial pose.
        if (tracking_ && loc_->State() == loclite::kLost) resetLocalization();

        pushAllROSMessages();
        MeasurementPtr synced_measurement = synchronizer_->getSyncedMeasurement();
        if (!synced_measurement->is_synced) {
            rate.sleep();
            continue;
        }

        synced_measurement->lidar.points =
            transformPointCloud(synced_measurement->lidar.points, lidar_extrinsic_);
        std::sort(synced_measurement->lidar.points.begin(), synced_measurement->lidar.points.end(),
                  [](const CustomPointType &a, const CustomPointType &b) {
                      return a.timestamp < b.timestamp;
                  });

        if (tracking_) {
            track(synced_measurement);
        } else {
            initialize(synced_measurement);
        }
    }
}

void LocNode::initialize(const MeasurementPtr &_measurement) {
    loclite::Cloud scan;
    scan.reserve(_measurement->lidar.points.size());
    for (const auto &p : _measurement->lidar.points) {
        loclite::PointType q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        scan.push_back(q);
    }
    std::vector<Eigen::Vector3d> acc, gyr;
    for (const auto &imu : _measurement->imu) {
        acc.push_back(imu.linear_acceleration);
        gyr.push_back(imu.angular_velocity);
    }

    using Status = loclite::Relocalizer::Status;
    const Status status = relocalizer_->AddFrame(scan, acc, gyr);
    switch (status) {
        case Status::kIdle:
            break;
        case Status::kAccumulating:
            loc_->SetState(loclite::kInitializing);
            break;
        case Status::kFailed:
            loc_->ResetWatchdog();
            break;
        case Status::kGaveUp:
            loc_->ResetWatchdog();
            loc_->SetState(loclite::kWaitForInitialPose);
            break;
        case Status::kAccepted:
            loc_->ResetWatchdog();
            // The filter re-estimates gravity and gyro bias on the next scans
            // (device still static) and starts at the validated pose.
            se3_lio_pipeline_.resetOdometry();
            se3_lio_pipeline_.setInitialPose(relocalizer_->Result());
            tracking_ = true;
            loc_->SetState(loclite::kGood);
            break;
    }
}

void LocNode::track(MeasurementPtr &_measurement) {
    se3_lio_pipeline_.estimatePose(_measurement);
    if (!se3_lio_pipeline_.isValid()) return;

    const se3_lio::State state = se3_lio_pipeline_.getState();
    if (!state.pose.allFinite() || !state.vel.allFinite() || state.num_inliers < 10) {
        RCLCPP_WARN(this->get_logger(), "[Track] lost: %d matched points%s", state.num_inliers,
                    state.pose.allFinite() ? "" : ", non-finite pose");
        loc_->SetState(loclite::kLost);
        return;
    }
    loc_->SetState(loclite::kGood);
    // SE(3)-LIO keeps the velocity in the IMU frame.
    loc_->Publish(state.stamp, state.pose, lidar_extrinsic_, state.rot() * state.vel);
}

}  // namespace se3_lio::ros2_node
