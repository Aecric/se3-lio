#pragma once

// Fixed-map pure localization: SE(3)-LIO tracks an existing global.pcd without
// updating or saving it. Interface and initialization follow Super-LIO's
// run_loclite_online (see docs/localization.md).

// C++ standard library
#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// SE3-LIO::ROS2 Node
#include "ros2_conversion.h"

// Fixed-map localization
#include "loclite/loc_interface.h"
#include "loclite/relocalizer.h"

// SE3-LIO
#include "common/utils.h"
#include "pipeline/SE3_LIO.h"
#include "synchronizer/measurement_synchronizer.h"

namespace se3_lio::ros2_node {

using IMU_MSG_TYPE = sensor_msgs::msg::Imu;

// LiDAR input type is selected at build time via CMake (-DLIDAR_TYPE=ouster|livox).
#if defined(LIDAR_LIVOX)
using LIDAR_MSG_TYPE = livox_ros_driver2::msg::CustomMsg;
#else  // LIDAR_OUSTER
using LIDAR_MSG_TYPE = sensor_msgs::msg::PointCloud2;
#endif

class LocNode : public rclcpp::Node {
public:
    /// Throws std::runtime_error on a missing/empty map or invalid parameters.
    explicit LocNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LocNode();

    void run();

private:
    void declareCoreParameters();
    void createSensorSubscriptions();
    void destroySensorSubscriptions();

    void imuCallback(const IMU_MSG_TYPE::SharedPtr _msg);
    void lidarCallback(const LIDAR_MSG_TYPE::SharedPtr _msg);

    void pushAllROSMessages();
    void process();

    /// Super-LIO SuperLIOReLoc::Reset(): drop the track, keep the fixed map.
    void resetLocalization();
    void initialize(const MeasurementPtr &_measurement);
    void track(MeasurementPtr &_measurement);

    std::mutex mutex_;
    std::thread thread_;
    std::atomic<bool> keep_process_{false};
    std::atomic<bool> restart_{false};

    rclcpp::Subscription<IMU_MSG_TYPE>::SharedPtr sub_imu_;
    rclcpp::Subscription<LIDAR_MSG_TYPE>::SharedPtr sub_lidar_;
    std::string imu_topic_, lidar_topic_;

    std::queue<IMU_MSG_TYPE::SharedPtr> imu_queue_;
    std::queue<LIDAR_MSG_TYPE::SharedPtr> lidar_queue_;

    std::unique_ptr<se3_lio::synchronizer::MeasurementSynchronizer> synchronizer_;

    se3_lio::pipeline::SE3_LIO se3_lio_pipeline_;
    se3_lio::pipeline::SE3_LIO_Config se3_lio_config_;

    Eigen::Matrix4d lidar_extrinsic_ = Eigen::Matrix4d::Identity();  // p_imu = T * p_lidar

    double lidar_min_range_ = 0.1;
    int lidar_point_filter_num_ = 1;

    std::unique_ptr<loclite::LocInterface> loc_;
    std::unique_ptr<loclite::Relocalizer> relocalizer_;
    bool tracking_ = false;
};

}  // namespace se3_lio::ros2_node
