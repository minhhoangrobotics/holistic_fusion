/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph_ros2/SmbEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>

// Project
#include "smb_estimator_graph_ros2/SmbStaticTransforms.h"

// Workspace
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros2/util/conversions.h"
#include "smb_estimator_graph_ros2/constants.h"

namespace smb_se {

namespace {
Eigen::Isometry3d interpolatePose(const Eigen::Isometry3d& first, const Eigen::Isometry3d& second, const double fraction) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = first.translation() + fraction * (second.translation() - first.translation());
  result.linear() = Eigen::Quaterniond(first.rotation()).slerp(
      fraction, Eigen::Quaterniond(second.rotation())).normalized().toRotationMatrix();
  return result;
}
}  // namespace

SmbEstimator::SmbEstimator(std::shared_ptr<rclcpp::Node>& node) : graph_msf::GraphMsfRos2(node) {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Constructor called." << COLOR_END << std::endl;

  staticTransformsPtr_ = std::make_shared<SmbStaticTransforms>(node_);
  // Call setup after declaring parameters
  SmbEstimator::setup();
}

void SmbEstimator::setup() {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Setup called." << COLOR_END << std::endl;
  RCLCPP_INFO(node_->get_logger(),
              "Fusion timing: LiDAR and leg poses interpolated at graph keys; velocity evaluated near interval midpoint");

  // Boolean flags
  node_->declare_parameter("sensor_params.useLioOdometry", false);
  node_->declare_parameter("sensor_params.useLioVelocityFactor", false);
  node_->declare_parameter("sensor_params.useWheelOdometryBetween", false);
  node_->declare_parameter("sensor_params.useWheelLinearVelocities", false);
  node_->declare_parameter("sensor_params.useVioOdometry", false);

  // Sensor parameters (int)
  node_->declare_parameter("sensor_params.lioOdometryRate", 0);
  node_->declare_parameter("sensor_params.wheelOdometryBetweenRate", 0);
  node_->declare_parameter("sensor_params.wheelLinearVelocitiesRate", 0);
  node_->declare_parameter("sensor_params.vioOdometryRate", 0);

  // Alignment parameters (vector of double)
  node_->declare_parameter("alignment_params.initialSe3AlignmentNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Noise parameters (vectors of double)
  node_->declare_parameter("noise_params.lioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.lioVelocityLocalNoiseDensity", std::vector<double>{0.3, 0.3, 0.4});
  node_->declare_parameter("noise_params.wheelPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.wheelLinearVelocitiesNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.vioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Extrinsic frames (string)
  node_->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelOdometryBetweenFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelLinearVelocityLeftFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelLinearVelocityRightFrame", std::string(""));
  node_->declare_parameter("extrinsics.vioOdometryFrame", std::string(""));

  // Wheel Radius (double)
  node_->declare_parameter("sensor_params.wheelRadius", 0.0);
  lioLegMaxHorizontalError_ = node_->declare_parameter<double>("sensor_params.lioLegMaxHorizontalError", 1.2);
  useLioLegConsistencyGate_ = node_->declare_parameter<bool>("sensor_params.useLioLegConsistencyGate", true);
  lioLegMaxVerticalError_ = node_->declare_parameter<double>("sensor_params.lioLegMaxVerticalError", 0.6);
  lioLegMaxYawError_ = node_->declare_parameter<double>("sensor_params.lioLegMaxYawError", 0.6);

  SmbEstimator::readParams();

  // Initialize ROS 2 publishers and subscribers
  SmbEstimator::initializePublishers();
  SmbEstimator::initializeSubscribers();
  SmbEstimator::initializeMessages();
  SmbEstimator::initializeServices();

  GraphMsfRos2::setup(staticTransformsPtr_);

  // Transforms
  staticTransformsPtr_->findTransformations();

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void SmbEstimator::initializePublishers() {
  pubMeasMapLioPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_path_map_imu", SMB_QUEUE_SIZE);
  pubMeasMapVioPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measVIO_path_map_imu", SMB_QUEUE_SIZE);
}

void SmbEstimator::initializeSubscribers() {
  if (useLioOdometryFlag_) {
    subLioOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", SMB_QUEUE_SIZE, std::bind(&SmbEstimator::lidarOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic" << std::endl;
  }

  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry_topic", SMB_QUEUE_SIZE, std::bind(&SmbEstimator::wheelOdometryPoseCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: /wheel_odometry_topic" << std::endl;
  }

  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocities_topic", SMB_QUEUE_SIZE, std::bind(&SmbEstimator::wheelLinearVelocitiesCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: /wheel_velocities_topic" << std::endl;
  }

  if (useVioOdometryFlag_) {
    subVioOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odometry_topic", SMB_QUEUE_SIZE, std::bind(&SmbEstimator::vioOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: /vio_odometry_topic" << std::endl;
  }
}

void SmbEstimator::initializeMessages() {
  measLio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measVio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
}

void SmbEstimator::initializeServices() {
  // Nothing for now
}

void SmbEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr) {
  if (graph_msf::GraphMsf::areRollAndPitchInited() && !graph_msf::GraphMsf::areYawAndPositionInited() && !useLioOdometryFlag_ &&
      !useWheelOdometryBetweenFlag_ && !useWheelLinearVelocitiesFlag_ && !useVioOdometryFlag_) {
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;

    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "IMU_init_6D", int(graphConfigPtr_->imuRate_), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->getImuFrame() + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
        imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9, 1.0, Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 1));

    graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    graph_msf::GraphMsf::pretendFirstMeasurementReceived();
  }

  graph_msf::GraphMsfRos2::imuCallback(imuPtr);
}

void SmbEstimator::lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& odomLidarPtr) {
  static int lidarOdometryCallbackCounter__ = -1;
  static double lastLidarOdometryTime = -std::numeric_limits<double>::infinity();

  Eigen::Isometry3d lio_T_M_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.sec + odomLidarPtr->header.stamp.nanosec * 1e-9;
  if (!std::isfinite(lidarOdometryTimeK) || !lio_T_M_Lk.matrix().allFinite() ||
      lidarOdometryTimeK <= lastLidarOdometryTime) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Ignoring invalid or out-of-order LiDAR odometry");
    return;
  }
  lastLidarOdometryTime = lidarOdometryTimeK;
  ++lidarOdometryCallbackCounter__;

  const std::string& lioOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  }

  if (useLioLegConsistencyGate_) {
    Eigen::Isometry3d legPose;
    if (!legPoseAt_(lidarOdometryTimeK, legPose)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Waiting for leg odometry near LiDAR stamp; LiDAR factor held");
      return;
    }
    if (!lioLegReferenceReady_) {
      lioFromLeg_ = lio_T_M_Lk * legPose.inverse();
      lioLegReferenceReady_ = true;
    } else {
      const Eigen::Isometry3d error = (lioFromLeg_ * legPose).inverse() * lio_T_M_Lk;
      const double horizontalError = error.translation().head<2>().norm();
      const double verticalError = std::abs(error.translation().z());
      const double yawError = std::abs(std::atan2(error.rotation()(1, 0), error.rotation()(0, 0)));
      if (horizontalError > lioLegMaxHorizontalError_ ||
          verticalError > lioLegMaxVerticalError_ ||
          yawError > lioLegMaxYawError_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Rejecting drifting KISS pose: leg disagreement xy=%.2f m, z=%.2f m, yaw=%.2f rad",
                             horizontalError, verticalError, yawError);
        return;
      }
    }
    // Compare motion since the last accepted LiDAR pose. A fixed initial
    // LiDAR-to-leg anchor turns normal long-term leg drift into a permanent
    // LiDAR rejection after the robot has walked across the map.
    lioFromLeg_ = lio_T_M_Lk * legPose.inverse();
  }

  // A graph key may be tens of milliseconds away from the LiDAR stamp.
  // Interpolate the measured trajectory to an existing key inside the last
  // scan interval, rather than assigning the current pose to a different time.
  double poseFactorTime = lidarOdometryTimeK;
  Eigen::Isometry3d poseAtKey = lio_T_M_Lk;
  bool havePoseAtKey = true;
  if (isGraphInited()) {
    havePoseAtKey = lastLioVelocityTime_ > 0.0 &&
        graphMgrPtr_->getGraphTimeAtOrBefore(lidarOdometryTimeK, poseFactorTime) &&
        poseFactorTime > lastLioVelocityTime_ && poseFactorTime <= lidarOdometryTimeK;
    if (havePoseAtKey) {
      poseAtKey = interpolatePose(lastLioVelocityPose_, lio_T_M_Lk,
          (poseFactorTime - lastLioVelocityTime_) / (lidarOdometryTimeK - lastLioVelocityTime_));
    }
  }
  if (havePoseAtKey) {
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
        "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::Huber(2.0), poseFactorTime, 1.0, poseAtKey, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
        staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);
    if (areYawAndPositionInited()) {
      this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
    } else {
      this->initYawAndPosition(unary6DMeasurement);
    }
  }

  // A LiDAR pose factor corrects position but leaves the high-rate IMU
  // propagation free to acquire a false translational velocity during
  // in-place turns. Constrain that velocity using the same time-stamped
  // LiDAR motion, expressed in the base sensor frame.
  if (useLioVelocityFactor_ && lastLioVelocityTime_ > 0.0) {
    const double dt = lidarOdometryTimeK - lastLioVelocityTime_;
    double velocityFactorTime;
    // A finite difference measures average velocity around the interval's
    // midpoint. Its local frame must also be evaluated at the selected key.
    if (dt >= 0.05 && dt <= 0.25 &&
        graphMgrPtr_->getGraphTimeAtOrBefore(0.5 * (lastLioVelocityTime_ + lidarOdometryTimeK), velocityFactorTime) &&
        velocityFactorTime >= lastLioVelocityTime_) {
      const Eigen::Isometry3d velocityFrame = interpolatePose(lastLioVelocityPose_, lio_T_M_Lk,
          (velocityFactorTime - lastLioVelocityTime_) / dt);
      const Eigen::Vector3d velocityLocal = velocityFrame.rotation().transpose() *
          (lio_T_M_Lk.translation() - lastLioVelocityPose_.translation()) / dt;
      graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> velocityMeasurement(
          "Lidar_local_velocity_3D", int(lioOdometryRate_), lioOdometryFrame,
          lioOdometryFrame + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::Huber(2.0), velocityFactorTime, 1.0,
          velocityLocal, lioVelocityLocalNoise_);
      this->addUnaryVelocity3LocalMeasurement(velocityMeasurement);
    }
  }
  lastLioVelocityPose_ = lio_T_M_Lk;
  lastLioVelocityTime_ = lidarOdometryTimeK;

  addToPathMsg(measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id, odomLidarPtr->header.stamp,
               (lio_T_M_Lk * staticTransformsPtr_->rv_T_frame1_frame2(lioOdometryFrame, staticTransformsPtr_->getImuFrame()).matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4);

  pubMeasMapLioPath_->publish(*measLio_mapImuPathPtr_);
}

bool SmbEstimator::legPoseAt_(double stamp, Eigen::Isometry3d& pose) {
  std::lock_guard<std::mutex> lock(recentLegOdometryMutex_);
  if (recentLegOdometry_.empty()) return false;
  nav_msgs::msg::Odometry::ConstSharedPtr previous;
  double previousTime = 0.0;
  for (const auto& current : recentLegOdometry_) {
    const auto& t = current->header.stamp;
    const double currentTime = t.sec + t.nanosec * 1e-9;
    if (std::abs(currentTime - stamp) < 1e-6) {
      graph_msf::odomMsgToEigen(*current, pose.matrix());
      return pose.matrix().allFinite();
    }
    if (currentTime > stamp) {
      if (!previous || currentTime - previousTime > 0.20) return false;
      Eigen::Isometry3d first, second;
      graph_msf::odomMsgToEigen(*previous, first.matrix());
      graph_msf::odomMsgToEigen(*current, second.matrix());
      pose = interpolatePose(first, second, (stamp - previousTime) / (currentTime - previousTime));
      return pose.matrix().allFinite();
    }
    previous = current;
    previousTime = currentTime;
  }
  return false;
}

void SmbEstimator::wheelOdometryPoseCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& wheelOdometryKPtr) {
  if (!std::isfinite(wheelOdometryKPtr->pose.covariance[0]) ||
      wheelOdometryKPtr->pose.covariance[0] < 0.0) {
    // No verified support foot. Reset the between-factor baseline so a later
    // contact cannot bridge an airborne or pushed interval.
    wheelOdometryCallbackCounter_ = -1;
    std::lock_guard<std::mutex> lock(recentLegOdometryMutex_);
    recentLegOdometry_.clear();
    return;
  }
  Eigen::Isometry3d T_O_Bw_k;
  graph_msf::odomMsgToEigen(*wheelOdometryKPtr, T_O_Bw_k.matrix());
  double wheelOdometryTimeK = wheelOdometryKPtr->header.stamp.sec + wheelOdometryKPtr->header.stamp.nanosec * 1e-9;
  if (!std::isfinite(wheelOdometryTimeK) || !T_O_Bw_k.matrix().allFinite()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Ignoring non-finite leg odometry");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(recentLegOdometryMutex_);
    recentLegOdometry_.push_back(wheelOdometryKPtr);
    while (!recentLegOdometry_.empty()) {
      const auto& oldest = recentLegOdometry_.front()->header.stamp;
      if (wheelOdometryTimeK - (oldest.sec + oldest.nanosec * 1e-9) <= 5.0) break;
      recentLegOdometry_.pop_front();
    }
  }
  if (!areRollAndPitchInited()) return;

  if (isGraphInited()) {
    double graphTime;
    if (!graphMgrPtr_->getGraphTimeAtOrBefore(wheelOdometryTimeK, graphTime) ||
        !legPoseAt_(graphTime, T_O_Bw_k)) return;
    wheelOdometryTimeK = graphTime;
  }

  ++wheelOdometryCallbackCounter_;

  if (wheelOdometryCallbackCounter_ == 0) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }
  if (wheelOdometryTimeK <= wheelOdometryTimeKm1_) {
    // Many 100 Hz leg samples share the same 20 Hz graph key.
    return;
  }
  if (wheelOdometryTimeK - wheelOdometryTimeKm1_ > 0.20) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }

  const std::string& wheelOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryBetweenFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelOdometryBetweenRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), wheelOdometryTimeK, 1.0, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    }
  } else if (wheelOdometryTimeK - wheelOdometryTimeKm1_ >=
             1.0 / std::max(1.0, wheelOdometryBetweenRate_)) {
    Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Wheel_odometry_6D", int(wheelOdometryBetweenRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::Huber(2.0), wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
    this->addBinaryPose3Measurement(delta6DMeasurement);

    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
  }
}

void SmbEstimator::wheelLinearVelocitiesCallback_(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& wheelsSpeedsPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK = wheelsSpeedsPtr->data[0];
  const double leftWheelSpeedRps = wheelsSpeedsPtr->data[1];
  const double rightWheelSpeedRps = wheelsSpeedsPtr->data[2];
  const double leftWheelSpeedMs = leftWheelSpeedRps * wheelRadiusMeter_;
  const double rightWheelSpeedMs = rightWheelSpeedRps * wheelRadiusMeter_;

  const std::string& wheelLinearVelocityLeftFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityLeftFrame();
  const std::string& wheelLinearVelocityRightFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityRightFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_ && !useWheelOdometryBetweenFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
          wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), timeK, 1.0,
          Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    }
  } else {
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
        wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
        Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
        wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
        Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);
  }
}

void SmbEstimator::vioOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr) {
  std::cout << "VIO odometry not yet stable enough for usage, disable flag." << std::endl;

  Eigen::Isometry3d vio_T_M_Ck;
  graph_msf::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  addToPathMsg(measVio_mapImuPathPtr_, vioOdomPtr->header.frame_id, vioOdomPtr->header.stamp,
               (vio_T_M_Ck * staticTransformsPtr_
                                 ->rv_T_frame1_frame2(dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getVioOdometryFrame(),
                                                      staticTransformsPtr_->getImuFrame())
                                 .matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4 * 10);

  pubMeasMapVioPath_->publish(*measVio_mapImuPathPtr_);
}

}  // namespace smb_se
