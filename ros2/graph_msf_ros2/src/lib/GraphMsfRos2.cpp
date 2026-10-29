// Implementation
#include "graph_msf_ros2/GraphMsfRos2.h"
#include "graph_msf/core/GraphManager.h"
#include <algorithm>
#include <cmath>

// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Workspace
#include "graph_msf_ros2/constants.h"
#include "graph_msf_ros2/util/conversions.h"

namespace graph_msf {

GraphMsfRos2::GraphMsfRos2(std::shared_ptr<rclcpp::Node>& node) : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "GraphMsfRos2-Constructor called.");
  // Configurations ----------------------------
  // Graph Config
  graphConfigPtr_ = std::make_shared<GraphConfig>();
}

void GraphMsfRos2::setup(std::shared_ptr<StaticTransforms> staticTransformsPtr) {
  RCLCPP_INFO(node_->get_logger(), "GraphMsfRos2-Setup called.");

  // Check
  if (staticTransformsPtr == nullptr) {
    throw std::runtime_error("Static transforms not set. Has to be set.");
  }

  // Sensor Params
  node_->declare_parameter("sensor_params.imuRate", 0.0);
  node_->declare_parameter("sensor_params.createStateEveryNthImuMeasurement", 25);
  node_->declare_parameter("sensor_params.useImuSignalLowPassFilter", false);
  node_->declare_parameter("sensor_params.imuLowPassFilterCutoffFreq", 0.0);
  node_->declare_parameter("sensor_params.imuBufferLength", 800);
  node_->declare_parameter("sensor_params.imuTimeOffset", 0.0);
  rosPublishRateHz_ = node_->declare_parameter<double>("ros_publish_rate_hz", 0.0);
  tfPublishRateHz_ = node_->declare_parameter<double>("tf_publish_rate_hz", 0.0);
  worldOdomCorrectionTau_ = node_->declare_parameter<double>("world_odom_correction_tau", 0.0);
  worldOdomCorrectionMaxLinearRate_ = node_->declare_parameter<double>(
      "world_odom_correction_max_linear_rate", 0.0);
  worldOdomCorrectionMaxAngularRate_ = node_->declare_parameter<double>(
      "world_odom_correction_max_angular_rate", 0.0);
  if (rosPublishRateHz_ < 0.0 || tfPublishRateHz_ < 0.0 || worldOdomCorrectionTau_ < 0.0 ||
      worldOdomCorrectionMaxLinearRate_ < 0.0 || worldOdomCorrectionMaxAngularRate_ < 0.0) {
    throw std::invalid_argument("ROS/TF rates and world correction time constant must be nonnegative");
  }
  useImuOrientationFactor_ = node_->declare_parameter<bool>("sensor_params.useImuOrientationFactor", false);
  imuOrientationFactorRate_ = node_->declare_parameter<double>("sensor_params.imuOrientationFactorRate", 10.0);
  imuOrientationFactorNoise_ = node_->declare_parameter<double>("sensor_params.imuOrientationFactorNoise", 0.05);
  if (useImuOrientationFactor_ && (imuOrientationFactorRate_ <= 0.0 || imuOrientationFactorNoise_ <= 0.0)) {
    throw std::invalid_argument("IMU orientation factor rate and noise must be positive");
  }

  // Initialization Params
  node_->declare_parameter("initialization_params.estimateGravityFromImu", false);
  node_->declare_parameter("initialization_params.gravityMagnitude", 9.80665);

  // Graph Params
  node_->declare_parameter("graph_params.realTimeSmootherUseIsam", false);
  node_->declare_parameter("graph_params.realTimeSmootherLag", 0.0);
  node_->declare_parameter("graph_params.useAdditionalSlowBatchSmoother", false);
  node_->declare_parameter("graph_params.slowBatchSmootherUseIsam", false);
  node_->declare_parameter("graph_params.gaussNewtonWildfireThreshold", 0.0);
  node_->declare_parameter("graph_params.minOptimizationFrequency", 0.0);
  node_->declare_parameter("graph_params.maxOptimizationFrequency", 0.0);
  node_->declare_parameter("graph_params.additionalOptimizationIterations", 0);
  node_->declare_parameter("graph_params.findUnusedFactorSlots", false);
  node_->declare_parameter("graph_params.enableDetailedResults", false);
  node_->declare_parameter("graph_params.realTimeSmootherUseCholeskyFactorization", false);
  node_->declare_parameter("graph_params.slowBatchSmootherUseCholeskyFactorization", false);
  node_->declare_parameter("graph_params.usingBiasForPreIntegration", false);
  node_->declare_parameter("graph_params.optimizeReferenceFramePosesWrtWorld", false);
  node_->declare_parameter("graph_params.optimizeExtrinsicSensorToSensorCorrectedOffset", false);
  node_->declare_parameter("graph_params.referenceFramePosesResetThreshold", 0.0);
  node_->declare_parameter("graph_params.centerMeasurementsAtKeyframePositionBeforeAlignment", false);
  
  node_->declare_parameter("graph_params.useWindowForMarginalsComputation", false);
  node_->declare_parameter("graph_params.windowSizeSecondsForMarginalsComputation", 0.0);
  node_->declare_parameter("graph_params.createReferenceAlignmentKeyframeEveryNSeconds", 0.0);

  // Noise Params
  node_->declare_parameter("noise_params.accNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.integrationNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.use2ndOrderCoriolis", false);
  node_->declare_parameter("noise_params.gyrNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.omegaCoriolis", 0.0);
  node_->declare_parameter("noise_params.accBiasRandomWalkNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.gyrBiasRandomWalkNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.biasAccOmegaInit", 0.0);
  node_->declare_parameter("noise_params.accBiasPrior", 0.0);
  node_->declare_parameter("noise_params.gyrBiasPrior", 0.0);
  node_->declare_parameter("noise_params.initialPositionNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.initialOrientationNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.initialVelocityNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.initialAccBiasNoiseDensity", 0.0);
  node_->declare_parameter("noise_params.initialGyroBiasNoiseDensity", 0.0);

  // Re-linearization Params
  node_->declare_parameter("relinearization_params.positionReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.rotationReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.velocityReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.accBiasReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.gyrBiasReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.referenceFrameReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.calibrationReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.displacementReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.landmarkReLinTh", 0.0);
  node_->declare_parameter("relinearization_params.relinearizeSkip", 0);
  node_->declare_parameter("relinearization_params.enableRelinearization", false);
  node_->declare_parameter("relinearization_params.evaluateNonlinearError", false);
  node_->declare_parameter("relinearization_params.cacheLinearizedFactors", false);
  node_->declare_parameter("relinearization_params.enablePartialRelinearizationCheck", false);

  // Common Params
  node_->declare_parameter("common_params.verbosity", 0);
  node_->declare_parameter("common_params.odomNotJumpAtStart", false);
  node_->declare_parameter("common_params.logRealTimeStateToMemory", false);
  node_->declare_parameter("common_params.logLatencyAndUpdateDurationToMemory", false);

  // Extrinsic frames
  node_->declare_parameter("extrinsics.worldFrame", std::string(""));
  node_->declare_parameter("extrinsics.odomFrame", std::string(""));
  node_->declare_parameter("extrinsics.imuFrame", std::string(""));
  node_->declare_parameter("extrinsics.initializeZeroYawAndPositionOfFrame", std::string(""));
  node_->declare_parameter("extrinsics.baseLinkFrame", std::string(""));

  // Name IDs
  node_->declare_parameter("name_ids.referenceFrameAligned", std::string(""));
  node_->declare_parameter("name_ids.sensorFrameCorrected", std::string(""));

  // Logging path
  node_->declare_parameter("launch.optimizationResultLoggingPath", std::string(""));

  // Read parameters ----------------------------
  GraphMsfRos2::readParams();

  // Call super class Setup ----------------------------
  GraphMsf::setup(graphConfigPtr_, staticTransformsPtr);

  // Publishers ----------------------------
  GraphMsfRos2::initializePublishers();

  // Subscribers ----------------------------
  GraphMsfRos2::initializeSubscribers();

  // Messages ----------------------------
  GraphMsfRos2::initializeMessages();

  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Services ----------------------------
  //   GraphMsfRos::initializeServices(*node_);

  // Time
  startTime = std::chrono::high_resolution_clock::now();

  // Wrap up
  RCLCPP_INFO(node_->get_logger(), "Set up successfully.");
}

void GraphMsfRos2::initializePublishers() {
  RCLCPP_INFO(node_->get_logger(), "Initializing publishers.");

  // Odometry
  pubEstOdomImu_ = node_->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldBaseFiltered_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/graph_msf/opt_odometry_world_base_filtered", ROS_QUEUE_SIZE);

  // Vector3 Variances
  pubEstWorldPosVariance_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_pos_variance", ROS_QUEUE_SIZE);
  pubEstWorldRotVariance_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_rot_variance", ROS_QUEUE_SIZE);

  // Velocity Marker
  pubVelocityMarker_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("/graph_msf/velocity_marker", ROS_QUEUE_SIZE);

  // Paths
  pubEstOdomImuPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImuPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImuPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/opt_path_world_imu", ROS_QUEUE_SIZE);

  // Imu Bias
  pubAccelBias_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/accel_bias", ROS_QUEUE_SIZE);
  pubGyroBias_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/gyro_bias", ROS_QUEUE_SIZE);

  // Added Imu Measurements
  pubAddedImuMeas_ = node_->create_publisher<sensor_msgs::msg::Imu>("/graph_msf/added_imu_meas", ROS_QUEUE_SIZE);
}

void GraphMsfRos2::initializeSubscribers() {
  RCLCPP_INFO(node_->get_logger(), "Initializing subscribers.");

  // Imu
  subImu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_topic", rclcpp::QoS(ROS_QUEUE_SIZE).best_effort(),
      std::bind(&GraphMsfRos2::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "GraphMsfRos2 Initialized main IMU subscriber with topic: %s",
              subImu_->get_topic_name());
}

void GraphMsfRos2::initializeMessages() {
  RCLCPP_INFO(node_->get_logger(), "Initializing messages.");

  // Odometry
  estOdomImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  estWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  optWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  optWorldBaseFilteredMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();

  // Vector3 Variances
  estWorldPosVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  estWorldRotVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

  // Path
  estOdomImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  estWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  optWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();

  // Imu Bias
  accelBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  gyroBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
}

// void GraphMsfRos2::initializeServices(rclcpp::Node& node) {
//   RCLCPP_INFO(node.get_logger(), "Initializing services.");

//   // Trigger offline smoother optimization
//   srvSmootherOptimize_ = node.create_service<graph_msf_ros::srv::OfflineOptimizationTrigger>(
//       "/graph_msf/trigger_offline_optimization",
//       std::bind(&GraphMsfRos::srvOfflineSmootherOptimizeCallback, this, std::placeholders::_1,
//       std::placeholders::_2));
// }

// bool GraphMsfRos2::srvOfflineSmootherOptimizeCallback(
//     const std::shared_ptr<graph_msf_ros::srv::OfflineOptimizationTrigger::Request> req,
//     std::shared_ptr<graph_msf_ros::srv::OfflineOptimizationTrigger::Response> res) {
//   // Max Iterations from service call
//   int maxIterations = req->max_optimization_iterations;

//   // Trigger offline smoother optimization and create response
//   if (GraphMsf::optimizeSlowBatchSmoother(maxIterations, optimizationResultLoggingPath)) {
//     res->success = true;
//     res->message = "Optimization successful.";
//   } else {
//     res->success = false;
//     res->message = "Optimization failed.";
//   }
//   return true;
// }

void GraphMsfRos2::addToPathMsg(const nav_msgs::msg::Path::SharedPtr& pathPtr, const std::string& fixedFrameName,
                                const rclcpp::Time& stamp, const Eigen::Vector3d& t, const int maxBufferLength) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixedFrameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pathPtr->header.frame_id = fixedFrameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);
  if (pathPtr->poses.size() > maxBufferLength) {
    pathPtr->poses.erase(pathPtr->poses.begin());
  }
}

void GraphMsfRos2::addToOdometryMsg(const nav_msgs::msg::Odometry::SharedPtr& msgPtr, const std::string& fixedFrame,
                                    const std::string& movingFrame, const rclcpp::Time& stamp,
                                    const Eigen::Isometry3d& T, const Eigen::Vector3d& F_v_W_F,
                                    const Eigen::Vector3d& F_w_W_F, const Eigen::Matrix<double, 6, 6>& poseCovariance,
                                    const Eigen::Matrix<double, 6, 6>& twistCovariance) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose using tf2
  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(T, pose_msg);
  msgPtr->pose.pose = pose_msg;

  msgPtr->twist.twist.linear.x = F_v_W_F(0);
  msgPtr->twist.twist.linear.y = F_v_W_F(1);
  msgPtr->twist.twist.linear.z = F_v_W_F(2);
  msgPtr->twist.twist.angular.x = F_w_W_F(0);
  msgPtr->twist.twist.angular.y = F_w_W_F(1);
  msgPtr->twist.twist.angular.z = F_w_W_F(2);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msgPtr->pose.covariance[6 * i + j] = poseCovariance(i, j);
      msgPtr->twist.covariance[6 * i + j] = twistCovariance(i, j);
    }
  }
}

void GraphMsfRos2::addToPoseWithCovarianceStampedMsg(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msgPtr, const std::string& frameName,
    const rclcpp::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Matrix<double, 6, 6>& transformCovariance) {
  msgPtr->header.frame_id = frameName;
  msgPtr->header.stamp = stamp;

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose using tf2
  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(T, pose_msg);
  msgPtr->pose.pose = pose_msg;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msgPtr->pose.covariance[6 * i + j] = transformCovariance(i, j);
    }
  }
}

void GraphMsfRos2::extractCovariancesFromOptimizedState(
    Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Extract covariances from optimized state
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
    poseCovarianceRos = graph_msf::convertCovarianceGtsamConventionToRosConvention(
        optimizedStateWithCovarianceAndBiasPtr->getPoseCovariance());
    twistCovarianceRos.block<3, 3>(0, 0) = optimizedStateWithCovarianceAndBiasPtr->getVelocityCovariance();
  } else {
    poseCovarianceRos.setZero();
    twistCovarianceRos.setZero();
  }
}

// Markers
void GraphMsfRos2::createVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                                        const Eigen::Vector3d& velocity, visualization_msgs::msg::Marker& marker) {
  // Arrow
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Scale and Color
  const double scale = velocity.norm();
  marker.scale.x = 0.1;  // shaft diameter
  marker.scale.y = 0.2;  // head diameter
  marker.scale.z = 0.2;  // head length
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  // Define Arrow through start and end point
  geometry_msgs::msg::Point startPoint, endPoint;
  startPoint.x = 0.0;  // origin
  startPoint.y = 0.0;  // origin
  startPoint.z = 1.0;  // 1 meter above origin
  endPoint.x = startPoint.x + velocity(0);
  endPoint.y = startPoint.y + velocity(1);
  endPoint.z = startPoint.z + velocity(2);
  marker.points.push_back(startPoint);
  marker.points.push_back(endPoint);

  // Quaternion for orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
}

void GraphMsfRos2::createContactMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                                       const Eigen::Vector3d& position, const std::string& nameSpace, const int id,
                                       visualization_msgs::msg::Marker& marker) {
  // Sphere
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.ns = nameSpace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Scale and Color
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Position
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);

  // Orientation
  marker.pose.orientation.w = 1.0;
}

long GraphMsfRos2::secondsSinceStart() {
  currentTime = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
}

// Main IMU Callback
void GraphMsfRos2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsgPtr) {
  if (imuMsgPtr->orientation_covariance[0] >= 0.0) {
    GraphMsf::setImuOrientationForAlignment(Eigen::Quaterniond(
        imuMsgPtr->orientation.w, imuMsgPtr->orientation.x,
        imuMsgPtr->orientation.y, imuMsgPtr->orientation.z));
  }
  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y,
                            imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y,
                             imuMsgPtr->angular_velocity.z);
  Eigen::Matrix<double, 6, 1> addedImuMeasurements;  // accel, gyro

  // Create pointer for carrying state
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr = nullptr;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr = nullptr;

  // Add measurement and get state
  if (GraphMsf::addCoreImuMeasurementAndGetState(
          linearAcc, angularVel, imuMsgPtr->header.stamp.sec + 1e-9 * imuMsgPtr->header.stamp.nanosec,
          preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr, addedImuMeasurements)) {
    const double imuTime = imuMsgPtr->header.stamp.sec + 1e-9 * imuMsgPtr->header.stamp.nanosec;
    if (useImuOrientationFactor_ && GraphMsf::isGraphInited() && GraphMsf::getNormalOperationFlag() &&
        graphMgrPtr_->hasGraphStateAtTime(imuTime) &&
        imuMsgPtr->orientation_covariance[0] >= 0.0 &&
        imuTime - lastImuOrientationFactorTime_ >= 1.0 / imuOrientationFactorRate_) {
      Eigen::Quaterniond rawOrientation(imuMsgPtr->orientation.w, imuMsgPtr->orientation.x,
                                        imuMsgPtr->orientation.y, imuMsgPtr->orientation.z);
      const double norm = rawOrientation.norm();
      if (rawOrientation.coeffs().allFinite() && norm > 0.5 && norm < 1.5) {
        const gtsam::Rot3 rawRotation(rawOrientation.normalized());
        if (!imuOrientationYawAligned_) {
          const gtsam::Rot3 graphRotation(preIntegratedNavStatePtr->getT_W_Ik().rotation());
          imuOrientationYawOffset_ = graphRotation.yaw() - rawRotation.yaw();
          imuOrientationYawAligned_ = true;
          RCLCPP_INFO(node_->get_logger(), "IMU orientation factor aligned with graph yaw (offset %.3f rad)",
                      imuOrientationYawOffset_);
        }
        const gtsam::Rot3 worldRotation = gtsam::Rot3::Yaw(imuOrientationYawOffset_) * rawRotation;
        const gtsam::Pose3 posePrior(worldRotation, preIntegratedNavStatePtr->getT_W_Ik().translation());
        Eigen::Matrix<double, 6, 1> noise;
        noise << imuOrientationFactorNoise_, imuOrientationFactorNoise_, imuOrientationFactorNoise_,
                 100.0, 100.0, 100.0;
        graphMgrPtr_->addUnaryFactorInImuFrame<gtsam::Pose3, 6, gtsam::PriorFactor<gtsam::Pose3>,
                                               gtsam::symbol_shorthand::X>(posePrior, noise, imuTime);
        {
          const std::lock_guard<std::mutex> lock(optimizeGraphMutex_);
          optimizeGraphFlag_ = true;
        }
        lastImuOrientationFactorTime_ = imuTime;
      }
    }
    // Encountered Delay
    auto now = node_->now();
    auto delay = now.seconds() - preIntegratedNavStatePtr->getTimeK();
    if (delay > 0.5) {
      RCLCPP_WARN(node_->get_logger(), "Encountered delay of %.14f seconds.", delay);
    }

    // Publish Odometry
    this->publishState(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);

    // Publish Filtered Imu Measurements (uncomment if needed)
    // this->publishAddedImuMeas_(addedImuMeasurements, imuMsgPtr->header.stamp);
  } else if (GraphMsf::isGraphInited()) {
    RCLCPP_WARN(node_->get_logger(), "Could not add IMU measurement.");
  }
}

// Publish state ---------------------------------------------------------------
// Higher Level Functions
void GraphMsfRos2::publishState(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  const double timeK = integratedNavStatePtr->getTimeK();
  bool publish_tf = true;
  bool publish_ros = true;
  Eigen::Isometry3d publishedWorldFromOdom;
  // Rate-limit the correction at the robot, not at the odom frame origin.
  // Rotating world->odom around a distant origin otherwise moves base_link
  // by metres even when the optimized base pose changes only slightly.
  const Eigen::Isometry3d odomFromBase =
      integratedNavStatePtr->getT_O_Ik_gravityAligned() *
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                               staticTransformsPtr_->getBaseLinkFrame());
  {
    std::lock_guard<std::mutex> lock(rosPublishGateMutex_);
    const Eigen::Isometry3d& target = integratedNavStatePtr->getT_W_O();
    if (!worldCorrectionInitialized_ || timeK < lastWorldCorrectionTime_ ||
        worldOdomCorrectionTau_ == 0.0) {
      publishedWorldFromOdom_ = target;
      worldCorrectionInitialized_ = true;
    } else {
      const double dt = std::max(0.0, timeK - lastWorldCorrectionTime_);
      const double alpha = 1.0 - std::exp(-dt / worldOdomCorrectionTau_);
      const Eigen::Vector3d baseInOdom = odomFromBase.translation();
      const Eigen::Vector3d currentBaseInWorld = publishedWorldFromOdom_ * baseInOdom;
      const Eigen::Vector3d targetBaseInWorld = target * baseInOdom;
      Eigen::Vector3d baseTranslationStep = alpha * (targetBaseInWorld - currentBaseInWorld);
      const double maxTranslationStep = worldOdomCorrectionMaxLinearRate_ * dt;
      if (worldOdomCorrectionMaxLinearRate_ > 0.0 &&
          baseTranslationStep.norm() > maxTranslationStep && maxTranslationStep > 0.0) {
        baseTranslationStep *= maxTranslationStep / baseTranslationStep.norm();
      }
      const Eigen::Quaterniond previous(publishedWorldFromOdom_.rotation());
      const Eigen::Quaterniond measured(target.rotation());
      double angularAlpha = alpha;
      if (worldOdomCorrectionMaxAngularRate_ > 0.0 && dt > 0.0) {
        const double angularDistance = previous.angularDistance(measured);
        if (angularDistance > 0.0) {
          angularAlpha = std::min(alpha,
              worldOdomCorrectionMaxAngularRate_ * dt / angularDistance);
        }
      }
      publishedWorldFromOdom_.linear() =
          previous.normalized().slerp(angularAlpha, measured.normalized()).toRotationMatrix();
      publishedWorldFromOdom_.translation() =
          currentBaseInWorld + baseTranslationStep - publishedWorldFromOdom_.linear() * baseInOdom;
    }
    lastWorldCorrectionTime_ = timeK;
    publishedWorldFromOdom = publishedWorldFromOdom_;
    const double tf_rate = tfPublishRateHz_ > 0.0 ? tfPublishRateHz_ : rosPublishRateHz_;
    publish_tf = tf_rate <= 0.0 || timeK < lastTfPublishTime_ ||
        timeK - lastTfPublishTime_ >= 1.0 / tf_rate;
    publish_ros = rosPublishRateHz_ <= 0.0 || timeK < lastRosPublishTime_ ||
        timeK - lastRosPublishTime_ >= 1.0 / rosPublishRateHz_;
    if (publish_tf) lastTfPublishTime_ = timeK;
    if (publish_ros) lastRosPublishTime_ = timeK;
  }
  if (publish_tf) {
    // Publish scan-critical TF directly from the IMU callback. Covariance,
    // paths and DDS work in the ROS publisher thread cannot hold these TFs.
    const Eigen::Isometry3d T_B_Ok =
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(),
                                                 staticTransformsPtr_->getImuFrame()) *
        integratedNavStatePtr->getT_O_Ik_gravityAligned().inverse();
    publishTfTreeTransform(staticTransformsPtr_->getOdomFrame(),
                           staticTransformsPtr_->getBaseLinkFrame(), timeK, T_B_Ok.inverse());
    publishTfTreeTransform(staticTransformsPtr_->getWorldFrame(),
                           staticTransformsPtr_->getOdomFrame(), timeK,
                           publishedWorldFromOdom);
  }
  if (!publish_ros) return;

  // Covariances
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  graph_msf::GraphMsfRos2::extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos,
                                                                optimizedStateWithCovarianceAndBiasPtr);

  // Variances (only diagonal elements)
  Eigen::Vector3d positionVarianceRos = poseCovarianceRos.block<3, 3>(0, 0).diagonal();
  Eigen::Vector3d orientationVarianceRos = poseCovarianceRos.block<3, 3>(3, 3).diagonal();

  // Publish non-time critical data in a separate thread
  std::thread publishNonTimeCriticalDataThread(&GraphMsfRos2::publishNonTimeCriticalData, this, poseCovarianceRos,
                                               twistCovarianceRos, positionVarianceRos, orientationVarianceRos,
                                               integratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr,
                                               publishedWorldFromOdom);
  publishNonTimeCriticalDataThread.detach();
}

// Copy the arguments in order to be thread safe
void GraphMsfRos2::publishNonTimeCriticalData(
    const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
    const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
    const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const Eigen::Isometry3d publishedWorldFromOdom) {
  // Mutex for not overloading ROS
  std::lock_guard<std::mutex> lock(rosPublisherMutex_);

  // Time
  const double& timeK = integratedNavStatePtr->getTimeK();  // Alias

  // Odometry messages
  publishImuOdoms(integratedNavStatePtr, poseCovarianceRos, twistCovarianceRos);

  // Publish the latest GTSAM-corrected state propagated by IMU at 10 Hz.
  // Optimizer updates alone may be slower than the navigation control loop.
  if (pubOptWorldBaseFiltered_->get_subscription_count() > 0 &&
      (lastFilteredBasePublishTime_ == 0.0 || timeK - lastFilteredBasePublishTime_ >= 0.095)) {
    const Eigen::Isometry3d T_I_B = staticTransformsPtr_->rv_T_frame1_frame2(
        staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getBaseLinkFrame());
    const Eigen::Matrix3d R_B_I = T_I_B.rotation().transpose();
    const Eigen::Vector3d w_I = integratedNavStatePtr->getI_w_W_I();
    const Eigen::Vector3d v_B = R_B_I *
        (integratedNavStatePtr->getI_v_W_I() + w_I.cross(T_I_B.translation()));
    const Eigen::Vector3d w_B = R_B_I * w_I;
    addToOdometryMsg(optWorldBaseFilteredMsgPtr_, staticTransformsPtr_->getWorldFrame(),
                     staticTransformsPtr_->getBaseLinkFrame(), rclcpp::Time(timeK * 1e9),
                     publishedWorldFromOdom * integratedNavStatePtr->getT_O_Ik_gravityAligned() * T_I_B,
                     v_B, w_B, poseCovarianceRos, twistCovarianceRos);
    pubOptWorldBaseFiltered_->publish(*optWorldBaseFilteredMsgPtr_);
    lastFilteredBasePublishTime_ = timeK;
  }

  // Publish Variances
  publishDiagVarianceVectors(positionVarianceRos, orientationVarianceRos, timeK);

  // Publish Velocity Markers
  publishVelocityMarkers(integratedNavStatePtr);

  // Publish paths
  publishImuPaths(integratedNavStatePtr);

  // Optimized estimate ----------------------
  publishOptimizedStateAndBias(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

void GraphMsfRos2::publishOptimizedStateAndBias(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) {
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      optimizedStateWithCovarianceAndBiasPtr->getTimeK() - lastOptimizedStateTimestamp_ > 1e-03) {
    // Time of this optimized state
    lastOptimizedStateTimestamp_ = optimizedStateWithCovarianceAndBiasPtr->getTimeK();

    // Odometry messages
    // world->imu
    if (pubOptWorldImu_->get_subscription_count() > 0) {
      addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                       rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9),
                       optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(),
                       optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(),
                       optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
      pubOptWorldImu_->publish(*optWorldImuMsgPtr_);
    }

    // Path
    // world->imu
    addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(),
                 rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9),
                 optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().translation(),
                 graphConfigPtr_->imuBufferLength_ * 20);
    if (pubOptWorldImuPath_->get_subscription_count() > 0) {
      pubOptWorldImuPath_->publish(*optWorldImuPathPtr_);
    }

    // Biases
    // Publish accel bias
    if (pubAccelBias_->get_subscription_count() > 0) {
      Eigen::Vector3d accelBias = optimizedStateWithCovarianceAndBiasPtr->getAccelerometerBias();
      accelBiasMsgPtr_->header.stamp = rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9);
      accelBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      accelBiasMsgPtr_->vector.x = accelBias(0);
      accelBiasMsgPtr_->vector.y = accelBias(1);
      accelBiasMsgPtr_->vector.z = accelBias(2);
      pubAccelBias_->publish(*accelBiasMsgPtr_);
    }
    // Publish gyro bias
    if (pubGyroBias_->get_subscription_count() > 0) {
      Eigen::Vector3d gyroBias = optimizedStateWithCovarianceAndBiasPtr->getGyroscopeBias();
      gyroBiasMsgPtr_->header.stamp = rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9);
      gyroBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      gyroBiasMsgPtr_->vector.x = gyroBias(0);
      gyroBiasMsgPtr_->vector.y = gyroBias(1);
      gyroBiasMsgPtr_->vector.z = gyroBias(2);
      pubGyroBias_->publish(*gyroBiasMsgPtr_);
    }

    // TFs in Optimized State
    for (const auto& framePairTransformMapIterator :
         optimizedStateWithCovarianceAndBiasPtr->getReferenceFrameTransforms().getTransformsMap()) {
      // Case 1: If includes world frame --> everything child of world --------------------------------
      if (framePairTransformMapIterator.first.first == staticTransformsPtr_->getWorldFrame() ||
          framePairTransformMapIterator.first.second == staticTransformsPtr_->getWorldFrame()) {
        // A. Get transform
        Eigen::Isometry3d T_W_M;
        std::string mapFrameName;
        const std::string& worldFrameName = staticTransformsPtr_->getWorldFrame();

        // If world is first, then map is second
        if (framePairTransformMapIterator.first.first == worldFrameName) {
          T_W_M = framePairTransformMapIterator.second.inverse();
          mapFrameName = framePairTransformMapIterator.first.second;
        }
        // If world is second, then map is first, this is the case for holistic alignment
        else {
          T_W_M = framePairTransformMapIterator.second;
          mapFrameName = framePairTransformMapIterator.first.first;

          // B. Publish TransformStamped for Aligned Frames
          std::string transformTopic =
              "/graph_msf/transform_" + worldFrameName + "_to_" + mapFrameName + referenceFrameAlignedNameId;
          auto poseWithCovarianceStampedMsgPtr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
          addToPoseWithCovarianceStampedMsg(
              poseWithCovarianceStampedMsgPtr, staticTransformsPtr_->getWorldFrame(),
              rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9), T_W_M,
              optimizedStateWithCovarianceAndBiasPtr->getReferenceFrameTransformsCovariance().rv_T_frame1_frame2(
                  framePairTransformMapIterator.first.first, framePairTransformMapIterator.first.second));
          // Check whether publisher already exists
          if (pubPoseStampedByTopicMap_.find(transformTopic) == pubPoseStampedByTopicMap_.end()) {
            pubPoseStampedByTopicMap_[transformTopic] =
                node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(transformTopic, 1);
            RCLCPP_INFO(node_->get_logger(), "Initialized publisher for %s", transformTopic.c_str());
          }
          if (pubPoseStampedByTopicMap_[transformTopic]->get_subscription_count() > 0) {
            pubPoseStampedByTopicMap_[transformTopic]->publish(*poseWithCovarianceStampedMsgPtr);
          }
        }

        // C. Publish TF Tree
        publishTfTreeTransform(worldFrameName, mapFrameName + referenceFrameAlignedNameId,
                               optimizedStateWithCovarianceAndBiasPtr->getTimeK(), T_W_M);

      }
      // Case 2: Other transformations
      else {
        const std::string& sensorFrameName = framePairTransformMapIterator.first.first;
        const std::string& sensorFrameNameCorrected = framePairTransformMapIterator.first.second;
        const Eigen::Isometry3d& T_sensor_sensorCorrected = framePairTransformMapIterator.second;

        // A. Publish TransformStamped for Corrected Frames
        std::string transformTopic = "/graph_msf/transform_" + sensorFrameName + "_to_" + sensorFrameNameCorrected;
        auto poseWithCovarianceStampedMsgPtr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        addToPoseWithCovarianceStampedMsg(
            poseWithCovarianceStampedMsgPtr, sensorFrameName,
            rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9), T_sensor_sensorCorrected,
            optimizedStateWithCovarianceAndBiasPtr->getReferenceFrameTransformsCovariance().rv_T_frame1_frame2(
                sensorFrameName, sensorFrameNameCorrected));
        // Check whether publisher already exists
        if (pubPoseStampedByTopicMap_.find(transformTopic) == pubPoseStampedByTopicMap_.end()) {
          pubPoseStampedByTopicMap_[transformTopic] =
              node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(transformTopic, 1);
          RCLCPP_INFO(node_->get_logger(), "Initialized publisher for %s", transformTopic.c_str());
        }
        if (pubPoseStampedByTopicMap_[transformTopic]->get_subscription_count() > 0) {
          pubPoseStampedByTopicMap_[transformTopic]->publish(*poseWithCovarianceStampedMsgPtr);
        }

        // B. Publish TF Tree
        publishTfTreeTransform(sensorFrameName, sensorFrameNameCorrected,
                               optimizedStateWithCovarianceAndBiasPtr->getTimeK(), T_sensor_sensorCorrected);
      }
    }
  }
}

// Lower Level Functions
void GraphMsfRos2::publishTfTreeTransform(const std::string& parentFrameName, const std::string& childFrameName,
                                          const double timeStamp, const Eigen::Isometry3d& T_frame_childFrame) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = rclcpp::Time(timeStamp * 1e9);
  transformStamped.header.frame_id = parentFrameName;
  transformStamped.child_frame_id = childFrameName;
  transformStamped.transform.translation.x = T_frame_childFrame.translation().x();
  transformStamped.transform.translation.y = T_frame_childFrame.translation().y();
  transformStamped.transform.translation.z = T_frame_childFrame.translation().z();
  Eigen::Quaterniond q(T_frame_childFrame.rotation());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Transform
  //   tf2::convert(T_frame_childFrame, transformStamped.transform);

  tfBroadcaster_->sendTransform(transformStamped);
}

void GraphMsfRos2::publishImuOdoms(
    const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
    const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const {
  // Odom->imu
  if (pubEstOdomImu_->get_subscription_count() > 0) {
    addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(),
                     rclcpp::Time(preIntegratedNavStatePtr->getTimeK() * 1e9),
                     preIntegratedNavStatePtr->getT_O_Ik_gravityAligned(), preIntegratedNavStatePtr->getI_v_W_I(),
                     preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstOdomImu_->publish(*estOdomImuMsgPtr_);
  }
  // World->imu
  if (pubEstWorldImu_->get_subscription_count() > 0) {
    addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                     rclcpp::Time(preIntegratedNavStatePtr->getTimeK() * 1e9), preIntegratedNavStatePtr->getT_W_Ik(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos,
                     twistCovarianceRos);
    pubEstWorldImu_->publish(*estWorldImuMsgPtr_);
  }
}

void GraphMsfRos2::publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos,
                                              const Eigen::Vector3d& rotVarianceRos, const double timeStamp) const {
  // World Position Variance
  if (pubEstWorldPosVariance_->get_subscription_count() > 0) {
    estWorldPosVarianceMsgPtr_->header.stamp = rclcpp::Time(timeStamp * 1e9);
    estWorldPosVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldPosVarianceMsgPtr_->vector.x = posVarianceRos(0);
    estWorldPosVarianceMsgPtr_->vector.y = posVarianceRos(1);
    estWorldPosVarianceMsgPtr_->vector.z = posVarianceRos(2);
    pubEstWorldPosVariance_->publish(*estWorldPosVarianceMsgPtr_);
  }
  // World Rotation Variance
  if (pubEstWorldRotVariance_->get_subscription_count() > 0) {
    estWorldRotVarianceMsgPtr_->header.stamp = rclcpp::Time(timeStamp * 1e9);
    estWorldRotVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldRotVarianceMsgPtr_->vector.x = rotVarianceRos(0);
    estWorldRotVarianceMsgPtr_->vector.y = rotVarianceRos(1);
    estWorldRotVarianceMsgPtr_->vector.z = rotVarianceRos(2);
    pubEstWorldRotVariance_->publish(*estWorldRotVarianceMsgPtr_);
  }
}

void GraphMsfRos2::publishVelocityMarkers(
    const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const {
  // Velocity in Odom Frame Marker
  visualization_msgs::msg::Marker velocityMarker;
  createVelocityMarker(staticTransformsPtr_->getImuFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9),
                       navStatePtr->getI_v_W_I(), velocityMarker);

  // Publish
  if (pubVelocityMarker_->get_subscription_count() > 0) {
    pubVelocityMarker_->publish(velocityMarker);
  }
}

void GraphMsfRos2::publishImuPaths(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const {
  // odom->imu
  addToPathMsg(estOdomImuPathPtr_, staticTransformsPtr_->getOdomFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9),
               navStatePtr->getT_O_Ik_gravityAligned().translation(), graphConfigPtr_->imuBufferLength_ * 20);
  if (pubEstOdomImuPath_->get_subscription_count() > 0) {
    pubEstOdomImuPath_->publish(*estOdomImuPathPtr_);
  }
  // world->imu
  addToPathMsg(estWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9),
               navStatePtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_ * 20);
  if (pubEstWorldImuPath_->get_subscription_count() > 0) {
    pubEstWorldImuPath_->publish(*estWorldImuPathPtr_);
  }
}

void GraphMsfRos2::publishAddedImuMeas(const Eigen::Matrix<double, 6, 1>& addedImuMeas,
                                       const rclcpp::Time& stamp) const {
  // Publish added imu measurement
  if (pubAddedImuMeas_->get_subscription_count() > 0) {
    sensor_msgs::msg::Imu addedImuMeasMsg;
    addedImuMeasMsg.header.stamp = stamp;
    addedImuMeasMsg.header.frame_id = staticTransformsPtr_->getImuFrame();
    addedImuMeasMsg.linear_acceleration.x = addedImuMeas(0);
    addedImuMeasMsg.linear_acceleration.y = addedImuMeas(1);
    addedImuMeasMsg.linear_acceleration.z = addedImuMeas(2);
    addedImuMeasMsg.angular_velocity.x = addedImuMeas(3);
    addedImuMeasMsg.angular_velocity.y = addedImuMeas(4);
    addedImuMeasMsg.angular_velocity.z = addedImuMeas(5);
    pubAddedImuMeas_->publish(addedImuMeasMsg);
  }
}

}  // namespace graph_msf

