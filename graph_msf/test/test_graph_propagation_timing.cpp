#include <gtest/gtest.h>

#include "graph_msf/core/GraphManager.h"

namespace {
std::shared_ptr<graph_msf::GraphConfig> makeConfig() {
  auto config = std::make_shared<graph_msf::GraphConfig>();
  config->imuRate_ = 200.0;
  config->createStateEveryNthImuMeasurement_ = 10;
  config->useImuSignalLowPassFilter_ = false;
  config->use2ndOrderCoriolisFlag_ = false;
  config->optimizeReferenceFramePosesWrtWorldFlag_ = false;
  config->realTimeSmootherLag_ = 4.0;
  config->relinearizeSkip_ = 1;
  config->biasAccOmegaInit_ = 1e-5;
  return config;
}

void checkPropagationAcrossOptimization(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& gyro,
                                       const int samplesBeforeOptimization) {
  const auto config = makeConfig();
  graph_msf::GraphManager manager(config, "imu", "world");
  ASSERT_TRUE(manager.initImuIntegrators(9.81));
  ASSERT_TRUE(manager.initPoseVelocityBiasGraph(1.0, gtsam::Pose3(), gtsam::Pose3()));
  const auto buffer = std::make_shared<graph_msf::ImuBuffer>(config);
  graph_msf::SafeIntegratedNavState state;
  std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias> optimized;
  buffer->addToImuBuffer(1.0, acceleration, gyro);
  for (int i = 1; i <= samplesBeforeOptimization; ++i) {
    const double stamp = 1.0 + 0.005 * i;
    buffer->addToImuBuffer(stamp, acceleration, gyro);
    manager.addImuFactorAndGetState(state, optimized, buffer, stamp, i % 10 == 0);
  }
  // Optimize between two graph keys, with seven already integrated IMU
  // samples after the latest key. The optimizer has no new IMU arriving
  // during this synchronous call; those seven samples must still survive.
  const double keyTime = 1.0 + 0.005 * (samplesBeforeOptimization / 10 * 10);
  EXPECT_TRUE(manager.hasGraphStateAtTime(keyTime));
  EXPECT_FALSE(manager.hasGraphStateAtTime(state.getTimeK()));
  manager.updateGraph();
  const double elapsed = 0.005 * (samplesBeforeOptimization + 1);
  buffer->addToImuBuffer(1.0 + elapsed, acceleration, gyro);
  manager.addImuFactorAndGetState(state, optimized, buffer, 1.0 + elapsed, false);
  ASSERT_NE(optimized, nullptr);
  EXPECT_NEAR(optimized->getTimeK(), keyTime, 1e-9);
  EXPECT_NEAR(gtsam::Rot3(state.getT_W_Ik().rotation()).yaw(), gyro.z() * elapsed, 1e-5);
  EXPECT_NEAR(state.getT_W_Ik().translation().x(), 0.5 * acceleration.x() * elapsed * elapsed, 1e-4);
}
}  // namespace

TEST(GraphPropagationTiming, PreservesTurnMotionBetweenKeyAndOptimizationStart) {
  checkPropagationAcrossOptimization(Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d(0, 0, 2.0), 17);
}

TEST(GraphPropagationTiming, PreservesWalkingMotionBetweenKeyAndOptimizationStart) {
  checkPropagationAcrossOptimization(Eigen::Vector3d(1.0, 0, 9.81), Eigen::Vector3d::Zero(), 217);
}

TEST(GraphPropagationTiming, ComposesInitialWorldAndOdomFrames) {
  const auto config = makeConfig();
  graph_msf::GraphManager manager(config, "imu", "world");
  ASSERT_TRUE(manager.initImuIntegrators(9.81));
  const gtsam::Pose3 worldFromImu(gtsam::Rot3::Yaw(0.8), gtsam::Point3(2.0, -1.0, 0.0));
  const gtsam::Pose3 odomFromImu(gtsam::Rot3::Yaw(-0.2), gtsam::Point3(0.3, 0.1, 0.0));
  ASSERT_TRUE(manager.initPoseVelocityBiasGraph(1.0, worldFromImu, odomFromImu));
  const auto buffer = std::make_shared<graph_msf::ImuBuffer>(config);
  const Eigen::Vector3d accel(0, 0, 9.81), gyro = Eigen::Vector3d::Zero();
  buffer->addToImuBuffer(1.0, accel, gyro);
  buffer->addToImuBuffer(1.005, accel, gyro);
  graph_msf::SafeIntegratedNavState state;
  std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias> optimized;
  manager.addImuFactorAndGetState(state, optimized, buffer, 1.005, false);
  EXPECT_LT((state.getT_W_Ik().matrix() - worldFromImu.matrix()).norm(), 1e-9);
}
