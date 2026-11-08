#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "graph_msf_ros2/GraphMsfRos2.h"

namespace {
// Exercise the production subscription without starting a graph or requiring TF.
class ImuReceiver : public graph_msf::GraphMsfRos2 {
 public:
  explicit ImuReceiver(std::shared_ptr<rclcpp::Node>& node) : GraphMsfRos2(node) {
    initializeSubscribers();
  }
  std::vector<int> received;
 protected:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) override {
    received.push_back(msg->header.stamp.nanosec);
  }
};
}

TEST(ImuSubscription, RetainsAcquisitionOrderAcrossAnExecutorStall) {
  rclcpp::init(0, nullptr);
  {
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    options.arguments({"--ros-args", "-r", "/imu_topic:=/test_graph_imu_buffer"});
    auto node = std::make_shared<rclcpp::Node>("imu_buffer_regression", options);
    ImuReceiver receiver(node);
    auto publisher = node->create_publisher<sensor_msgs::msg::Imu>(
        "/test_graph_imu_buffer", rclcpp::QoS(200).best_effort());
    // No executor runs during this burst: it represents a blocked callback.
    // A depth-one subscription can retain only the last motion increment.
    for (int i = 1; i <= 40; ++i) {
      auto sample = std::make_unique<sensor_msgs::msg::Imu>();
      sample->header.stamp.sec = 100;
      sample->header.stamp.nanosec = i * 5000000;
      publisher->publish(std::move(sample));
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (receiver.received.size() < 40 && std::chrono::steady_clock::now() < deadline)
      executor.spin_some();
    EXPECT_EQ(receiver.received.size(), 40u);
    for (size_t i = 0; i < receiver.received.size() && i < 40; ++i)
      EXPECT_EQ(receiver.received[i], (i + 1) * 5000000);
  }
  rclcpp::shutdown();
}
