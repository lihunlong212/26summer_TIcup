#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "pid_control_pkg/pid_controller.hpp"

namespace
{

using namespace std::chrono_literals;

rclcpp::QoS durableQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

class PidProbe : public rclcpp::Node
{
public:
  PidProbe()
  : Node("pid_probe")
  {
    target_pub =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/target_position", durableQos());
    height_pub = create_publisher<std_msgs::msg::Int16>("/height", 10);
    visual_pub =
      create_publisher<std_msgs::msg::Bool>(
      "/visual_takeover_active", durableQos());
    hold_pub =
      create_publisher<std_msgs::msg::Bool>("/motion_hold_active", durableQos());
    descent_pub =
      create_publisher<std_msgs::msg::Bool>(
      "/visual_descent_active", durableQos());
    fine_pub = create_publisher<std_msgs::msg::Int32MultiArray>("/fine_data", 10);
    velocity_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        velocities.emplace_back(msg->data.begin(), msg->data.end());
      });
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void publishCommon(bool publish_fine, bool motion_hold = false)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "laser_link";
    transform.transform.rotation.w = 1.0;
    tf_broadcaster->sendTransform(transform);

    std_msgs::msg::Float32MultiArray target;
    target.data = {0.0F, 0.0F, 0.0F, 0.0F};
    target_pub->publish(target);
    std_msgs::msg::Int16 height;
    height.data = 150;
    height_pub->publish(height);
    publishBool(visual_pub, true);
    publishBool(hold_pub, motion_hold);
    publishBool(descent_pub, true);

    if (publish_fine) {
      std_msgs::msg::Int32MultiArray fine;
      fine.data = {0, 0};
      fine_pub->publish(fine);
    }
  }

  static void publishBool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
    bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    publisher->publish(msg);
  }

  std::vector<std::vector<float>> velocities;

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr descent_pub;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_pub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

class VisualDescentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    probe = std::make_shared<PidProbe>();
    controller = std::make_shared<pid_control_pkg::PositionPIDController>();
    executor.add_node(probe);
    executor.add_node(controller);
    pump(300ms, true);
  }

  void TearDown() override
  {
    executor.remove_node(controller);
    executor.remove_node(probe);
    controller.reset();
    probe.reset();
  }

  void pump(std::chrono::milliseconds duration, bool publish_fine)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      probe->publishCommon(publish_fine);
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  bool waitForVelocity(
    const std::function<bool(const std::vector<float> &)> & predicate,
    std::chrono::milliseconds timeout,
    bool publish_fine)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      probe->publishCommon(publish_fine);
      executor.spin_some();
      for (const auto & velocity : probe->velocities) {
        if (predicate(velocity)) {
          return true;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<PidProbe> probe;
  std::shared_ptr<pid_control_pkg::PositionPIDController> controller;
};

TEST_F(VisualDescentTest, LimitsDescentAndForcesExactZeroWhenVisionIsStale)
{
  ASSERT_TRUE(waitForVelocity(
      [](const std::vector<float> & velocity) {
        return velocity.size() == 4 &&
               std::fabs(velocity[2] + 20.0F) < 0.2F;
      },
      1s, true));

  for (const auto & velocity : probe->velocities) {
    if (velocity.size() == 4 && velocity[2] < 0.0F) {
      EXPECT_GE(velocity[2], -20.01F);
    }
  }

  probe->velocities.clear();
  pump(300ms, false);
  ASSERT_TRUE(waitForVelocity(
      [](const std::vector<float> & velocity) {
        return velocity.size() == 4 &&
               std::all_of(
          velocity.begin(), velocity.end(),
          [](float value) {return std::fabs(value) < 1e-6F;});
      },
      1s, false));
}

TEST_F(VisualDescentTest, MotionHoldContinuouslyPublishesExactZero)
{
  probe->velocities.clear();
  const auto deadline = std::chrono::steady_clock::now() + 250ms;
  while (std::chrono::steady_clock::now() < deadline) {
    probe->publishCommon(true, true);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  std::size_t zero_count = 0;
  for (const auto & velocity : probe->velocities) {
    if (velocity.size() == 4 &&
      std::all_of(
        velocity.begin(), velocity.end(),
        [](float value) {return std::fabs(value) < 1e-6F;}))
    {
      ++zero_count;
    }
  }
  EXPECT_GE(zero_count, 5U);
}

}  // namespace
