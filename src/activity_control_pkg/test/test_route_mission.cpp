#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "activity_control_pkg/route_target_publisher.hpp"

namespace
{

using namespace std::chrono_literals;

rclcpp::QoS durableQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

class MissionProbe : public rclcpp::Node
{
public:
  MissionProbe()
  : Node("mission_probe"),
    pose_x_m(1.0),
    pose_y_m(0.0),
    pose_yaw_deg(0.0),
    height_cm(150),
    waypoint_index(-2),
    visual_active(false),
    visual_descent_active(false),
    has_drone_state(false),
    drone_state(0)
  {
    tf_stamp = now();
    choice_pub = create_publisher<std_msgs::msg::UInt8>("/fly_choice", 10);
    height_pub = create_publisher<std_msgs::msg::Int16>("/height", 10);
    fine_pub = create_publisher<std_msgs::msg::Int32MultiArray>("/fine_data", 10);
    serial_result_pub =
      create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/serial_byte_command_result", 10);
    target_velocity_pub =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10);

    drone_state_sub = create_subscription<std_msgs::msg::UInt8>(
      "/drone_state", durableQos(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        has_drone_state = true;
        drone_state = msg->data;
        drone_states.push_back(msg->data);
      });
    waypoint_sub = create_subscription<std_msgs::msg::Int32>(
      "/current_waypoint_index", durableQos(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        waypoint_index = msg->data;
      });
    visual_sub = create_subscription<std_msgs::msg::Bool>(
      "/visual_takeover_active", durableQos(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        visual_active = msg->data;
      });
    visual_descent_sub = create_subscription<std_msgs::msg::Bool>(
      "/visual_descent_active", durableQos(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        visual_descent_active = msg->data;
      });
    target_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", durableQos(),
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        target.assign(msg->data.begin(), msg->data.end());
      });
    serial_sub = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/serial_byte_command", 10,
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() == 2) {
          serial_commands.emplace_back(msg->data[0], msg->data[1]);
        }
      });
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void publishInputs()
  {
    // Keep test TF strictly monotonic even if the host/WSL wall clock is adjusted
    // while multiple ROS test processes are running in parallel.
    tf_stamp = tf_stamp + rclcpp::Duration::from_seconds(0.01);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = tf_stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "laser_link";
    transform.transform.translation.x = pose_x_m;
    transform.transform.translation.y = pose_y_m;
    const double yaw_rad = pose_yaw_deg * M_PI / 180.0;
    transform.transform.rotation.z = std::sin(yaw_rad / 2.0);
    transform.transform.rotation.w = std::cos(yaw_rad / 2.0);
    tf_broadcaster->sendTransform(transform);

    std_msgs::msg::Int16 height;
    height.data = height_cm;
    height_pub->publish(height);
  }

  void publishChoice(uint8_t value)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    choice_pub->publish(msg);
  }

  void publishFine(int32_t forward_error = 6, int32_t lateral_error = -4)
  {
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {forward_error, lateral_error};
    fine_pub->publish(msg);
  }

  void publishTargetVelocity()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {1.0F, 0.0F, 1.0F, 0.0F};
    target_velocity_pub->publish(msg);
  }

  void publishSerialResult(uint8_t frame_id, uint8_t value)
  {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {frame_id, value, 1};
    serial_result_pub->publish(msg);
  }

  std::size_t commandCount(uint8_t frame_id, uint8_t value) const
  {
    return static_cast<std::size_t>(std::count(
      serial_commands.begin(), serial_commands.end(),
      std::make_pair(frame_id, value)));
  }

  double pose_x_m;
  double pose_y_m;
  double pose_yaw_deg;
  int16_t height_cm;
  int32_t waypoint_index;
  bool visual_active;
  bool visual_descent_active;
  bool has_drone_state;
  uint8_t drone_state;
  std::vector<uint8_t> drone_states;
  std::vector<float> target;
  std::vector<std::pair<uint8_t, uint8_t>> serial_commands;

private:
  rclcpp::Time tf_stamp;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr choice_pub;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_result_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr drone_state_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_descent_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

class RouteMissionTest : public ::testing::Test
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

  rclcpp::NodeOptions routeOptions() const
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "route_drop_waypoints",
      std::vector<std::string>{
        "(0 0 150 0)", "(-32 65 150 0)", "(-32 182 150 0)"});
    options.append_parameter_override("route_drop_normal_count", 1);
    options.append_parameter_override(
      "route_land_waypoints",
      std::vector<std::string>{
        "(0 0 150 0)", "(-32 65 150 0)", "(-32 182 150 0)"});
    options.append_parameter_override("route_land_normal_count", 1);
    options.append_parameter_override("takeoff_hover_sec", 0.2);
    options.append_parameter_override("fine_data_timeout_sec", 0.2);
    options.append_parameter_override("pre_descent_alignment_sec", 0.3);
    options.append_parameter_override("drop_alignment_height_cm", 55.0);
    options.append_parameter_override("drop_alignment_tolerance_px", 100.0);
    options.append_parameter_override("drop_alignment_required_frames", 3);
    options.append_parameter_override("landing_trigger_height_cm", 45.0);
    options.append_parameter_override("landed_hold_sec", 0.2);
    options.append_parameter_override("return_height_cm", 150.0);
    options.append_parameter_override("drone_state_action_height_cm", 80.0);
    return options;
  }

  void makeSystem(const rclcpp::NodeOptions & options)
  {
    probe = std::make_shared<MissionProbe>();
    controller =
      std::make_shared<activity_control_pkg::RouteTargetPublisherNode>(options);
    executor.add_node(probe);
    executor.add_node(controller);
    pump(300ms);
  }

  void pump(
    std::chrono::milliseconds duration,
    const std::function<void()> & stimulate = {})
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      probe->publishInputs();
      if (stimulate) {
        stimulate();
      }
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  bool waitFor(
    const std::function<bool()> & predicate,
    std::chrono::milliseconds timeout,
    const std::function<void()> & stimulate = {})
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      probe->publishInputs();
      if (stimulate) {
        stimulate();
      }
      executor.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  void expectTarget(double x, double y, double z, double yaw)
  {
    ASSERT_GE(probe->target.size(), 4U);
    EXPECT_NEAR(probe->target[0], x, 0.6);
    EXPECT_NEAR(probe->target[1], y, 0.6);
    EXPECT_NEAR(probe->target[2], z, 0.6);
    EXPECT_NEAR(probe->target[3], yaw, 0.6);
  }

  void TearDown() override
  {
    if (controller) {
      executor.remove_node(controller);
    }
    if (probe) {
      executor.remove_node(probe);
    }
    controller.reset();
    probe.reset();
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<MissionProbe> probe;
  std::shared_ptr<activity_control_pkg::RouteTargetPublisherNode> controller;
};

TEST_F(RouteMissionTest, DropSearchTakeoverTriggerAndReturn)
{
  makeSystem(routeOptions());
  ASSERT_FALSE(probe->has_drone_state);
  EXPECT_TRUE(probe->get_publishers_info_by_topic("/mission_state").empty());

  probe->publishChoice(9);
  pump(100ms);
  EXPECT_FALSE(probe->has_drone_state);

  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->has_drone_state && probe->drone_state == 1;
      }, 1s,
      [this]() {
        probe->publishChoice(1);
        probe->publishTargetVelocity();
      }));

  pump(300ms, [this]() {probe->publishFine();});
  EXPECT_EQ(probe->waypoint_index, 0);
  EXPECT_FALSE(probe->visual_active);

  probe->pose_x_m = 0.0;
  pump(100ms);
  EXPECT_EQ(probe->waypoint_index, 0);
  EXPECT_EQ(probe->drone_state, 1);
  ASSERT_TRUE(waitFor(
      [this]() {return probe->waypoint_index == 1;}, 1s));
  pump(300ms);
  EXPECT_EQ(probe->drone_state, 2);
  EXPECT_FALSE(probe->visual_active);

  probe->pose_x_m = -0.32;
  probe->pose_y_m = 0.65;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->waypoint_index == 2;}, 1s));

  ASSERT_TRUE(waitFor(
      [this]() {return probe->visual_active;}, 1s,
      [this]() {probe->publishFine();}));
  EXPECT_TRUE(probe->visual_active);
  EXPECT_EQ(probe->drone_state, 2);
  ASSERT_GE(probe->target.size(), 4U);
  EXPECT_FLOAT_EQ(probe->target[2], 150.0F);
  EXPECT_FALSE(probe->visual_descent_active);
  pump(100ms, [this]() {probe->publishFine();});
  EXPECT_FLOAT_EQ(probe->target[2], 150.0F);
  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->target.size() >= 4 &&
               std::fabs(probe->target[2] - 55.0F) < 0.6F &&
               probe->visual_descent_active;
      }, 1s, [this]() {probe->publishFine();}));

  probe->pose_x_m = 0.44;
  probe->pose_y_m = -0.22;
  probe->pose_yaw_deg = 15.0;
  probe->height_cm = 80;
  pump(100ms, [this]() {probe->publishFine();});
  EXPECT_EQ(probe->drone_state, 2);
  probe->height_cm = 79;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->drone_state == 3;}, 1s,
      [this]() {probe->publishFine();}));
  probe->height_cm = 55;

  // Two aligned frames are insufficient.
  probe->publishFine(80, -80);
  pump(60ms);
  probe->publishFine(80, -80);
  pump(60ms);
  EXPECT_EQ(probe->commandCount(0x11, 0x01), 0U);

  // One out-of-tolerance frame resets the count.
  probe->publishFine(101, 0);
  pump(60ms);
  probe->publishFine(80, -80);
  pump(60ms);
  probe->publishFine(80, -80);
  pump(60ms);
  EXPECT_EQ(probe->commandCount(0x11, 0x01), 0U);

  // A visual timeout also resets the count.
  pump(260ms);
  probe->publishFine(80, -80);
  pump(60ms);
  EXPECT_EQ(probe->commandCount(0x11, 0x01), 0U);
  probe->publishFine(80, -80);
  pump(60ms);
  EXPECT_EQ(probe->commandCount(0x11, 0x01), 0U);
  probe->publishFine(80, -80);
  ASSERT_TRUE(waitFor(
      [this]() {return probe->commandCount(0x11, 0x01) == 1;}, 1s));
  pump(250ms, [this]() {probe->publishFine();});
  EXPECT_EQ(probe->commandCount(0x11, 0x01), 1U);
  EXPECT_FALSE(probe->visual_active);

  probe->publishSerialResult(0x11, 0x01);
  ASSERT_TRUE(waitFor(
      [this]() {return probe->drone_state == 5;}, 1s));
  expectTarget(44.0, -22.0, 150.0, 15.0);
  pump(200ms, [this]() {probe->publishFine();});
  EXPECT_FALSE(probe->visual_active);

  probe->height_cm = 150;
  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->target.size() >= 4 &&
               std::fabs(probe->target[0]) < 0.6 &&
               std::fabs(probe->target[1]) < 0.6;
      }, 1s));
  expectTarget(0.0, 0.0, 150.0, 0.0);

  probe->pose_x_m = 0.0;
  probe->pose_y_m = 0.0;
  probe->pose_yaw_deg = 0.0;
  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->target.size() >= 4 &&
               std::fabs(probe->target[2]) < 0.6;
      }, 1s));
  expectTarget(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(
    std::count(probe->drone_states.begin(), probe->drone_states.end(), 0), 0);
}

TEST_F(RouteMissionTest, LandingTriggerHoldTakeoffAndReturn)
{
  makeSystem(routeOptions());
  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->has_drone_state && probe->drone_state == 1;
      }, 1s,
      [this]() {
        probe->publishChoice(2);
        probe->publishTargetVelocity();
      }));

  probe->pose_x_m = 0.0;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->waypoint_index == 1;}, 1s));
  ASSERT_TRUE(waitFor(
      [this]() {return probe->visual_active;}, 1s,
      [this]() {probe->publishFine();}));
  ASSERT_GE(probe->target.size(), 4U);
  EXPECT_FLOAT_EQ(probe->target[2], 150.0F);
  EXPECT_FALSE(probe->visual_descent_active);
  EXPECT_EQ(probe->drone_state, 2);
  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->target.size() >= 4 &&
               std::fabs(probe->target[2] - 45.0F) < 0.6F &&
               probe->visual_descent_active;
      }, 1s, [this]() {probe->publishFine();}));

  probe->pose_x_m = 0.12;
  probe->pose_y_m = 0.34;
  probe->pose_yaw_deg = 20.0;
  probe->height_cm = 80;
  pump(100ms, [this]() {probe->publishFine();});
  EXPECT_EQ(probe->drone_state, 2);
  probe->height_cm = 79;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->drone_state == 4;}, 1s,
      [this]() {probe->publishFine();}));
  probe->height_cm = 50;
  pump(250ms);
  EXPECT_EQ(probe->commandCount(0x44, 0x01), 0U);
  probe->height_cm = 45;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->commandCount(0x44, 0x01) == 1;}, 1s));
  EXPECT_EQ(probe->drone_state, 4);
  pump(200ms, [this]() {probe->publishFine();});
  EXPECT_EQ(probe->commandCount(0x44, 0x01), 1U);
  EXPECT_FALSE(probe->visual_active);

  probe->publishSerialResult(0x44, 0x01);
  ASSERT_TRUE(waitFor(
      [this]() {return probe->commandCount(0x44, 0x00) == 1;}, 1s));
  EXPECT_EQ(probe->commandCount(0x44, 0x00), 1U);

  // The car moves during the five-second hold (shortened to 0.2 s in this test).
  probe->pose_x_m = 0.56;
  probe->pose_y_m = 0.78;
  probe->pose_yaw_deg = 30.0;
  probe->publishSerialResult(0x44, 0x00);
  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->drone_state == 5 &&
               probe->target.size() >= 4 &&
               std::fabs(probe->target[0] - 56.0F) < 0.6F &&
               std::fabs(probe->target[1] - 78.0F) < 0.6F &&
               std::fabs(probe->target[2] - 150.0F) < 0.6F;
      }, 1s));
  expectTarget(56.0, 78.0, 150.0, 30.0);
  pump(200ms, [this]() {probe->publishFine();});
  EXPECT_FALSE(probe->visual_active);
  EXPECT_EQ(
    std::count(probe->drone_states.begin(), probe->drone_states.end(), 0), 0);
}

TEST_F(RouteMissionTest, SearchWithoutTagEntersReturnStateFive)
{
  auto options = routeOptions();
  options.append_parameter_override(
    "route_drop_waypoints",
    std::vector<std::string>{"(0 0 150 0)", "(-32 65 150 0)"});
  makeSystem(options);

  ASSERT_TRUE(waitFor(
      [this]() {
        return probe->has_drone_state && probe->drone_state == 1;
      }, 1s,
      [this]() {
        probe->publishChoice(1);
        probe->publishTargetVelocity();
      }));
  probe->pose_x_m = 0.0;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->drone_state == 2;}, 1s));

  probe->pose_x_m = -0.32;
  probe->pose_y_m = 0.65;
  ASSERT_TRUE(waitFor(
      [this]() {return probe->drone_state == 5;}, 1s));
  expectTarget(0.0, 0.0, 150.0, 0.0);
}

TEST_F(RouteMissionTest, InvalidTupleRejectsMission)
{
  auto options = routeOptions();
  options.append_parameter_override(
    "route_drop_waypoints",
    std::vector<std::string>{"(0 0 150)", "(1 2 3 4)"});
  makeSystem(options);
  pump(300ms, [this]() {
      probe->publishChoice(1);
      probe->publishTargetVelocity();
    });
  EXPECT_FALSE(probe->has_drone_state);
}

TEST_F(RouteMissionTest, InvalidNormalCountRejectsMission)
{
  auto options = routeOptions();
  options.append_parameter_override(
    "route_drop_waypoints",
    std::vector<std::string>{"(0 0 150 0)"});
  options.append_parameter_override("route_drop_normal_count", 1);
  makeSystem(options);
  pump(300ms, [this]() {
      probe->publishChoice(1);
      probe->publishTargetVelocity();
    });
  EXPECT_FALSE(probe->has_drone_state);
}

}  // namespace
