#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "uart_to_stm32/uart_to_stm32.hpp"

namespace
{

using namespace std::chrono_literals;

rclcpp::QoS durableQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

class UartProbe : public rclcpp::Node
{
public:
  UartProbe()
  : Node("uart_fly_choice_probe")
  {
    fly_choice_sub = create_subscription<std_msgs::msg::UInt8>(
      "/fly_choice", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        received_choices.push_back(msg->data);
      });
    fly_choice_status_pub =
      create_publisher<std_msgs::msg::UInt8>("/fly_choice_status", durableQos());
    target_velocity_pub =
      create_publisher<std_msgs::msg::Float32MultiArray>("/target_velocity", 10);
  }

  void publishAcceptedChoice(uint8_t choice)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = choice;
    fly_choice_status_pub->publish(msg);
  }

  void publishVelocity()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {1.0F, -2.0F, 3.0F, -4.0F};
    target_velocity_pub->publish(msg);
  }

  std::vector<uint8_t> received_choices;

private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fly_choice_sub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fly_choice_status_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub;
};

class FlyChoiceSerialTest : public ::testing::Test
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
    char slave_name[128] = {};
    ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);

    termios tty {};
    ASSERT_EQ(tcgetattr(slave_fd, &tty), 0);
    cfmakeraw(&tty);
    ASSERT_EQ(tcsetattr(slave_fd, TCSANOW, &tty), 0);
    close(slave_fd);
    slave_fd = -1;

    const int flags = fcntl(master_fd, F_GETFL, 0);
    ASSERT_GE(flags, 0);
    ASSERT_EQ(fcntl(master_fd, F_SETFL, flags | O_NONBLOCK), 0);

    bridge_node = std::make_shared<rclcpp::Node>("uart_bridge_test_node");
    bridge = std::make_shared<uart_to_stm32::UartToStm32>(bridge_node);
    ASSERT_TRUE(bridge->initialize(slave_name, 921600));
    probe = std::make_shared<UartProbe>();
    executor.add_node(bridge_node);
    executor.add_node(probe);
    pump(200ms);
    drainSerial();
  }

  void TearDown() override
  {
    executor.remove_node(probe);
    executor.remove_node(bridge_node);
    bridge.reset();
    probe.reset();
    bridge_node.reset();
    if (slave_fd >= 0) {
      close(slave_fd);
    }
    if (master_fd >= 0) {
      close(master_fd);
    }
  }

  void pump(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::vector<uint8_t> drainSerial()
  {
    std::vector<uint8_t> bytes;
    std::array<uint8_t, 256> buffer {};
    while (true) {
      const ssize_t count = read(master_fd, buffer.data(), buffer.size());
      if (count <= 0) {
        break;
      }
      bytes.insert(bytes.end(), buffer.begin(), buffer.begin() + count);
    }
    return bytes;
  }

  void writeFrame(const std::array<uint8_t, 7> & frame)
  {
    ASSERT_EQ(
      write(master_fd, frame.data(), frame.size()),
      static_cast<ssize_t>(frame.size()));
  }

  int master_fd{-1};
  int slave_fd{-1};
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> bridge_node;
  std::shared_ptr<uart_to_stm32::UartToStm32> bridge;
  std::shared_ptr<UartProbe> probe;
};

TEST_F(FlyChoiceSerialTest, ConvertsValidModeFramesToLocalFlyChoice)
{
  writeFrame({0xAA, 0xFF, 0x11, 0x01, 0x01, 0xBC, 0x84});
  pump(300ms);
  ASSERT_FALSE(probe->received_choices.empty());
  EXPECT_EQ(probe->received_choices.back(), 1);

  writeFrame({0xAA, 0xFF, 0x11, 0x01, 0x02, 0xBD, 0x85});
  pump(300ms);
  ASSERT_GE(probe->received_choices.size(), 2U);
  EXPECT_EQ(probe->received_choices.back(), 2);
}

TEST_F(FlyChoiceSerialTest, RejectsInvalidModeAndChecksum)
{
  writeFrame({0xAA, 0xFF, 0x11, 0x01, 0x03, 0xBE, 0x86});
  writeFrame({0xAA, 0xFF, 0x11, 0x01, 0x01, 0xBC, 0x00});
  pump(300ms);
  EXPECT_TRUE(probe->received_choices.empty());
}

TEST_F(FlyChoiceSerialTest, KeepsVelocitySerialOutputSilentUntilChoiceAccepted)
{
  probe->publishVelocity();
  pump(200ms);
  EXPECT_TRUE(drainSerial().empty());

  probe->publishAcceptedChoice(1);
  pump(100ms);
  probe->publishVelocity();
  pump(200ms);
  const auto bytes = drainSerial();
  ASSERT_GE(bytes.size(), 14U);
  EXPECT_EQ(bytes[0], 0xAA);
  EXPECT_EQ(bytes[1], 0xFF);
  EXPECT_EQ(bytes[2], 0x31);
  EXPECT_EQ(bytes[3], 0x08);
}

}  // namespace
