#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <cstdint>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <serial_comm/serial_comm.h>

namespace uart_to_stm32
{

class UartToStm32
{
public:
  explicit UartToStm32(rclcpp::Node::SharedPtr node);
  ~UartToStm32();

  bool initialize(const std::string & serial_port, unsigned int baud_rate);

private:
  void targetVelocityCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void serialByteCommandCallback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void flyChoiceStatusCallback(
    const std_msgs::msg::UInt8::SharedPtr msg);
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);
  void sendTargetVelocityToSerial(
    float vx_cm_per_s,
    float vy_cm_per_s,
    float vz_cm_per_s,
    float vyaw_deg_per_s);
  bool sendByteFrame(uint8_t frame_id, uint8_t value);
  void publishByteFrameResult(uint8_t frame_id, uint8_t value, bool success);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<serial_comm::SerialComm> serial_comm_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_byte_command_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fly_choice_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fly_choice_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
    serial_byte_command_result_pub_;
  bool has_st_ready_pub_;
  std::atomic<bool> velocity_output_enabled_{false};
  uint8_t last_stm32_fly_choice_{0};

  static constexpr uint8_t TARGET_VELOCITY_FRAME_ID = 0x31;
  static constexpr uint8_t ST_READY_QUERY_ID = 0xF1;
  static constexpr uint8_t HEIGHT_FRAME_ID = 0x05;
  static constexpr uint8_t FLY_CHOICE_FRAME_ID = 0x11;
};

}  // namespace uart_to_stm32

#endif  // UART_TO_STM32__UART_TO_STM32_HPP_
