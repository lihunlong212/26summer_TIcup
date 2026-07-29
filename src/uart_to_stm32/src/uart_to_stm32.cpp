#include "uart_to_stm32/uart_to_stm32.hpp"

#include <cmath>
#include <utility>

namespace uart_to_stm32
{

UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)), has_st_ready_pub_(false)
{
}

UartToStm32::~UartToStm32()
{
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool UartToStm32::initialize(
  const std::string & serial_port, unsigned int baud_rate)
{
  serial_comm_ = std::make_unique<serial_comm::SerialComm>();
  if (!serial_comm_->initialize(serial_port, baud_rate)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to open %s at %u baud: %s",
      serial_port.c_str(), baud_rate, serial_comm_->get_last_error().c_str());
    return false;
  }

  target_velocity_sub_ =
    node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", 10,
    std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));
  serial_byte_command_sub_ =
    node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/serial_byte_command", 10,
    std::bind(&UartToStm32::serialByteCommandCallback, this, std::placeholders::_1));
  const auto height_topic = node_->declare_parameter<std::string>(
    "height_topic", "/height_stm32");
  height_pub_ = node_->create_publisher<std_msgs::msg::Int16>(height_topic, 10);
  is_st_ready_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
    "/is_st_ready", rclcpp::QoS(1).transient_local().reliable());
  serial_byte_command_result_pub_ =
    node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
    "/serial_byte_command_result", 10);

  serial_comm_->start_protocol_receive(
    [this](uint8_t id, const std::vector<uint8_t> & data) {
      protocolDataHandler(id, data);
    },
    [this](const std::string & error) {
      RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", error.c_str());
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "STM32 bridge ready on %s at %u baud; frame 0x05 publishes %s in cm.",
    serial_port.c_str(), baud_rate, height_topic.c_str());
  return true;
}

void UartToStm32::targetVelocityCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(
      node_->get_logger(),
      "/target_velocity requires [vx_cm_s, vy_cm_s, vz_cm_s, vyaw_deg_s].");
    return;
  }
  sendTargetVelocityToSerial(
    msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

void UartToStm32::serialByteCommandCallback(
  const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 2) {
    RCLCPP_WARN(
      node_->get_logger(),
      "/serial_byte_command must contain exactly [frame_id, value].");
    return;
  }
  const uint8_t frame_id = msg->data[0];
  const uint8_t value = msg->data[1];
  publishByteFrameResult(frame_id, value, sendByteFrame(frame_id, value));
}

void UartToStm32::sendTargetVelocityToSerial(
  float vx_cm_per_s,
  float vy_cm_per_s,
  float vz_cm_per_s,
  float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is closed; target velocity was not sent.");
    return;
  }

  const int16_t values[4] = {
    static_cast<int16_t>(std::lround(vx_cm_per_s)),
    static_cast<int16_t>(std::lround(vy_cm_per_s)),
    static_cast<int16_t>(std::lround(vz_cm_per_s)),
    static_cast<int16_t>(std::lround(vyaw_deg_per_s))};
  std::vector<uint8_t> payload(8);
  for (std::size_t index = 0; index < 4; ++index) {
    const uint16_t encoded = static_cast<uint16_t>(values[index]);
    payload[index * 2] = static_cast<uint8_t>(encoded & 0xFF);
    payload[index * 2 + 1] = static_cast<uint8_t>((encoded >> 8) & 0xFF);
  }
  if (!serial_comm_->send_protocol_data(
      TARGET_VELOCITY_FRAME_ID,
      static_cast<uint8_t>(payload.size()),
      payload))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Failed to send target velocity: %s",
      serial_comm_->get_last_error().c_str());
  }
}

bool UartToStm32::sendByteFrame(uint8_t frame_id, uint8_t value)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Serial port is closed; frame 0x%02X:0x%02X was not sent.",
      static_cast<unsigned>(frame_id), static_cast<unsigned>(value));
    return false;
  }
  const std::vector<uint8_t> payload{value};
  if (serial_comm_->send_protocol_data(frame_id, 1, payload)) {
    RCLCPP_INFO(
      node_->get_logger(), "Sent serial frame id=0x%02X data=0x%02X.",
      static_cast<unsigned>(frame_id), static_cast<unsigned>(value));
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed serial frame 0x%02X: %s",
      static_cast<unsigned>(frame_id), serial_comm_->get_last_error().c_str());
    return false;
  }
}

void UartToStm32::publishByteFrameResult(
  uint8_t frame_id, uint8_t value, bool success)
{
  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {frame_id, value, success ? uint8_t{1} : uint8_t{0}};
  serial_byte_command_result_pub_->publish(msg);
}

void UartToStm32::protocolDataHandler(
  uint8_t id, const std::vector<uint8_t> & data)
{
  if (id == HEIGHT_FRAME_ID) {
    if (data.size() < 2) {
      RCLCPP_WARN(node_->get_logger(), "Height frame 0x05 is shorter than 2 bytes.");
      return;
    }
    const int16_t height_cm = static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
    std_msgs::msg::Int16 msg;
    msg.data = height_cm;
    height_pub_->publish(msg);
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Published STM32 height: %d cm", height_cm);
    return;
  }

  if (id == ST_READY_QUERY_ID && !has_st_ready_pub_) {
    if (data.size() >= 2 && data[1] == 1) {
      std_msgs::msg::UInt8 msg;
      msg.data = 1;
      is_st_ready_pub_->publish(msg);
      has_st_ready_pub_ = true;
      RCLCPP_INFO(node_->get_logger(), "Flight controller reported ready.");
    }
    return;
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 10000,
    "Ignoring serial frame id=0x%02X length=%zu.",
    static_cast<unsigned>(id), data.size());
}

}  // namespace uart_to_stm32
