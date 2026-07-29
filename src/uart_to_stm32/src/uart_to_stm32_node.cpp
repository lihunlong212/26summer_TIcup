#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "uart_to_stm32/uart_to_stm32.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("uart_to_stm32_node");

  node->declare_parameter<std::string>("serial_port", "/dev/ttyS6");
  node->declare_parameter<int>("baud_rate", 921600);

  const auto serial_port = node->get_parameter("serial_port").as_string();
  const auto baud_rate =
    static_cast<unsigned int>(node->get_parameter("baud_rate").as_int());

  try {
    auto app = std::make_shared<uart_to_stm32::UartToStm32>(node);
    if (!app->initialize(serial_port, baud_rate)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize UartToStm32");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
