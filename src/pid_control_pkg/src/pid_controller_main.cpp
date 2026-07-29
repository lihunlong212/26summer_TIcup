#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pid_control_pkg/pid_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_pkg::PositionPIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
