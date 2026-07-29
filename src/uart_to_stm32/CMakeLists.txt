cmake_minimum_required(VERSION 3.5)
project(uart_to_stm32)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_comm REQUIRED)

add_executable(uart_to_stm32_node
  src/uart_to_stm32_node.cpp
  src/uart_to_stm32.cpp
)

target_include_directories(uart_to_stm32_node PRIVATE
  include
)

ament_target_dependencies(uart_to_stm32_node
  rclcpp
  std_msgs
  serial_comm
)

install(TARGETS uart_to_stm32_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  serial_comm
)

ament_package()
