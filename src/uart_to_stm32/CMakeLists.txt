cmake_minimum_required(VERSION 3.5)
project(uart_to_stm32)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial_comm REQUIRED)

add_executable(uart_to_stm32_node
  src/uart_to_stm32_node.cpp
  src/uart_to_stm32.cpp
)

target_include_directories(uart_to_stm32_node PRIVATE
  include
)

ament_target_dependencies(uart_to_stm32_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  serial_comm
)

install(TARGETS uart_to_stm32_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_library(UTIL_LIBRARY util REQUIRED)
  ament_add_gtest(test_fly_choice_serial
    test/test_fly_choice_serial.cpp
    src/uart_to_stm32.cpp
  )
  target_include_directories(test_fly_choice_serial PRIVATE
    include
  )
  ament_target_dependencies(test_fly_choice_serial
    geometry_msgs
    rclcpp
    std_msgs
    tf2
    tf2_ros
    serial_comm
  )
  target_link_libraries(test_fly_choice_serial
    ${UTIL_LIBRARY}
  )
  set_tests_properties(test_fly_choice_serial PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=85"
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  serial_comm
)

ament_package()
