cmake_minimum_required(VERSION 3.8)
project(pid_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(position_pid_controller_lib SHARED src/pid_controller.cpp)
target_include_directories(position_pid_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(position_pid_controller_lib
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

add_executable(position_pid_controller src/pid_controller_main.cpp)
target_link_libraries(position_pid_controller position_pid_controller_lib)
ament_target_dependencies(position_pid_controller rclcpp)

if(MSVC)
  target_compile_options(position_pid_controller_lib PRIVATE /W4)
else()
  target_compile_options(position_pid_controller_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS
  position_pid_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS position_pid_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_visual_descent test/test_visual_descent.cpp)
  target_link_libraries(test_visual_descent position_pid_controller_lib)
  ament_target_dependencies(test_visual_descent
    rclcpp
    std_msgs
    geometry_msgs
    tf2_ros
  )
  set_tests_properties(test_visual_descent PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=82"
  )
endif()

ament_package()
