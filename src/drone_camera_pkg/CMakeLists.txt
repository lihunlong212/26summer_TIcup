cmake_minimum_required(VERSION 3.8)
project(drone_camera_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio aruco)

add_library(apriltag_detector SHARED src/apriltag_detector.cpp)
target_include_directories(apriltag_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(apriltag_detector ${OpenCV_LIBS})

add_executable(drone_camera_node src/drone_camera_node.cpp)
ament_target_dependencies(drone_camera_node rclcpp std_msgs)
target_include_directories(drone_camera_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(drone_camera_node apriltag_detector ${OpenCV_LIBS})

install(TARGETS
  apriltag_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS drone_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_apriltag_detector test/test_apriltag_detector.cpp)
  target_include_directories(test_apriltag_detector PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_apriltag_detector apriltag_detector ${OpenCV_LIBS})
  set_tests_properties(test_apriltag_detector PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=83"
  )
endif()

ament_package()
