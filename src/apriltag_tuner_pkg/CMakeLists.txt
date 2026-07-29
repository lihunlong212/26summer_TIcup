cmake_minimum_required(VERSION 3.8)
project(apriltag_tuner_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio aruco)

add_library(apriltag_tuning_io SHARED src/tuning_io.cpp)
target_include_directories(apriltag_tuning_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(apriltag_tuning_io ${OpenCV_LIBS})

add_executable(apriltag_tuner_node src/apriltag_tuner_node.cpp)
ament_target_dependencies(apriltag_tuner_node rclcpp)
target_include_directories(apriltag_tuner_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(apriltag_tuner_node apriltag_tuning_io ${OpenCV_LIBS})

install(TARGETS
  apriltag_tuning_io
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS apriltag_tuner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tuning_io test/test_tuning_io.cpp)
  target_include_directories(test_tuning_io PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_tuning_io apriltag_tuning_io ${OpenCV_LIBS})
  set_tests_properties(test_tuning_io PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=84"
  )
endif()

ament_package()
