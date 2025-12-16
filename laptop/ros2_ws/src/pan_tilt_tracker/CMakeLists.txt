cmake_minimum_required(VERSION 3.8)
project(pan_tilt_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pan_tilt_tracker_node
  src/pan_tilt_tracker_node.cpp
)

target_include_directories(pan_tilt_tracker_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pan_tilt_tracker_node
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(pan_tilt_tracker_node
  rclcpp
  std_msgs
  vision_msgs
  sensor_msgs
  cv_bridge
)

install(TARGETS
  pan_tilt_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
