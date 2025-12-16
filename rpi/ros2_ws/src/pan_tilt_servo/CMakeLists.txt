cmake_minimum_required(VERSION 3.8)
project(pan_tilt_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(pan_tilt_servo_node
  src/pan_tilt_servo_node.cpp
)

ament_target_dependencies(pan_tilt_servo_node
  rclcpp
  std_msgs
)

# pigpiod 데몬 인터페이스 라이브러리 링크
target_link_libraries(pan_tilt_servo_node
  pigpiod_if2
  pthread
  rt
)

install(TARGETS pan_tilt_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()