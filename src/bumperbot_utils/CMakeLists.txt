cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher 
  rclcpp
  nav_msgs
)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  trajectory_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
