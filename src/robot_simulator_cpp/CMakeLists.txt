cmake_minimum_required(VERSION 3.8)
project(robot_simulator_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies once only
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable and dependencies
add_executable(odometry_node src/odometry_node.cpp)

ament_target_dependencies(odometry_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  custom_interfaces
)

install(TARGETS odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Final call to ament_package()
ament_package()
