cmake_minimum_required(VERSION 3.8)
project(swarm-robot-navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(white_detector src/white_detector.cpp)
ament_target_dependencies(white_detector rclcpp sensor_msgs std_srvs)
install(TARGETS white_detector
  DESTINATION lib/${PROJECT_NAME})

add_executable(merged_map src/merged_map.cpp)
ament_target_dependencies(merged_map rclcpp nav_msgs)
install(TARGETS merged_map DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY src launch rviz config world behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
