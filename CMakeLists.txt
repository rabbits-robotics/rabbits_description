cmake_minimum_required(VERSION 3.8)
project(rabbits_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rabcl REQUIRED)

include_directories(include)

# 脚リンクの派生関節を足すミキサー（motor_joint_states → joint_states）
add_executable(linkage_state_publisher src/linkage_state_publisher.cpp)
ament_target_dependencies(linkage_state_publisher rclcpp sensor_msgs geometry_msgs tf2 tf2_ros rabcl)

install(TARGETS
  linkage_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  urdf
  meshes
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_library(linkage_state_publisher_lib STATIC src/linkage_state_publisher.cpp)
  ament_target_dependencies(linkage_state_publisher_lib rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_msgs rabcl)
  target_compile_definitions(linkage_state_publisher_lib PRIVATE RABBITS_DESCRIPTION_EXCLUDE_MAIN)

  ament_add_gtest(test_linkage_state_publisher test/test_linkage_state_publisher.cpp)
  target_link_libraries(test_linkage_state_publisher linkage_state_publisher_lib)
endif()

ament_package()
