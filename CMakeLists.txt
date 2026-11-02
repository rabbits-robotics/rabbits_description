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

# ライブラリ: gamma_linkage.yaml の読み込みと接地点の計算（他パッケージからは rabbits_description::rabbits_description）
add_library(rabbits_description SHARED
  src/linkage_params.cpp
  src/footprint.cpp
)
target_include_directories(rabbits_description PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rabbits_description rclcpp tf2 rabcl)

# 脚リンクの派生関節を足すミキサー（motor_joint_states → joint_states）
add_executable(linkage_state_publisher src/linkage_state_publisher.cpp)
target_link_libraries(linkage_state_publisher rabbits_description)
ament_target_dependencies(linkage_state_publisher rclcpp sensor_msgs geometry_msgs tf2 tf2_ros rabcl)

install(TARGETS rabbits_description
  EXPORT export_rabbits_description
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

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

ament_export_targets(export_rabbits_description HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp tf2 rabcl)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_linkage_params test/test_linkage_params.cpp)
  target_link_libraries(test_linkage_params rabbits_description)

  ament_add_gtest(test_footprint test/test_footprint.cpp)
  target_link_libraries(test_footprint rabbits_description)

  add_library(linkage_state_publisher_lib STATIC src/linkage_state_publisher.cpp)
  target_link_libraries(linkage_state_publisher_lib rabbits_description)
  ament_target_dependencies(linkage_state_publisher_lib rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_msgs rabcl)
  target_compile_definitions(linkage_state_publisher_lib PRIVATE RABBITS_DESCRIPTION_EXCLUDE_MAIN)

  ament_add_gtest(test_linkage_state_publisher test/test_linkage_state_publisher.cpp)
  target_link_libraries(test_linkage_state_publisher linkage_state_publisher_lib)
endif()

ament_package()
