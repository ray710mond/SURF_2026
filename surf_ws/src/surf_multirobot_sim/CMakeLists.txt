cmake_minimum_required(VERSION 3.8)
project(surf_multirobot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

add_library(communication_core
  src/adaptive_mode.cpp
  src/voxel_codec.cpp
)
target_include_directories(communication_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(communication_core PUBLIC surf_multirobot_msgs)
target_link_libraries(communication_core PUBLIC zstd::zstd)

add_executable(shared_cloud_mux src/shared_cloud_mux.cpp)
ament_target_dependencies(shared_cloud_mux rclcpp sensor_msgs)

add_executable(odometry_to_tf src/odometry_to_tf.cpp)
ament_target_dependencies(odometry_to_tf rclcpp nav_msgs tf2_ros)

add_executable(map_odom_localizer src/map_odom_localizer.cpp)
ament_target_dependencies(map_odom_localizer rclcpp nav_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(voxel_delta_sender src/voxel_delta_sender.cpp)
ament_target_dependencies(voxel_delta_sender
  geometry_msgs rclcpp sensor_msgs surf_multirobot_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(voxel_delta_sender communication_core Threads::Threads)

add_executable(voxel_delta_receiver src/voxel_delta_receiver.cpp)
ament_target_dependencies(voxel_delta_receiver rclcpp surf_multirobot_msgs)
target_link_libraries(voxel_delta_receiver communication_core)

add_executable(link_emulator src/link_emulator.cpp)
ament_target_dependencies(link_emulator rclcpp surf_multirobot_msgs)

install(
  TARGETS
    communication_core
    shared_cloud_mux
    odometry_to_tf
    map_odom_localizer
    voxel_delta_sender
    voxel_delta_receiver
    link_emulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch worlds models config maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(PROGRAMS scripts/prepare_link_trace.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_communication_core test/test_communication_core.cpp)
  if(TARGET test_communication_core)
    target_link_libraries(test_communication_core communication_core)
    ament_target_dependencies(test_communication_core surf_multirobot_msgs)
  endif()
endif()

ament_package()
