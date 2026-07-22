cmake_minimum_required(VERSION 3.8)
project(surf_multirobot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(shared_cloud_mux src/shared_cloud_mux.cpp)
ament_target_dependencies(shared_cloud_mux rclcpp sensor_msgs)

add_executable(odometry_to_tf src/odometry_to_tf.cpp)
ament_target_dependencies(odometry_to_tf rclcpp nav_msgs tf2_ros)

add_executable(map_odom_localizer src/map_odom_localizer.cpp)
ament_target_dependencies(map_odom_localizer rclcpp nav_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(link_emulator src/link_emulator.cpp)
ament_target_dependencies(link_emulator nav_msgs rclcpp surf_multirobot_msgs)

install(
  TARGETS
    shared_cloud_mux
    odometry_to_tf
    map_odom_localizer
    link_emulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch models config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/prepare_link_trace.py DESTINATION lib/${PROJECT_NAME})

ament_package()
