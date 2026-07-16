cmake_minimum_required(VERSION 3.8)
project(surf_multirobot_sim)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
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

install(
  TARGETS shared_cloud_mux odometry_to_tf map_odom_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
