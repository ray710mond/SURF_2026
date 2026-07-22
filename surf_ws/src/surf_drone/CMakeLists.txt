cmake_minimum_required(VERSION 3.8)
project(surf_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

add_library(drone_communication_core src/adaptive_mode.cpp src/voxel_codec.cpp)
target_include_directories(drone_communication_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drone_communication_core PUBLIC surf_multirobot_msgs)
target_link_libraries(drone_communication_core PUBLIC zstd::zstd)

add_executable(drone_scan_sender src/drone_scan_sender.cpp)
ament_target_dependencies(drone_scan_sender
  geometry_msgs rclcpp sensor_msgs surf_multirobot_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(drone_scan_sender drone_communication_core Threads::Threads)

install(TARGETS drone_communication_core drone_scan_sender
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_drone_communication test/test_drone_communication.cpp)
  if(TARGET test_drone_communication)
    target_link_libraries(test_drone_communication drone_communication_core)
    ament_target_dependencies(test_drone_communication surf_multirobot_msgs)
  endif()
endif()
ament_package()
