cmake_minimum_required(VERSION 3.8)
project(surf_humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

add_library(humanoid_voxel_decoder src/voxel_codec.cpp)
target_include_directories(humanoid_voxel_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(humanoid_voxel_decoder PUBLIC surf_multirobot_msgs)
target_link_libraries(humanoid_voxel_decoder PUBLIC zstd::zstd)

add_executable(drone_data_receiver src/drone_data_receiver.cpp)
ament_target_dependencies(drone_data_receiver rclcpp surf_multirobot_msgs)
target_link_libraries(drone_data_receiver humanoid_voxel_decoder)

install(TARGETS humanoid_voxel_decoder drone_data_receiver
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
