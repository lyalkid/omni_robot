cmake_minimum_required(VERSION 3.8)
project(my_omni_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Kinematics executable
add_executable(kinematics src/kinematics.cpp)
target_include_directories(kinematics PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(kinematics
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
  kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  config
  rviz
  worlds
  meshes
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
