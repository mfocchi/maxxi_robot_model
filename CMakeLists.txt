cmake_minimum_required(VERSION 3.5)
project(robot_model)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_library(motionModels include/${PROJECT_NAME}/motionModels.cpp)
add_library(generalPurpose  include/${PROJECT_NAME}/generalPurpose.cpp)

add_executable(uni2diff src/unicycle_2_differential.cpp)
add_executable(unicycle_sim_node src/unicycle_sim_node.cpp)

target_link_libraries(motionModels generalPurpose)
target_link_libraries(uni2diff motionModels)
target_link_libraries(unicycle_sim_node motionModels)

target_include_directories(uni2diff PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(unicycle_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(uni2diff
  Eigen3
  rclcpp 
  geometry_msgs 
  std_msgs
  tf2
  tf2_ros)

ament_target_dependencies(unicycle_sim_node
  Eigen3
  rclcpp 
  geometry_msgs 
  std_msgs
  tf2
  tf2_ros)

install(TARGETS 
  uni2diff
  DESTINATION lib/${PROJECT_NAME}
  )

  install(TARGETS 
  unicycle_sim_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
