cmake_minimum_required(VERSION 3.12)
project(flec)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  message_filters
  message_generation
  pcl_ros
)


## Add message files
# Uncomment and adjust if needed
 add_message_files(
   FILES
   svd.msg
   pcodom.msg
 )


## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  
)


## Find other required packages
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)


## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flec
  CATKIN_DEPENDS roscpp std_msgs nav_msgs message_filters message_runtime
)

## Build the node
add_executable(flec
  src/flec_node.cpp 
  src/pc_odom_manager.cpp 
  src/ceresOptimization.cpp 
  src/observability_module/observation_module.cpp
  src/utils/handlers.cpp
  src/utils/savePC2File.cpp
  src/batch_manager.cpp
)

## Link libraries
target_link_libraries(flec
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

## Add dependencies to generated messages if any
add_dependencies(flec ${catkin_EXPORTED_TARGETS})

## Installation directives if needed
