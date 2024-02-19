cmake_minimum_required(VERSION 3.0.2)
project(flec)

## Compile as C++11, supported in ROS Kinetic and newer
#Used C-17 compiler for Ceres
 add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

  roscpp
  std_msgs
  nav_msgs

)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flec
  CATKIN_DEPENDS Eigen3 roscpp std_msgs nav_msgs
  )



## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS} 
)

add_executable(odometry_extractor src/odometry_extractor_node.cpp src/OdometryExtractor.cpp)

target_link_libraries(odometry_extractor
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_dependencies(odometry_extractor ${catkin_EXPORTED_TARGETS} ${EIGEN3_LIBRARIES})

## Install executables and/or libraries

## Mark other files for installation (e.g. launch and bag files, etc.)