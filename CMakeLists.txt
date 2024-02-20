cmake_minimum_required(VERSION 3.12)
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
  #Ceres
)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flec
  CATKIN_DEPENDS eigen roscpp std_msgs nav_msgs 
  )



## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS} 
)

add_executable(odometry_extractor src/odometry_extractor_node.cpp src/OdometryExtractor.cpp src/HandEyeCalibrationSolver.cpp)

target_link_libraries(odometry_extractor
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}

)

add_dependencies(odometry_extractor ${catkin_EXPORTED_TARGETS} ${EIGEN3_EXPORTED_TARGETS} ${CERES_EXPORTED_TARGETS})

## Install executables and/or libraries

## Mark other files for installation (e.g. launch and bag files, etc.)