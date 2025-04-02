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
  message_filters
  message_generation
  #Ceres
)



## Add message files
#add_message_files(
#  FILES
#  svd.msg
#)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flec
  CATKIN_DEPENDS eigen roscpp std_msgs nav_msgs message_filters message_runtime
  )



## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS} 
)

add_executable(flec src/flec_node.cpp 
                    src/odometry_extractor.cpp 
                    src/ceresOptimization.cpp 
                    #src/transform_Publisher.cpp
                    src/observability_module/observation_module.cpp
                    ##src/observability_module/svd_Publisher.cpp
                    src/utils/handlers.cpp
                    src/batch_manager.cpp)



target_link_libraries(flec
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}

)

add_dependencies(flec ${catkin_EXPORTED_TARGETS} ${EIGEN3_EXPORTED_TARGETS} ${CERES_EXPORTED_TARGETS})

## Install executables and/or libraries

## Mark other files for installation (e.g. launch and bag files, etc.)