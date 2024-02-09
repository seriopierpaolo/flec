cmake_minimum_required(VERSION 3.0.2)
project(trajectory_match)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions

find_package(PCL 1.3 REQUIRED)
find_package(Ceres REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_match
  CATKIN_DEPENDS roscpp rospy nav_msgs geometry_msgs pcl_conversions pcl_ros

)



###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)


 add_executable(odometry_pc src/odometry_pc.cpp)
 target_link_libraries(odometry_pc ${catkin_LIBRARIES} ${PCL_LIBRARIES})
 add_dependencies(odometry_pc ${PROJECT_NAME}_cpp)

 add_executable(icp src/icp.cpp)
 target_link_libraries(icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
 add_dependencies(icp ${PROJECT_NAME}_cpp)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

