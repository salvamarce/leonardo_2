cmake_minimum_required(VERSION 3.0.2)
project(leonardo_2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
  rviz
  controller_manager
  joint_state_controller
  robot_state_publisher
  aruco_ros
  tf
  tf_conversions
)

include_directories(/usr/include/eigen3)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES leonardo_2
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable( navigation src/test_navigation.cpp src/navigation.cpp src/planner_spline.cpp)
target_link_libraries( navigation ${catkin_LIBRARIES})

add_executable( aruco_manager src/aruco_manager.cpp src/test_aruco.cpp src/navigation.cpp src/planner_spline.cpp)
target_link_libraries( aruco_manager ${catkin_LIBRARIES})

add_executable( main_leonardo src/main.cpp src/navigation.cpp src/aruco_manager.cpp src/planner_spline.cpp)
target_link_libraries( main_leonardo ${catkin_LIBRARIES})

add_executable( t265_matrix src/t265_matrix.cpp )
target_link_libraries( t265_matrix ${catkin_LIBRARIES})
