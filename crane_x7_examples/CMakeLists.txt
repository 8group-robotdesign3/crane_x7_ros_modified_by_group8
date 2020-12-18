cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_examples)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs 
  OpenCV 
  cv_bridge
  image_transport
  pcl_conversions
  pcl_msgs
  pcl_ros
  actionlib
  control_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  tf
  trajectory_msgs
)

find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

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
  #INCLUDE_DIRS include
  #LIBRARIES crane_x7_examples
  CATKIN_DEPENDS message_runtime
  actionlib
  control_msgs
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  shape_msgs
  tf
  trajectory_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  )

add_executable(depth_estimater src/depth.cpp)
add_executable(locate_estimate src/locate_estimate.cpp)

target_link_libraries(depth_estimater  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(locate_estimate  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
