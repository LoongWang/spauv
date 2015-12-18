cmake_minimum_required(VERSION 2.8.3)
project(spauv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #actionlib_msgs 
   std_msgs  # Or other packages containing msgs
 )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
  cfg/bucket.cfg
   cfg/PID.cfg
   cfg/Mission.cfg
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spauv
#  CATKIN_DEPENDS actionlib actionlib_msgs cv_bridge dynamic_reconfigure image_transport message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(core src/spauv_main.cpp)
add_dependencies(core Mission_gencfg)
target_link_libraries(core ${catkin_LIBRARIES})

add_executable(controller src/spauv_controller.cpp)
add_dependencies(controller PID_gencfg)
target_link_libraries(controller ${catkin_LIBRARIES})


