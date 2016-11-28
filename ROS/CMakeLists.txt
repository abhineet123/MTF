cmake_minimum_required(VERSION 2.8.3)
project(mtf_bridge)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  cmake_modules
)

find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Patch.msg
  PatchTrackers.msg
  Point.msg
  BufferInit.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  visual_servoing
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES mtf_bridge
  #  CATKIN_DEPENDS other_catkin_pkg
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
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  /usr/include/dc1394/
  /usr/include
  /usr/local/include/opencv
  /usr/local/include  
)

link_directories(
  /usr/lib
  /usr/X11R6/lib64
  /usr/X11R6/lib
  /lib/
)

add_library(SharedImageReader src/SharedImageReader.cpp)
add_dependencies(SharedImageReader ${catkin_EXPORTED_TARGETS})

add_executable(sample_tracker_node src/sample_tracker_node.cpp)
add_executable(SharedImageWriter src/SharedImageWriter.cpp)

target_link_libraries(SharedImageReader
  ${catkin_LIBRARIES}
 )
target_link_libraries(SharedImageWriter
  ${catkin_LIBRARIES}
  rt
)

target_link_libraries(sample_tracker_node
  SharedImageReader
  ${catkin_LIBRARIES}
  rt
  mtf
  flann
  hdf5
  ${OpenCV_LIBRARIES}
)

set_target_properties(sample_tracker_node PROPERTIES COMPILE_FLAGS "-Wfatal-errors -Wno-write-strings -O3 -D DISABLE_VISP -D DISABLE_XVISION -D DISABLE_LEARNING_TRACKERS -D DISABLE_PFSL3") 
