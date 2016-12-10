cmake_minimum_required(VERSION 2.8.3)
project(mtf_bridge)

option(WITH_TEMPLATED "Enable templated implementations of search methods (faster at runtime but take much longer to compile)" OFF)
option(WITH_THIRD_PARTY "Enable Third Party Trackers" ON)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(MTF_RUNTIME_FLAGS -Wfatal-errors -Wno-write-strings -O3 -DDISABLE_XVISION -DDISABLE_PFSL3 -DDISABLE_GOTURN -DDISABLE_GRID_FEAT -DDISABLE_REGNET -DDISABLE_DFM) 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

if(NOT WITH_TEMPLATED)
message(STATUS "Templated implementations of SMs are disabled")
set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DENABLE_ONLY_NT  -DDISABLE_NN)
else()
find_package(FLANN)
find_package(HDF5)
if(FLANN_FOUND AND HDF5_FOUND) 
set(MTF_LIBS ${MTF_LIBS} ${FLANN_LIBS} ${HDF5_LIBRARIES})
else()
message(STATUS "FLANN/HDF5 library not found so the templated version NN search method has been disabled")
set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DDISABLE_NN)
endif()
endif()

if(NOT WITH_THIRD_PARTY)
message(STATUS "Third party trackers disabled")
set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DDISABLE_THIRD_PARTY_TRACKERS)
else()
option(WITH_VISP "Enable ViSP template tracker and pipeline" OFF)
if(NOT WITH_VISP)
message(STATUS "ViSP disabled")
set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DDISABLE_VISP)
endif()
option(WITH_DFT "Enable Descriptor Fields tracker (third party) " OFF)
if(NOT WITH_DFT)
message(STATUS "DFT disabled")
set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DDISABLE_DFT)
endif()
option(WITH_MIL "Enable Multiple Instance Learning based tracker (third party)" OFF)
if(NOT WITH_MIL)
message(STATUS "MIL tracker disabled")
set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DDISABLE_MIL)
endif()
endif()

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

set(MTF_LIBS mtf ${OpenCV_LIBRARIES})

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
  /usr/include
  /usr/local/include  
)

link_directories(
  /usr/lib
  /usr/local/lib
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
  ${MTF_LIBS}
)
add_dependencies(SharedImageReader ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(SharedImageWriter ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(sample_tracker_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

target_compile_options(sample_tracker_node PUBLIC ${MTF_RUNTIME_FLAGS})

# set_target_properties(sample_tracker_node PROPERTIES COMPILE_FLAGS ${MTF_RUNTIME_FLAGS}) 
