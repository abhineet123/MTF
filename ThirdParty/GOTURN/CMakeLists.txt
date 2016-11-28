cmake_minimum_required(VERSION 2.8)
project(GOTURN)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

include("${CMAKE_SOURCE_DIR}/../../cmake/mtfUtils.cmake")

set(LIB_NAME goturn CACHE STRING "GOTURN_LIB_NAME")
set(LIB_DEFS  CACHE STRING "GOTURN_LIB_DEFINITIONS")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}../../cmake/Modules/" ${CMAKE_MODULE_PATH})

find_package(Boost COMPONENTS system filesystem regex REQUIRED)
find_package(TinyXML REQUIRED)
find_package( OpenCV REQUIRED )
message("Open CV version is ${OpenCV_VERSION}")

find_package(Caffe REQUIRED)
include_directories(${Caffe_INCLUDE_DIRS})

addPrefix("${LIB_DEFS}" "-D" LIB_DEFS)
message("Using LIB_DEFS: ${LIB_DEFS}")

add_definitions(${Caffe_DEFINITIONS} ${LIB_DEFS})    # ex. -DCPU_ONLY
message("Caffe_DIR is ${Caffe_DIR}")

message("LIB_NAME is ${LIB_NAME}")

add_library (${LIB_NAME} SHARED
src/helper/bounding_box.cpp
src/train/example_generator.cpp
src/helper/helper.cpp
src/helper/high_res_timer.cpp
src/helper/image_proc.cpp
src/loader/loader_alov.cpp
src/loader/loader_imagenet_det.cpp
src/loader/loader_vot.cpp
src/network/regressor.cpp
src/network/regressor_base.cpp
src/network/regressor_train.cpp
src/network/regressor_train_base.cpp
src/tracker/tracker.cpp
src/tracker/tracker_manager.cpp
src/train/tracker_trainer.cpp
src/loader/video.cpp
src/loader/video_loader.cpp
src/native/vot.cpp

include/mtf/ThirdParty/GOTURN/helper/bounding_box.h
include/mtf/ThirdParty/GOTURN/train/example_generator.h
include/mtf/ThirdParty/GOTURN/helper/helper.h
include/mtf/ThirdParty/GOTURN/helper/high_res_timer.h
include/mtf/ThirdParty/GOTURN/helper/image_proc.h
include/mtf/ThirdParty/GOTURN/loader/loader_alov.h
include/mtf/ThirdParty/GOTURN/loader/loader_imagenet_det.h
include/mtf/ThirdParty/GOTURN/loader/loader_vot.h
include/mtf/ThirdParty/GOTURN/network/regressor.h
include/mtf/ThirdParty/GOTURN/network/regressor_base.h
include/mtf/ThirdParty/GOTURN/network/regressor_train.h
include/mtf/ThirdParty/GOTURN/network/regressor_train_base.h
include/mtf/ThirdParty/GOTURN/tracker/tracker.h
include/mtf/ThirdParty/GOTURN/tracker/tracker_manager.h
include/mtf/ThirdParty/GOTURN/train/tracker_trainer.h
include/mtf/ThirdParty/GOTURN/loader/video.h
include/mtf/ThirdParty/GOTURN/loader/video_loader.h
include/mtf/ThirdParty/GOTURN/native/vot.h
)

# Add src to include directories.
include_directories(include)
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Caffe_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})

# Add all files.
#file(GLOB_RECURSE hdrs src/*.h*)
#file(GLOB_RECURSE srcs src/*.cpp)
#add_library (${PROJECT_NAME} ${srcs} ${hdrs})

# add_executable (test_tracker_vot src/test/test_tracker_vot.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Caffe_LIBRARIES})
# target_link_libraries (test_tracker_vot ${LIB_NAME})

# add_executable (test_tracker_alov src/test/test_tracker_alov.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Caffe_LIBRARIES})
# target_link_libraries (test_tracker_alov ${LIB_NAME})

# add_executable (save_videos_vot src/test/save_videos_vot.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Caffe_LIBRARIES})
# target_link_libraries (save_videos_vot ${LIB_NAME})

# add_executable (train src/train/train.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Caffe_LIBRARIES} ${TinyXML_LIBRARIES})
# target_link_libraries (train ${LIB_NAME})

# add_executable (show_tracker_vot src/visualizer/show_tracker_vot.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Caffe_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries (show_tracker_vot ${LIB_NAME})

# add_executable (show_tracker_alov src/visualizer/show_tracker_alov.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Caffe_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries (show_tracker_alov ${LIB_NAME})

# add_executable (show_imagenet src/visualizer/show_imagenet.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})
# target_link_libraries (show_imagenet ${LIB_NAME})

# add_executable (show_alov src/visualizer/show_alov.cpp)
# target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES})
# target_link_libraries (show_alov ${LIB_NAME})


