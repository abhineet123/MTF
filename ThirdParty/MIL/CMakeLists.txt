cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(OBJECT_TRACKING)

FIND_PACKAGE( OpenCV REQUIRED )

# add platform-specific defines here so code can know which
# OS we're using (WIN32, UNIX, APPLE)
# :TODO:
set(LIB_NAME mil CACHE STRING "LIB_NAME")
ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
set(MIL_INCLUDE_DIRS include/mtf/ThirdParty/MIL)
add_library(${LIB_NAME} SHARED src/cv_onlineboosting.cpp ${MIL_INCLUDE_DIRS}/cv_onlineboosting.h src/cv_onlinemil.cpp ${MIL_INCLUDE_DIRS}/cv_onlinemil.h src/object_tracker.cpp ${MIL_INCLUDE_DIRS}/object_tracker.h)
target_link_libraries(${LIB_NAME} ${OpenCV_LIBRARIES})
target_include_directories(${LIB_NAME} PUBLIC include/mtf/ThirdParty/MIL)

add_executable(milApp samples/object_tracker_app.cpp)
include_directories(include)
add_dependencies(milApp ${LIB_NAME})
target_link_libraries(milApp ${OpenCV_LIBRARIES} ${LIB_NAME})
target_include_directories(milApp PUBLIC include/mtf/ThirdParty/MIL)
