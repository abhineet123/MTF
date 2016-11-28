project(FragTrack)

cmake_minimum_required(VERSION 2.6)

find_package(OpenCV REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(FRG_LIB_NAME frg CACHE STRING "FRG_LIB_NAME")
set(FRG_SRC src/fragtrack_envelope.cpp src/Fragments_Tracker.cpp src/emd.cpp)

#INCLUDE_DIRECTORIES ( ${OpenCV_INCLUDE_DIRS})

add_executable(FragTrack ${FRG_SRC})
target_include_directories(FragTrack PUBLIC include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(FragTrack ${OpenCV_LIBS})

add_library (${FRG_LIB_NAME} SHARED ${FRG_SRC})
target_link_libraries(${FRG_LIB_NAME} ${OpenCV_LIBS})
target_include_directories(${FRG_LIB_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS})
set(FRG_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install FRG tracker library")
install(TARGETS ${FRG_LIB_NAME} LIBRARY DESTINATION ${FRG_INSTALL_DIR})