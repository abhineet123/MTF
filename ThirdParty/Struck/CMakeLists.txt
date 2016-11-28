project(struck)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(STRK_LIB_NAME struck CACHE STRING "STRK_LIB_NAME")

INCLUDE_DIRECTORIES (include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )

add_library (${STRK_LIB_NAME} SHARED
    src/Config.cpp
    src/Features.cpp
    src/HaarFeature.cpp
    src/HaarFeatures.cpp
    src/HistogramFeatures.cpp
    src/ImageRep.cpp
    src/LaRank.cpp
    src/MultiFeatures.cpp
    src/RawFeatures.cpp
    src/Sampler.cpp
    src/Tracker.cpp
    src/GraphUtils/GraphUtils.cpp
    )
set_target_properties(${STRK_LIB_NAME} PROPERTIES COMPILE_FLAGS "-Wfatal-errors -Wno-write-strings -O3  -std=c++11")
target_link_libraries(${STRK_LIB_NAME} ${OpenCV_LIBS})
target_include_directories(${STRK_LIB_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
set(STRUCK_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install Struck tracker library")
install(TARGETS ${STRK_LIB_NAME} LIBRARY DESTINATION ${STRUCK_INSTALL_DIR})