project(struck)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

if (WIN32)
	set(EIGEN_INCLUDE_DIR_WIN "C:/Eigen/include" CACHE PATH "Eigen header directory")
	set(OPENCV_INCLUDE_DIR_WIN "C:/OpenCV/build/include" CACHE PATH "OpenCV header directory")
	set(OPENCV_LIB_DIR_WIN "C:/OpenCV/build/x86/vc14/lib" CACHE PATH "OpenCV library directory")
	set(OpenCV_SUFFIX "2410" CACHE STRING "OpenCV Suffix")
	find_package(Eigen3)
	if(NOT EIGEN3_FOUND)
		set(EIGEN3_INCLUDE_DIR "${EIGEN_INCLUDE_DIR_WIN}")
	endif()
	set(OpenCV_COMPONENTS calib3d contrib core features2d flann gpu highgui imgproc legacy ml nonfree objdetect ocl photo stitching superres video videostab)
	find_package(OpenCV)
	if(NOT OpenCV_FOUND)
		set(OpenCV_INCLUDE_DIRS "${OPENCV_INCLUDE_DIR_WIN}")
		set(OpenCV_LIBRARY_DIR "${OPENCV_LIB_DIR_WIN}")
		addPrefixAndSuffix("${OpenCV_COMPONENTS}" "${OpenCV_LIBRARY_DIR}/opencv_" "${OpenCV_SUFFIX}.lib" OpenCV_LIBS)
	endif()
else()
	find_package(Eigen3 REQUIRED)
	find_package(OpenCV REQUIRED)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(STRK_LIB_NAME struck CACHE STRING "STRK_LIB_NAME")

INCLUDE_DIRECTORIES (include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )

set(STRK_SRC src/Config.cpp
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
    src/GraphUtils/GraphUtils.cpp)
	
if(WIN32)
	add_library (${STRK_LIB_NAME} STATIC ${STRK_SRC})
	target_compile_definitions(${STRK_LIB_NAME} PUBLIC _CRT_SECURE_NO_WARNINGS)
	set(STRUCK_INSTALL_DIR_DEFAULT C:/MTF/lib)
else()
	add_library (${STRK_LIB_NAME} SHARED ${STRK_SRC})
	set(STRUCK_INSTALL_DIR_DEFAULT /usr/local/lib)
	set_target_properties(${STRK_LIB_NAME} PROPERTIES COMPILE_FLAGS "-Wfatal-errors -Wno-write-strings -O3  -std=c++11")
endif()
target_link_libraries(${STRK_LIB_NAME} ${OpenCV_LIBS})
target_include_directories(${STRK_LIB_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
set(STRUCK_INSTALL_DIR ${STRUCK_INSTALL_DIR_DEFAULT} CACHE PATH "Directory to install Struck tracker library")
if(WIN32)
	install(TARGETS ${STRK_LIB_NAME} ARCHIVE DESTINATION ${STRUCK_INSTALL_DIR})
else()
	install(TARGETS ${STRK_LIB_NAME} LIBRARY DESTINATION ${STRUCK_INSTALL_DIR})
endif() 
