# Open The CMake GUI
# specify the source directory and the binary directory
# press configure
# set CMAKE_INSTALL_PREFIX to the path where you want to install the program
# press configure
# check BUILD_WITH_QT if you want to build the program with a QT-Config GUI
# check GENERATE_DEB_PACKAGE if you want to build a debian package (only on Linux)
#
# UNIX Makefile:
# 1) go to the binary folder and type "make" to build the project
# 2) (optional) type "make install all" to install the files into the install
#    directory
# 3) (optional) type "make package" to create a package of the install folder
#    (.tgz file if GENERATE_DEB_PACKAGE=false, .deb file if GENERATE_DEB_PACKAGE=true)
#
# Microsoft Visual C++:
# 1) open the .sln file
# 2) change the mode to "Release" -> only Release is supported!
# 3) build the project "ALL_BUILD" to build the opentld project
# 4) build the project "INSTALL" to install the files into the install
#    directory
# 5) build the project "PACKAGE" to create an NSIS-installer (NSIS is required)


project(OpenTLD)

cmake_minimum_required(VERSION 2.6)
set(CV_BLOBS_LIB_NAME cvblobs CACHE STRING "CV_BLOBS_LIB_NAME")
set(TLD_LIB_NAME opentld CACHE STRING "TLD_LIB_NAME")

if (WIN32)
	set(OPENCV_INCLUDE_DIR_WIN "C:/OpenCV/build/include" CACHE PATH "OpenCV header directory")
	set(OPENCV_LIB_DIR_WIN "C:/OpenCV/build/x86/vc14/lib" CACHE PATH "OpenCV library directory")
	set(OpenCV_SUFFIX "2410" CACHE STRING "OpenCV Suffix")
	set(OpenCV_COMPONENTS calib3d contrib core features2d flann gpu highgui imgproc legacy ml nonfree objdetect ocl photo stitching superres video videostab)
	find_package(OpenCV)
	if(NOT OpenCV_FOUND)
		set(OpenCV_INCLUDE_DIRS "${OPENCV_INCLUDE_DIR_WIN}")
		set(OpenCV_LIBRARY_DIR "${OPENCV_LIB_DIR_WIN}")
		addPrefixAndSuffix("${OpenCV_COMPONENTS}" "${OpenCV_LIBRARY_DIR}/opencv_" "${OpenCV_SUFFIX}.lib" OpenCV_LIBS)
	endif()
else()
	find_package(OpenCV REQUIRED)
endif()

set(TLD_SRC
	src/mftracker/BB.cpp
    src/mftracker/BBPredict.cpp
    src/mftracker/FBTrack.cpp
    src/mftracker/Lk.cpp
    src/mftracker/Median.cpp
    src/Clustering.cpp
    src/DetectionResult.cpp
    src/DetectorCascade.cpp
    src/EnsembleClassifier.cpp
    src/ForegroundDetector.cpp
    src/MedianFlowTracker.cpp
    src/NNClassifier.cpp
    src/TLDUtil.cpp
    src/VarianceFilter.cpp)
set(CV_BLOBS_SRC
    3rdparty/cvblobs/blob.cpp
    3rdparty/cvblobs/BlobContour.cpp
    3rdparty/cvblobs/BlobOperators.cpp
    3rdparty/cvblobs/BlobProperties.cpp
    3rdparty/cvblobs/BlobResult.cpp
    3rdparty/cvblobs/ComponentLabeling.cpp
    3rdparty/cvblobs/blob.h
    3rdparty/cvblobs/BlobContour.h
    3rdparty/cvblobs/BlobLibraryConfiguration.h
    3rdparty/cvblobs/BlobOperators.h
    3rdparty/cvblobs/BlobProperties.h
    3rdparty/cvblobs/BlobResult.h
    3rdparty/cvblobs/ComponentLabeling.h)
if(WIN32)
	add_library(${TLD_LIB_NAME} STATIC ${TLD_SRC})	
	target_compile_definitions(${TLD_LIB_NAME} PUBLIC _CRT_SECURE_NO_WARNINGS)
	add_library(${CV_BLOBS_LIB_NAME} STATIC ${CV_BLOBS_SRC})	
	set(CVBLOBS_INSTALL_DIR_DEFAULT C:/MTF/lib)
	target_compile_definitions(${CV_BLOBS_LIB_NAME} PUBLIC _CRT_SECURE_NO_WARNINGS  _SCL_SECURE_NO_WARNINGS)
else()
	add_library(${TLD_LIB_NAME} SHARED ${TLD_SRC})	
	add_library(${CV_BLOBS_LIB_NAME} SHARED ${CV_BLOBS_SRC})
	set(CVBLOBS_INSTALL_DIR_DEFAULT /usr/local/lib)
	set(CV_BLOBS_FLAGS -std=c++11 -DNDEBUG)
endif()

# set(CVBLOBS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cvblobs)
# add_subdirectory(${CVBLOBS_SRC_DIR})
# link_directories(${CVBLOBS_SRC_DIR} ${OpenCV_LIB_DIR})
# message(STATUS "CVBLOBS_SRC_DIR: ${CVBLOBS_SRC_DIR}")
# find_library(CV_BLOBS_LIB ${CV_BLOBS_LIB_NAME} HINTS ${CVBLOBS_SRC_DIR})

target_compile_options(${CV_BLOBS_LIB_NAME} PUBLIC ${CV_BLOBS_FLAGS})
target_link_libraries(${TLD_LIB_NAME} ${CV_BLOBS_LIB_NAME} ${OpenCV_LIBS})
target_include_directories(${TLD_LIB_NAME} PUBLIC include 3rdparty/cvblobs ${OpenCV_INCLUDE_DIRS})

# set_target_properties(${LIB_NAME} PROPERTIES OUTPUT_NAME ${LIB_NAME})
if(WIN32)
	install(TARGETS ${TLD_LIB_NAME} ${CV_BLOBS_LIB_NAME} ARCHIVE DESTINATION ${MTF_INSTALL_DIR})
else()	
	install(TARGETS ${TLD_LIB_NAME} ${CV_BLOBS_LIB_NAME} LIBRARY DESTINATION ${MTF_INSTALL_DIR})
endif()

