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
set(TLD_LIB_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install OpenTLD tracker library")

find_package(OpenCV REQUIRED)
# add_subdirectory(3rdparty/cvblobs)
add_library(${TLD_LIB_NAME} SHARED
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
#link_directories()
target_link_libraries(${TLD_LIB_NAME} ${CV_BLOBS_LIB_NAME} ${OpenCV_LIBS}  -L{CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cvblobs -L${OpenCV_LIB_DIR})
target_include_directories(${TLD_LIB_NAME} PUBLIC include 3rdparty/cvblobs ${OpenCV_INCLUDE_DIRS})

# set_target_properties(${LIB_NAME} PROPERTIES OUTPUT_NAME ${LIB_NAME})
install(TARGETS ${TLD_LIB_NAME} LIBRARY DESTINATION ${TLD_LIB_INSTALL_DIR})
