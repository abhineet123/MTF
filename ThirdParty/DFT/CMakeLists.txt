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


project(DFT)

cmake_minimum_required(VERSION 2.6)
set(DFT_LIB_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install DFT tracker library")

set(CFLAGS  -Wfatal-errors -std=c++11 -O3 -DNDEBUG -DEIGEN_NO_DEBUG -ffast-math -fopenmp -fPIC)
set(LDFLAGS -fopenmp)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

add_library(dft SHARED
src/Homography.cpp
src/IterativeOptimization.cpp
src/Utilities.cpp
)	
target_compile_options(dft PUBLIC  ${CFLAGS}  ${LDFLAGS})
target_include_directories(dft PUBLIC include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(dft ${OpenCV_LIBS} -L${OpenCV_LIB_DIR})
install(TARGETS dft LIBRARY DESTINATION ${DFT_LIB_INSTALL_DIR})

add_executable(homographyDemo src/ISMAR_Demo/main.cpp)
target_compile_options(homographyDemo PUBLIC  ${CFLAGS}  ${LDFLAGS})
target_link_libraries(homographyDemo dft ${OpenCV_LIBS} -L${OpenCV_LIB_DIR})
target_include_directories(homographyDemo PUBLIC include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

