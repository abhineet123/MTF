cmake_minimum_required(VERSION 2.8.8)
# suppress the warning for "install" target
if(CMAKE_MAJOR_VERSION EQUAL 3)
	cmake_policy(SET CMP0037 OLD)
endif()
project(MTF)
set (MTF_VERSION_MAJOR 1)
set (MTF_VERSION_MINOR 0)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
include("${MTF_SOURCE_DIR}/cmake/mtfUtils.cmake")

if (WIN32)
	set(EIGEN_INCLUDE_DIR_WIN "C:/Eigen/include" CACHE PATH "Eigen header directory")
	set(OPENCV_INCLUDE_DIR_WIN "C:/OpenCV/build/include" CACHE PATH "OpenCV header directory")
	set(OPENCV_LIB_DIR_WIN "C:/OpenCV/build/x86/vc14/lib" CACHE PATH "OpenCV library directory")
	set(OpenCV_SUFFIX "2410" CACHE STRING "OpenCV Suffix")
	set(BOOST_INCLUDE_DIR_WIN "H:/Programming/C/boost_1_63_0" CACHE PATH "Boost header directory")
	set(BOOST_LIB_DIR_WIN "H:/Programming/C/boost_1_63_0/stage/lib" CACHE PATH "Boost library directory")
	set(Boost_SUFFIX "-vc140-mt-1_63" CACHE STRING "Boost Suffix")
	find_package(Eigen3 3.2.10)
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
	set(Boost_COMPONENTS random filesystem system)
	set(Boost_USE_STATIC_LIBS ON)
	set(DEFINITION_SWITCH "/D")
	set(INCLUDE_SWITCH "/I")
	set(LINK_SWITCH "/l")
	find_package(Boost COMPONENTS ${Boost_COMPONENTS})
	if(NOT Boost_FOUND)
		set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIR_WIN}")
		set(Boost_LIBRARY_DIRS "${BOOST_LIB_DIR_WIN}")
		addPrefixAndSuffix("${Boost_COMPONENTS}" "${Boost_LIBRARY_DIRS}/libboost_" "${Boost_SUFFIX}.lib" Boost_LIBRARIES)
	endif()
else()
	find_package(Eigen3 REQUIRED)
	find_package(OpenCV REQUIRED)
	find_package(Boost REQUIRED COMPONENTS random filesystem system)
	set(DEFINITION_SWITCH "-D")
	set(INCLUDE_SWITCH "-I")
	set(LINK_SWITCH "-l")
endif()

if(EIGEN3_FOUND)
	message(STATUS "Getting Eigen headers from: ${EIGEN3_INCLUDE_DIR}")
else(EIGEN3_FOUND)
	message(FATAL_ERROR "Eigen not found")	
endif(EIGEN3_FOUND)
if(OpenCV_FOUND)
	message(STATUS "Getting OpenCV headers from: ${OpenCV_INCLUDE_DIRS}")
	# message(STATUS "Getting OpenCV libraries from: ${OpenCV_LIBRARY_DIRS}")
	message(STATUS "Using OpenCV libraries: ${OpenCV_LIBS}")
else(OpenCV_FOUND)
	message(FATAL_ERROR "OpenCV not found")	
endif(OpenCV_FOUND)
if(Boost_FOUND)
	message(STATUS "Getting Boost headers from: ${Boost_INCLUDE_DIRS}")
	message(STATUS "Getting Boost libraries from: ${Boost_LIBRARY_DIRS}")
	message(STATUS "Using Boost libraries: ${Boost_LIBRARIES}")
else(Boost_FOUND)
	message(FATAL_ERROR "Boost not found")	
endif(Boost_FOUND)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if (WIN32)
	set(MTF_INSTALL_DIR_DEFAULT C:/MTF)
else()
	set(MTF_INSTALL_DIR_DEFAULT /usr/local)
endif()
option(WITH_TIMING "Show timing information during compilation" OFF)

set(MTF_INSTALL_DIR ${MTF_INSTALL_DIR_DEFAULT} CACHE PATH "Directory to install the library and header files")
set(MTF_LIBS "" CACHE STRING "Any additional libraries to link against that are missed by the cmake process")
set(MTF_INCLUDE_DIRS "" CACHE STRING "Any additional header directories that are missed by the cmake process")
set(MTF_DEFINITIONS "" CACHE STRING "Any additional compiler flags")

set(MTF_LIBS ${MTF_LIBS} ${OpenCV_LIBS} ${Boost_LIBRARIES})
set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} include)

set(MTF_EXT_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
set(MTF_SRC "")
set(MTF_SPECIFIC_SRC "")
set(MTF_SPECIFIC_PROPERTIES "")

set(MTF_LIB_INSTALL_DIR ${MTF_INSTALL_DIR}/lib)
set(MTF_HEADER_INSTALL_DIR ${MTF_INSTALL_DIR}/include)

if (WIN32)
	set(MTF_COMPILETIME_FLAGS /bigobj /arch:SSE2)
	option(WITH_FAST_FP "Enable fast floating point operations optimization in Visual C++" ON)
	if(WITH_FAST_FP)
		set(MTF_COMPILETIME_FLAGS ${MTF_COMPILETIME_FLAGS} /fp:fast)
	endif()
	option(WITH_FULL_OPTIMIZATION "Enable full optimization in Visual C++" OFF)
	if(WITH_FULL_OPTIMIZATION)
		set(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE} /Ox)
	endif()
	# set(WARNING_FLAGS /W4)
else()
	set(WARNING_FLAGS -Wfatal-errors -Wno-write-strings -Wno-unused-result -Wformat=0)
	set(MTF_COMPILETIME_FLAGS -std=c++11)
endif()

if(WITH_TIMING)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
endif(WITH_TIMING)

set(MTF_DEFINITIONS ${MTF_DEFINITIONS} NDEBUG EIGEN_NO_DEBUG)
# set(MTF_COMPILETIME_FLAGS ${MTF_COMPILETIME_FLAGS} ${HEADER_DEF_SWITCH}NDEBUG ${HEADER_DEF_SWITCH}EIGEN_NO_DEBUG)
set(MTF_RUNTIME_FLAGS ${WARNING_FLAGS} ${MTF_COMPILETIME_FLAGS})
set(MTF_SUB_DIRS ThirdParty Macros Config Utilities AM SSM SM Examples Diagnostics)
# set(MTF_SUB_DIRS ThirdParty)
foreach(SUB_DIR ${MTF_SUB_DIRS})
  include(${SUB_DIR}/${SUB_DIR}.cmake)
endforeach(SUB_DIR)

# include(ThirdParty/CMakeLists.txt)
# include(Macros)
# include(Config)
# include(AM)
# include(SSM)	
# include(SM)	
# include(Utilities)	
# include(Examples)
# include(Diagnostics)

# message(STATUS "SEARCH_METHODS: ${SEARCH_METHODS}")

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

# addSuffix("${BASE_CLASSES}" ".h" BASE_HEADERS)
# addPrefixAndSuffix("${SEARCH_METHODS}" "SM/" ".h" SEARCH_METHODS_HEADERS)
# addPrefixAndSuffix("${APPEARANCE_MODELS}" "AM/" ".h" APPEARANCE_MODELS_HEADERS)
# addPrefixAndSuffix("${STATE_SPACE_MODELS}" "SSM/" ".h" STATE_SPACE_MODELS_HEADERS)
# addPrefixAndSuffix("${MTF_UTILITIES}" "Utilities/" ".h" MTF_UTILITIES_HEADERS)
# addPrefixAndSuffix("${MTF_MACROS}" "Macros/" ".h" MTF_MACROS_HEADERS)
# addPrefixAndSuffix("${MTF_TOOLS}" "Tools/" ".h" MTF_TOOLS_HEADERS)
# addPrefixAndSuffix("${MTF_CONFIG}" "Config/" ".h" MTF_CONFIG_HEADERS)

# message(STATUS "SEARCH_METHODS_SRC: ${SEARCH_METHODS_SRC}")
# message(STATUS "APPEARANCE_MODELS_SRC: ${APPEARANCE_MODELS_SRC}")
# message(STATUS "STATE_SPACE_MODELS_SRC: ${STATE_SPACE_MODELS_SRC}")
# message(STATUS "MTF_UTILITIES_SRC: ${MTF_UTILITIES_SRC}")

# message(STATUS "SEARCH_METHODS_HEADERS: ${SEARCH_METHODS_HEADERS}")
# message(STATUS "APPEARANCE_MODELS_HEADERS: ${APPEARANCE_MODELS_HEADERS}")
# message(STATUS "STATE_SPACE_MODELS_HEADERS: ${STATE_SPACE_MODELS_HEADERS}")
# message(STATUS "MTF_UTILITIES_HEADERS: ${MTF_UTILITIES_HEADERS}")
# message(STATUS "MTF_MACROS_HEADERS: ${MTF_MACROS_HEADERS}")
# message(STATUS "MTF_TOOLS_HEADERS: ${MTF_TOOLS_HEADERS}")
# message(STATUS "MTF_CONFIG_HEADERS: ${MTF_CONFIG_HEADERS}")



# message(STATUS "LEARNING_SRC: ${LEARNING_SRC}")
# message(STATUS "LEARNING_HEADERS: ${LEARNING_HEADERS}")
# message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
# message(STATUS "lEARNING_INCLUDE_DIRS: ${lEARNING_INCLUDE_DIRS}")
# message(STATUS "FLANN_INCLUDE_DIR: ${FLANN_INCLUDE_DIR}")
option(WITH_OPENMP "Enable parallelization using OpenMP" OFF)
if(WITH_OPENMP)
	find_package(OpenMP)
	if (OPENMP_FOUND)
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set(MTF_DEFINITIONS ${MTF_DEFINITIONS} ENABLE_OMP)
	# else()
		# if (WIN32)
			# string(REPLACE "/openmp" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
			# string(REPLACE "/openmp" "" CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
		# endif()
	endif()
# else()
	# if (WIN32)
		# string(REPLACE "/openmp" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
		# string(REPLACE "/openmp" "" CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
	# endif()
endif()
option(WITH_SPI "Enable support for selective pixel integration" OFF)
if(NOT WITH_SPI)
set(MTF_DEFINITIONS ${MTF_DEFINITIONS} DISABLE_SPI)
endif()

if (WIN32)
	add_library (mtf STATIC ${MTF_SRC})
	set(MTF_DEFINITIONS ${MTF_DEFINITIONS} _CRT_SECURE_NO_WARNINGS)
else()
	add_library (mtf SHARED ${MTF_SRC})
endif()

target_compile_options(mtf PUBLIC ${MTF_RUNTIME_FLAGS})
target_compile_definitions(mtf PUBLIC ${MTF_DEFINITIONS})
target_link_libraries(mtf ${MTF_LIBS})
target_include_directories(mtf PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
# set_source_files_properties(ThirdParty/DSST/src/DSST.cc PROPERTIES COMPILE_FLAGS "-O2  -msse2")
# foreach(var ${in_var})
  # set_source_files_properties(ThirdParty/DSST/src/DSST.cc PROPERTIES COMPILE_FLAGS "-O2  -msse2")
# endforeach(var)
# set_source_files_properties(${SEARCH_METHODS_SRC} PROPERTIES OBJECT_DEPENDS ${APPEARANCE_MODELS_HEADERS} ${STATE_SPACE_MODELS_HEADERS} ${BASE_HEADERS} Utilities/miscUtils.h)
# set(MTF_HEADERS
	# mtf.h
 	# ${BASE_HEADERS}
	# ${SEARCH_METHODS_HEADERS}
	# ${APPEARANCE_MODELS_HEADERS}
	# ${STATE_SPACE_MODELS_HEADERS}
	# ${MTF_UTILITIES_HEADERS}
	# ${MTF_MACROS_HEADERS}
	# ${MTF_CONFIG_HEADERS}	
	# ${LEARNING_HEADERS}
	# ${TEST_HEADERS}
	# )

if(WIN32)
	install(TARGETS mtf ARCHIVE DESTINATION ${MTF_LIB_INSTALL_DIR} COMPONENT lib)
else()
	install(TARGETS mtf LIBRARY DESTINATION ${MTF_LIB_INSTALL_DIR} COMPONENT lib)
endif() 
# install(TARGETS mtf LIBRARY DESTINATION ${MTF_LIB_INSTALL_DIR} COMPONENT lib)
# message(STATUS "MTF_INCLUDE_DIRS: ${MTF_INCLUDE_DIRS}")
addSuffix("${MTF_INCLUDE_DIRS}" "/" MTF_INCLUDE_DIRS)
# message(STATUS "MTF_INCLUDE_DIRS_REL: ${MTF_INCLUDE_DIRS}")
install(DIRECTORY ${MTF_INCLUDE_DIRS}
	DESTINATION ${MTF_HEADER_INSTALL_DIR}
	COMPONENT header
	)
if(NOT WIN32)
add_custom_target(install_lib
  ${CMAKE_COMMAND}
  ${HEADER_DEF_SWITCH} "CMAKE_INSTALL_COMPONENT=lib"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
  )
add_custom_target(mtfi DEPENDS mtf install_lib install_header mtfe)
add_custom_target(install_header
  ${CMAKE_COMMAND}
  ${HEADER_DEF_SWITCH} "CMAKE_INSTALL_COMPONENT=header"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
  )
add_custom_target(install DEPENDS install_lib install_header)
endif()
  
# install headers while maintaining the directory structure
# installHeadersWithDir("${MTF_HEADERS}" "${MTF_HEADER_INSTALL_DIR}")
# add_custom_target(install_mtf
# COMMAND cp -f 
# )
# add_custom_target(add_lib_path
# COMMAND echo "export LIBRARY_PATH=$LIBRARY_PATH:${MTF_LIB_INSTALL_DIR}">>~/.bashrc
# COMMAND echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MTF_LIB_INSTALL_DIR}">>~/.bashrc
# )

if(WIN32)
	configure_file(cmake/mtfConfig.cmake.in "${PROJECT_BINARY_DIR}/mtfConfig.cmake" NEWLINE_STYLE WIN32)
else()
	configure_file(cmake/mtfConfig.cmake.in "${PROJECT_BINARY_DIR}/mtfConfig.cmake" NEWLINE_STYLE UNIX)
endif()

configure_file("${PROJECT_BINARY_DIR}/mtfConfig.cmake"  "${CMAKE_CURRENT_SOURCE_DIR}/ROS/src/mtf_bridge/cmake/mtfConfig.cmake"  COPYONLY)
# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

message(STATUS "MTF_LIBS: ${MTF_LIBS}")
message(STATUS "MTF_INCLUDE_DIRS: ${MTF_INCLUDE_DIRS}")
message(STATUS "MTF_COMPILETIME_FLAGS: ${MTF_COMPILETIME_FLAGS}")
message(STATUS "MTF_RUNTIME_FLAGS: ${MTF_RUNTIME_FLAGS}")
message(STATUS "MTF_DEFINITIIIONS: ${MTF_DEFINITIONS}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
