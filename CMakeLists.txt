cmake_minimum_required(VERSION 2.8.8)
# suppress the warning for "install" target
if(CMAKE_MAJOR_VERSION EQUAL 3)
	cmake_policy(SET CMP0037 OLD)
endif()
project(MTF)
set (MTF_VERSION_MAJOR 1)
set (MTF_VERSION_MINOR 0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS random filesystem system)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(MTF_LIB_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install the library")
set(MTF_HEADER_INSTALL_DIR /usr/local/include CACHE PATH "Directory to install the header")

include("${MTF_SOURCE_DIR}/cmake/mtfUtils.cmake")

set(MTF_LIBS ${OpenCV_LIBS} ${Boost_LIBRARIES})
set(MTF_INCLUDE_DIRS include)
set(MTF_EXT_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
set(MTF_SRC "")
set(MTF_SPECIFIC_SRC "")
set(MTF_SPECIFIC_PROPERTIES "")
set(WARNING_FLAGS -Wfatal-errors -Wno-write-strings -Wno-unused-result -Wformat=0)
set(CT_FLAGS -std=c++11 -DNDEBUG -DEIGEN_NO_DEBUG)
set(MTF_RUNTIME_FLAGS ${WARNING_FLAGS} ${CT_FLAGS})
set(MTF_SUB_DIRS ThirdParty Macros Config Utilities AM SSM SM Examples Test)
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
# include(Test)

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

add_library (mtf SHARED ${MTF_SRC})
target_compile_options(mtf PUBLIC ${WARNING_FLAGS} ${CT_FLAGS})
target_link_libraries(mtf ${MTF_LIBS} ${LIBS_LEARNING})
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
	
install(TARGETS mtf LIBRARY DESTINATION ${MTF_LIB_INSTALL_DIR} COMPONENT lib)
add_custom_target(install_lib
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=lib"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
  )
# message(STATUS "MTF_INCLUDE_DIRS: ${MTF_INCLUDE_DIRS}")
addSuffix("${MTF_INCLUDE_DIRS}" "/" MTF_INCLUDE_DIRS)
# message(STATUS "MTF_INCLUDE_DIRS_REL: ${MTF_INCLUDE_DIRS}")
add_custom_target(mtfi DEPENDS mtf install_lib install_header mtfe)

install(DIRECTORY ${MTF_INCLUDE_DIRS}
	DESTINATION ${MTF_HEADER_INSTALL_DIR}
	COMPONENT header
	)
add_custom_target(install_header
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=header"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
  )
add_custom_target(install DEPENDS install_lib install_header)

  
# install headers while maintaining the directory structure
# installHeadersWithDir("${MTF_HEADERS}" "${MTF_HEADER_INSTALL_DIR}")
# add_custom_target(install_mtf
# COMMAND cp -f 
# )
# add_custom_target(add_lib_path
# COMMAND echo "export LIBRARY_PATH=$LIBRARY_PATH:${MTF_LIB_INSTALL_DIR}">>~/.bashrc
# COMMAND echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MTF_LIB_INSTALL_DIR}">>~/.bashrc
# )