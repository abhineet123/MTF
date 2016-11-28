project (CMT)
cmake_minimum_required (VERSION 2.6)
find_package(OpenCV REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(LIB_NAME cmt CACHE STRING "LIB_NAME")
set(CMT_SRC src/common.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp src/fastcluster/fastcluster.cpp)
if(WIN32)
	set(CMT_SRC ${CMT_SRC} src/getopt/getopt.cpp)
endif()

# message(STATUS "CMT_SRC: ${CMT_SRC}")

add_definitions ("-Wall")
add_library (${LIB_NAME} SHARED ${CMT_SRC})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})
target_include_directories(${LIB_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS})
set(CMT_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install CMT tracker library")
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${CMT_INSTALL_DIR})

# target_compile_options(cmt PUBLIC ${WARNING_FLAGS} ${CT_FLAGS})

# if(WIN32)
# add_library (cmt SHARED src/common.cpp Consensus.cpp Fusion.cpp Matcher.cpp Tracker.cpp
    # fastcluster/fastcluster.cpp getopt/getopt.cpp
    # )
# else()
# add_library (cmt SHARED common.cpp gui.cpp main.cpp
    # Consensus.cpp Fusion.cpp Matcher.cpp Tracker.cpp
    # fastcluster/fastcluster.cpp)
# endif()
# target_link_libraries(cmt ${OpenCV_LIBS})