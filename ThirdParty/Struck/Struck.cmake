# project(struck)

# cmake_minimum_required(VERSION 2.6)

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

# if(NOT CMAKE_BUILD_TYPE)
    # set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
# endif()

# INCLUDE_DIRECTORIES (include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )
# set(STRUCK_MODULES
    # Config
    # Features
    # HaarFeature
    # HaarFeatures
    # HistogramFeatures
    # ImageRep
    # LaRank
    # MultiFeatures
    # RawFeatures
    # Sampler
    # Tracker
    # GraphUtils/GraphUtils)
# addPrefixAndSuffix("${STRUCK_MODULES}" "${CMAKE_CURRENT_LIST_DIR}/src/" ".cpp" STRUCK_SRC)
# message(STATUS "STRUCK_SRC: ${STRUCK_SRC}")
# add_library (struck SHARED ${STRUCK_SRC})

# set_target_properties(struck PROPERTIES COMPILE_FLAGS "-Wfatal-errors -Wno-write-strings -O3  -std=c++11")
# target_link_libraries(struck ${OpenCV_LIBS})
# target_include_directories(struck PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_subdirectory(${CMAKE_CURRENT_LIST_DIR})
set(THIRD_PARTY_INCLUDE_DIRS ${THIRD_PARTY_INCLUDE_DIRS} Struck/include)
set(LEARNING_TRACKERS ${LEARNING_TRACKERS} Struck/src/Struck)
set(THIRD_PARTY_LIBS ${THIRD_PARTY_LIBS} struck)
