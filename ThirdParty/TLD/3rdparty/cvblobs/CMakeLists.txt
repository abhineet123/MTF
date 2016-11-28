project(CV_Blobs)
cmake_minimum_required(VERSION 2.6)
set(CVBLOBS_LIB_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install cvblobs library")
set(CV_BLOBS_LIB_NAME cvblobs CACHE STRING "CV_BLOBS_LIB_NAME")
add_library(${CV_BLOBS_LIB_NAME} SHARED
    blob.cpp
    BlobContour.cpp
    BlobOperators.cpp
    BlobProperties.cpp
    BlobResult.cpp
    ComponentLabeling.cpp
    blob.h
    BlobContour.h
    BlobLibraryConfiguration.h
    BlobOperators.h
    BlobProperties.h
    BlobResult.h
    ComponentLabeling.h)
install(TARGETS ${CV_BLOBS_LIB_NAME} LIBRARY DESTINATION ${CVBLOBS_LIB_INSTALL_DIR})

