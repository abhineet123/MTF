project(CV_Blobs)
cmake_minimum_required(VERSION 2.6)
set(CV_BLOBS_SRC
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
set(CV_BLOBS_LIB_NAME cvblobs CACHE STRING "CV_BLOBS_LIB_NAME")
if(WIN32)
	add_library(${CV_BLOBS_LIB_NAME} STATIC ${CV_BLOBS_SRC})	
	set(CVBLOBS_INSTALL_DIR_DEFAULT C:/MTF/lib)
	set(CV_BLOBS_FLAGS /std:c++11 /DNDEBUG)
else()
	add_library(${CV_BLOBS_LIB_NAME} SHARED ${CV_BLOBS_SRC})
	set(CVBLOBS_INSTALL_DIR_DEFAULT /usr/local/lib)
	set(CV_BLOBS_FLAGS -std=c++11 -DNDEBUG)
endif()
set(CVBLOBS_LIB_INSTALL_DIR ${CVBLOBS_INSTALL_DIR_DEFAULT} CACHE PATH "Directory to install CVBlobs tracker library")
target_compile_options(${CV_BLOBS_LIB_NAME} PUBLIC ${CV_BLOBS_FLAGS})
if(WIN32)
	install(TARGETS ${CV_BLOBS_LIB_NAME} ARCHIVE DESTINATION ${CVBLOBS_LIB_INSTALL_DIR})
else()	
	install(TARGETS ${CV_BLOBS_LIB_NAME} LIBRARY DESTINATION ${CVBLOBS_LIB_INSTALL_DIR})
endif()


