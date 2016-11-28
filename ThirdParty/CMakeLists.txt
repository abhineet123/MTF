project (MTF_THIRDPARTY)

cmake_minimum_required (VERSION 2.6)

find_package(OpenCV REQUIRED)

# message(STATUS "CMT_SRC: ${CMT_SRC}")
set(THIRD_PARTY_SUB_DIRS DSST KCF CMT RCT TLD Struck MIL DFT FRG PFSL3 ViSP Xvision)
foreach(SUB_DIR ${THIRD_PARTY_SUB_DIRS})
  include(ThirdParty/${SUB_DIR}/${SUB_DIR}.cmake)
endforeach(SUB_DIR)
add_definitions ("-Wall")
add_library (mtf_thirdparty SHARED ${MTF_THIRDPARTY_LIB_SRC})
target_link_libraries(mtf_thirdparty ${OpenCV_LIBS})
target_include_directories(mtf_thirdparty PUBLIC include ${OpenCV_INCLUDE_DIRS})
set(MTFTP_INSTALL_DIR /usr/local/lib CACHE PATH "Directory to install MTF third party trackers library")
install(TARGETS mtf_thirdparty LIBRARY DESTINATION ${MTF_THIRDPARTY_INSTALL_DIR})

