cmake_minimum_required(VERSION 2.8)

find_package(OpenCV)
find_package(Caffe)
include_directories(${Caffe_INCLUDES})
add_definitions(${Caffe_DEFINITIONS}) # ex. -DCPU_ONLY

add_executable(forward my_forward.cpp)

target_link_libraries( forward ${OpenCV_LIBS} ${Caffe_LIBRARIES} ) 
