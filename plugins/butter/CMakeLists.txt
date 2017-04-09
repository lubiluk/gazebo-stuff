cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(butter SHARED butter.cc)
target_link_libraries(butter ${GAZEBO_LIBRARIES})

set_property(TARGET butter PROPERTY CXX_STANDARD 14)

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )
