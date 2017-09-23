cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Plugins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -Wall")

add_library(ButterFactoryPlugin SHARED ButterFactoryPlugin.cc)
target_link_libraries(ButterFactoryPlugin ${GAZEBO_LIBRARIES})

add_library(SwingPlugin SHARED SwingPlugin.cc)
target_link_libraries(SwingPlugin ${GAZEBO_LIBRARIES})

add_library(ButterPlugin SHARED ButterPlugin.cc)
target_link_libraries(ButterPlugin ${GAZEBO_LIBRARIES})

add_library(GripPlugin SHARED GripPlugin.cc)
target_link_libraries(GripPlugin ${GAZEBO_LIBRARIES})

add_library(StickPlugin SHARED StickPlugin.cc)
target_link_libraries(StickPlugin ${GAZEBO_LIBRARIES})

add_library(ScrapePlugin SHARED ScrapePlugin.cc)
target_link_libraries(ScrapePlugin ${GAZEBO_LIBRARIES})

add_library(ShpereContainerFactoryPlugin SHARED ShpereContainerFactoryPlugin.cc)
target_link_libraries(ShpereContainerFactoryPlugin ${GAZEBO_LIBRARIES})

add_library(GrainsFactoryPlugin SHARED GrainsFactoryPlugin.cc)
target_link_libraries(GrainsFactoryPlugin ${GAZEBO_LIBRARIES})
