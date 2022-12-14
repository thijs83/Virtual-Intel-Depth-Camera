cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES pointcloud_filter
  CATKIN_DEPENDS roscpp visualization_msgs pcl_ros
)

add_executable(${PROJECT_NAME}_node src/filter.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)

if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  add_format_target(pointcloud_filter FILES ${SOURCES})
  add_tidy_target(pointcloud_filter
    FILES ${SOURCES}
    DEPENDS pointcloud_filter_node
  )
endif()
