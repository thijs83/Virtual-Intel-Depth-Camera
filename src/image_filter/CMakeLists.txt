cmake_minimum_required(VERSION 3.0.2)
project(image_filter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Python3 COMPONENTS NumPy)

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs
  DEPENDS OpenCV Python3
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/image_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(image_filter FILES ${PYSOURCES})
endif()
