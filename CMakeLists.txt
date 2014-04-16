cmake_minimum_required(VERSION 2.8.3)
project(cython_catkin_example)

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros)

find_package(Eigen REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io filters surface kdtree visualization features)

catkin_python_setup()

catkin_package(
  DEPENDS eigen
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

# CYTHON MODULE
set(cython_catkin_example_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/cython_catkin_example)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${cython_catkin_example_ROOT}/cmake)
include(UseCython)
add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
  ${cython_catkin_example_ROOT}/cmake/ReplicatePythonSourceTree.cmake
  ${CMAKE_CURRENT_BINARY_DIR}
  WORKING_DIRECTORY ${cython_catkin_example_ROOT})

# if not set, cython_catkin_example will build as standalone project
set(cython_catkin_example_ROS TRUE)
include_directories(${cython_catkin_example_ROOT}/include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set_source_files_properties(${cython_catkin_example_ROOT}/src/cython_catkin_example.pyx PROPERTIES CYTHON_IS_CXX TRUE)
add_subdirectory(${cython_catkin_example_ROOT}/src)

install(PROGRAMS
  scripts/cython_catkin_example
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
