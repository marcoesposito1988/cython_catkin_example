# If the pyx file is a C++ file, we should specify that here.
set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/cython_catkin_example.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )
  
if(APPLE)
	include_directories(/System/Library/Frameworks/Python.framework/Versions/2.7/Extras/lib/python/numpy/core/include/)
endif(APPLE)

# Multi-file cython modules do not appear to be working at the moment.
cython_add_module(cython_catkin_example cython_catkin_example.pyx cython_catkin_example.cpp)

target_link_libraries(cython_catkin_example ${PCL_LIBRARIES})

if(${cython_catkin_example_ROS})
	set_target_properties(cython_catkin_example PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
	install(TARGETS cython_catkin_example
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	)
endif(${cython_catkin_example_ROS})