cython_catkin_example
====================

Introduction
-------------

This is a ROS package that demonstrates how to integrate Cython in the standard catkin-based ROS building system.
In particular, it demonstrates how to manipulate PCL point clouds through a minimal interface.

It is heavily based on cython-cmake-example by thewtex (https://github.com/thewtex/cython-cmake-example).


Setup
------

This is a normal catkin package.

Quick reference:

0. source the base setup.bash if needed
1. create the directories my_ros_ws and my_ros_ws/src
2. inside my_ros_ws/src execute catkin_init_workspace
3. inside my_ros_ws/src clone this repository
4. inside my_ros_ws execute catkin_make
5. you should be able to execute "rosrun cython_catkin_example example.py", which should not report any existing point cloud.

Playing around
--------------

0. source my_ros_ws/devel/setup.bash
1. execute rospython
2. in the shell, from cython_catkin_example import cython_catkin_example
3. create an object myobject = cython_catkin_example.PyCCExample()
4. load a PCL point cloud from a PCD file myobject.load_point_cloud_from_file("/path/to/file.pcd")
5. list current point clouds name_list = myobject.get_point_xyz_clouds_names()
6. get details about it myobject.get_point_xyz_cloud_details("name")

You are free to explore the rest of the few functionalities and to extend it as you please.

Hope this can be helpful.
