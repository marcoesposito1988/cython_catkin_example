#!/usr/bin/env python

#import roslib; roslib.load_manifest('cython_catkin_example')
import rospy

from cython_catkin_example import cython_catkin_example

if __name__ == '__main__':
	example = cython_catkin_example.PyCCExample()
	names = example.get_point_xyz_clouds_names()
	print("Current point clouds: " + ",".join(names))