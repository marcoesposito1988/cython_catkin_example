#cython: embedsignature=True

from cython.view cimport array as cvarray

import numpy as np

cimport numpy as cnp

from cython.operator import dereference as deref
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.list cimport list
from libcpp.map cimport map

cdef extern from "cython_catkin_example.h":
    cdef cppclass CCExample:
        CCExample() except +

        list[string] getPointXYZCloudsNames()
        map[string,float] getPointXYZCloudDetails(string cloudName)
        void deletePointCloud(string cloudName)

        bool savePointCloudToFile(string cloudName, string fileName)
        bool loadPointCloudFromFile(string cloudName, string fileName)
        void loadPointCloudFromArrays(string destCloud, int n, float *xarr, float *yarr, float *zarr)
        
cdef class PyCCExample:
    cdef CCExample *thisptr
    def __cinit__(self):
        self.thisptr = new CCExample()
    def __dealloc__(self):
        del self.thisptr        

    def get_point_xyz_clouds_names(self):
        return self.thisptr.getPointXYZCloudsNames()
    def get_point_xyz_cloud_details(self,cloudName):
        return self.thisptr.getPointXYZCloudDetails(cloudName)
    def delete_point_cloud(self,name):
        self.thisptr.deletePointCloud(name)
        
    def save_point_cloud_to_file(self,cloud_name,file_name):
        return self.thisptr.savePointCloudToFile(cloud_name,file_name)
    def load_point_cloud_from_file(self,cloud_name,file_name):
        return self.thisptr.loadPointCloudFromFile(cloud_name,file_name)
    def load_array(self,dest_cloud,cnp.ndarray[float,ndim=2] arr):
        assert arr.shape[1] == 3
        assert arr.shape[0] > 0
        cdef cnp.ndarray xarr = arr[:,0]
        cdef cnp.ndarray yarr = arr[:,1]
        cdef cnp.ndarray zarr = arr[:,2]
        l = len(xarr)
        self.thisptr.loadPointCloudFromArrays(dest_cloud, l, <float*> xarr.data,<float*> yarr.data,<float*> zarr.data)
