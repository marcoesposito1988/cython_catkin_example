/*
	Example of Cython and catkin integration
	Copyright (C) 2014  Marco Esposito <marcoesposito1988@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef cython_catkin_example_H
#define cython_catkin_example_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <list>
#include <string>
#include <map>

class CCExample {
	
public:
    
    typedef pcl::PCLPointCloud2 PCloud;
    
	// ------- intended python API ---------------------------------------------

    std::list<std::string> getPointXYZCloudsNames() {
        std::list<std::string> ret;
        for(std::map<std::string,PCloud::Ptr>::iterator it = pointClouds.begin(); it != pointClouds.end(); ++it) {
            ret.push_back(it->first);
        }
        return ret;
    }

    std::map<std::string, float> getPointXYZCloudDetails(std::string cloudName);
    	
	bool loadPointCloudFromArrays(const std::string& destCloud,
                                  int n,
                                  float *xarr,
                                  float *yarr,
                                  float *zarr);
	
	void deletePointCloud(const std::string& cloudName);

    bool loadPointCloudFromFile(const std::string& destCloud, const std::string& filename);
    bool savePointCloudToFile(const std::string& srcCloud, const std::string& filename);

	// ------- accessible from C++ for debugging -------------------------------

    // storage
    std::map<std::string, PCloud::Ptr> pointClouds;

    // utilities
    template<typename T>
    static typename pcl::PointCloud<T>::Ptr specializeCloud(const PCloud::ConstPtr &srcCloud);

    template<typename T>
    static PCloud::Ptr generalizeCloud(const typename pcl::PointCloud<T>::Ptr& srcCloud);

};

#endif // cython_catkin_example_H
