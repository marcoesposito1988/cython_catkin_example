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

#include <cython_catkin_example.h>

#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

map<string, float> CCExample::getPointXYZCloudDetails(string cloudName) {
    map<string, float> details;
    PCloud::ConstPtr cloud;
    try {
        cloud = pointClouds.at(cloudName);
    } catch (out_of_range) {
        return details;
    }

    details["width"] = cloud->width;
    details["height"] = cloud->height;
    details["data_size"] = cloud->data.size();

    for (unsigned int i = 0; i < cloud->fields.size(); ++i) {
        stringstream ss;
        ss << "Field " << i << " : " << cloud->fields[i] << endl;
        details[ss.str()] = 0;
    }

    return details;
}

bool CCExample::loadPointCloudFromArrays(const string& destCloud,
                                            int n,
                                            float* xarr,
                                            float* yarr,
                                            float* zarr) {
    PointCloud<PointXYZ>::Ptr temp(new PointCloud<PointXYZ>);
	for (int i = 0; i < n; i++) {
		temp->push_back(PointXYZ(xarr[i],yarr[i],zarr[i]));
	}
    pointClouds[destCloud] = generalizeCloud<PointXYZ>(temp);
    return true;
}

void CCExample::deletePointCloud(const string& cloudName) {
	pointClouds.erase(cloudName);
}

// implementation - accessible for debug

bool CCExample::loadPointCloudFromFile(const string& destCloud, const string& filename) {
    PCloud::Ptr cloud(new PCloud);
	if (io::loadPCDFile(filename, *cloud) == -1) {
		return false;
	}
	pointClouds[destCloud] = cloud;
    return true;
}

bool CCExample::savePointCloudToFile(const string& srcCloud, const string& filename) {
	PCloud::Ptr cloud;
	
	try {
        cloud = pointClouds.at(srcCloud);
    } catch (out_of_range) {
        return false;
    }
    
    if (!cloud)
        return false;
	
	std::string extension = filename.substr(filename.find_last_of (".") + 1);
    if (extension == "pcd")
        return !(io::savePCDFile(filename, *cloud));
    else
        return false;

	return true;
}

template<typename T>
typename PointCloud<T>::Ptr CCExample::specializeCloud(const PCloud::ConstPtr &srcCloud) {
    typename PointCloud<T>::Ptr ret(new PointCloud<T>);
    fromPCLPointCloud2(*srcCloud,*ret);
    return ret;
}

template<typename T>
CCExample::PCloud::Ptr CCExample::generalizeCloud(const typename pcl::PointCloud<T>::Ptr& srcCloud) {
    PCloud::Ptr ret(new PCloud);
    toPCLPointCloud2(*srcCloud,*ret);
    return ret;
}
