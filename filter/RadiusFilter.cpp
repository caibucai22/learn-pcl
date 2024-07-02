#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>

#include "../my_functions.h"

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("could not read file\n");
		return -1;
	}
	cout << "load success, the num of points: " << cloud->points.size() << endl;

	// 半径滤波
	pcl::StopWatch time;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
	ror.setInputCloud(cloud);
	ror.setRadiusSearch(0.1);					// 半径
	ror.setMinNeighborsInRadius(500); // 设置查询点的领域点集数小于10 删除
	ror.filter(*cloud_filtered);

	// info print

	cout << "after filtering, the num of points: " << cloud_filtered->points.size() << endl;
	cout << "elapsed time: " << time.getTime() << endl;
	visualizeCloud(cloud, cloud_filtered);
	return 0;
}