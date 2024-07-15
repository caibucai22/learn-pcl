#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/median_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

using namespace std;

int main()
{
	// ---------------------------------------加载点云--------------------------------------
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("./bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file pcd\n");
		return(-1);
	}*/

	// Synthetic point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 5;
	cloud.height = 5;
	cloud.is_dense = false;
	cloud.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < 5; ++i)
	{
		cloud(i, 0).z = static_cast<float>(i + 1);
		cloud(i, 1).z = static_cast<float>(i + 6);
		cloud(i, 2).z = static_cast<float>(10 - i);
		cloud(i, 3).z = static_cast<float>(5 - i);
		cloud(i, 4).z = static_cast<float>(100);
	}
	cloud(2, 4).z = 500;

	pcl::PointCloud<pcl::PointXYZ> medianOutput;
	pcl::MedianFilter<pcl::PointXYZ> medianFilter;
	medianFilter.setInputCloud(cloud.makeShared());
	medianFilter.setWindowSize(3);

	medianFilter.filter(medianOutput);

	// int windowsize = medianFilter.getWindowSize();
	// for (int i = 0; i < cloud->points.size(); i += windowsize)
	// {
	// 	cout << "锟斤拷锟斤拷 " << i / windowsize << endl;

	// 	for (int j = i; j < i + windowsize; j++)
	// 	{
	// 		if (j % windowsize == 0)
	// 		{
	// 			cout << cloud->points[j] << endl;
	// 		}
	// 	}
	// }

	cout << "window size: " << medianFilter.getWindowSize() << " movement: " << medianFilter.getMaxAllowedMovement() << endl;

	pcl::io::savePCDFileASCII("../../_pointclouds_/median.pcd", medianOutput);

	return 0;
}
