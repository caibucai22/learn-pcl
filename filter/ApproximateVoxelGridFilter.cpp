#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h> // 体素中心滤波
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>
#include "../my_functions.h"

using namespace std;

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	if (argc < 2 || reader.read(argv[1], *cloud) < 0)
	{
		PCL_ERROR("load pcd failed.");
		return -1;
	}
	cout << "load success. the num of points: " << cloud->points.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// ApproximateVoxelGrid
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
	avg.setInputCloud(cloud);
	avg.setLeafSize(0.01f, 0.01f, 0.01f); // 设置最小体素边长
	avg.filter(*cloud_filtered);
	// ApproximateVoxelGrid
	cout << "after ApproximateVoxelGrid filtering, the num of point: " << cloud_filtered->points.size() << endl;

	// 保存结果
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("ApproximateVoxelGrid_output.pcd", *cloud_filtered, false);

	// 可视化
	visualizeCloud(cloud, cloud_filtered);

	return 0;
}