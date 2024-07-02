#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

	// sor
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);						 // 进行统计时 考虑邻近点数
	sor.setStddevMulThresh(0.1); // 设置判断是否为离群点的阈值
	sor.filter(*cloud_filtered);
	// sor

	cout << "after statitical filtering, the num of cloud points:" << cloud_filtered->points.size() << endl;
	// 保存
	pcl::PCDWriter writer;
	// writer.write("inliers.pcd", *cloud_filtered, true);
	visualizeCloud(cloud, cloud_filtered);
	return 0;
}