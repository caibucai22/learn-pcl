#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/approximate_voxel_grid.h> // 体素中心滤波
#include<pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <chrono>
#include <thread>

using namespace std;

int
main4(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read("ro.pcd", *cloud);
	cout << "the num of points: " << cloud->points.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
	avg.setInputCloud(cloud);
	avg.setLeafSize(0.01f, 0.01f, 0.01f); // 设置最小体素边长
	avg.filter(*cloud_filtered);
	cout << "approximate voxel grid filter end" << endl;
	cout << "after filter the num of point: " << cloud_filtered->points.size() << endl;
	// 保存结果
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("2f.pcd", *cloud_filtered, false);

	// 可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("show cloud"));
	int v1(0);
	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	view->setBackgroundColor(0, 0, 0, v1);
	view->addText("raw point cloud", 10, 10, "v1_text", v1);
	int v2(0);
	view->createViewPort(0.5, 0.0, 1, 1.0, v2);
	view->setBackgroundColor(0.1, 0.1, 0.1, v2);
	view->addText("filtered point cloud", 10, 10, "v2_text", v2);


	view->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	view->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "sample cloud", v2);

	while (!view->wasStopped()) {
		view->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}

	return 0;
}