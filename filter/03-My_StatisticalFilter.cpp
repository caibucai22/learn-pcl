#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <chrono>
#include <thread>

using namespace std;

void visualizeCloud_temp(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("show cloud"));
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("point cloud", 10, 10, "v1_text", v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filter cloud", 10, 10, "v2_text", v2);

	// 按照z 字段进行渲染
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
}

int
main6() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read("./ro.pcd", *cloud);
	cout << "the num of cloud points:" << cloud->points.size() << endl;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50); // 进行统计时 考虑邻近点数
	sor.setStddevMulThresh(0.1); // 设置判断是否为离群点的阈值

	sor.filter(*cloud_filtered);
	cout << "after filtering the num of cloud points:" << cloud_filtered->points.size() << endl;
	// 保存
	pcl::PCDWriter writer;
	//writer.write("inliers.pcd", *cloud_filtered, true);
	visualizeCloud_temp(cloud, cloud_filtered);
	return 0;
}