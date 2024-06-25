/*
* @author    : Csy
* @date      : 2023-12-15 11:18
* @brief     : 点云 双边滤波 完成后 点数不变 只改变强度
*/

#include "my_functions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>

int main_bifilter()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	

	std::cout << "点云数据中所使用的有效字段为：" << pcl::getFieldsList(*cloud) << std::endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	// ----------------------------建立 kdtree
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(cloud);

	pcl::BilateralFilter<pcl::PointXYZI> bf;
	bf.setInputCloud(cloud);
	//bf.setSearchMethod();
	bf.setHalfSize(0.1);
	bf.setStdDev(0.03);
	bf.filter(*filter_cloud);
	pcl::io::savePCDFileBinary("box_filtered.pcd", *filter_cloud);

	int count = 0;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (cloud->points[i].intensity != filter_cloud->points[i].intensity)
			++count;
	}
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

	int v1(0), v2(0);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("point clouds", 10, 10, "v1_text", v1);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);

	viewer->setWindowName("双边滤波");

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>originColor(cloud, "intensity");
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>filterColor(filter_cloud, "intensity");

	viewer->addPointCloud<pcl::PointXYZI>(cloud, originColor, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZI>(filter_cloud, filterColor, "filtered cloud", v1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}

	return 0;
}