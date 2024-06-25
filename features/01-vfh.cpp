#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/vfh.h>

#include<pcl/filters/voxel_grid.h>
#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<ctime>
#include"my_functions.h"

/*
* @author    : Csy
* @date      : 2023-12-15 11:18
* @brief     : vfh 
*/

int main02() {
	
	cout << "hello pcl" << endl << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 读取点云
	pcl::io::loadPCDFile("bunny.pcd", *cloud);

	//下采样
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f, 0.01f, 0.01f);
	filter.filter(*filter_cloud);

	cout << "采样前：" << cloud->points.size() << " 点" << endl;
	cout << "采样后：" << filter_cloud->points.size() << " 点" << endl;
	visualizeCloud(cloud, filter_cloud);
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(filter_cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>& cloud_normals = *cloud_normals_ptr;

	ne.setRadiusSearch(0.03);
	// 法线估计计算
	auto t1 = clock();
	ne.compute(cloud_normals);
	cout << "法线计算耗时：" << double(clock() - t1) / CLOCKS_PER_SEC << " 秒" << endl;


	// vfh
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(filter_cloud);
	vfh.setInputNormals(cloud_normals_ptr);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	vfh.setSearchMethod(tree2);

	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_fe_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fphf_(new pcl::PointCloud<pcl::FPFHSignature33>);
	//vfh.setRadiusSearch(0.05);

	auto t2 = clock();
	vfh.compute(*vfh_fe_ptr);
	cout << "vfh计算耗时：" << double(clock() - t2) / CLOCKS_PER_SEC << " 秒" << endl;

	cout << vfh_fe_ptr->points.size() << endl;


	// 可视化有问题
	//fphf_ = vfh_fe_ptr;

	//pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("vfh plotter");
	//plotter->setShowLegend(true);

	//cout << pcl::getFieldsList<pcl::FPFHSignature33>(*fphf_);

	//// display
	//plotter->addFeatureHistogram<pcl::FPFHSignature33>(*fphf_, "fpfh", 0);
	//plotter->spin();
	//plotter->clearPlots();
	//system("pause");


	return 0;
}