#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_plotter.h>

#include <time.h>
#include <iostream>
#include "../my_functions.h"

using namespace std;

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	// 读取点云
	if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("load pcd fail");
		return -1;
	}
	std::cout << "load success, the num of points is " << cloud->points.size() << std::endl;

	// 下采样
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f, 0.01f, 0.01f);
	filter.filter(*filter_cloud);

	cout << "before sampling, " << cloud->points.size() << " points" << endl;
	cout << "after  sampling, " << filter_cloud->points.size() << " points" << endl;
	visualizeCloud(cloud, filter_cloud);

	// 估计法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(filter_cloud);

	// kdtree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	// kdtreeflann 测试有问题
	// pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);

	normal_estimation.setSearchMethod(kdtree);

	// normal_estimation.setSearchMethod(tree_flann);

	normal_estimation.setRadiusSearch(0.05);
	auto t1 = clock();
	normal_estimation.compute(*normals);
	cout << "normal estimation, elapsed time " << double(clock() - t1) / CLOCKS_PER_SEC << " seconds" << endl;

	// 创建pfh 估计对象 pfh, 并输入点云数据集 cloud 和 法线 normals
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(filter_cloud);
	pfh.setInputNormals(normals);
	// 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	// 创建 kd 树 无需二次创建
	// pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
	// kdtreeflann 测试有问题
	// pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	pfh.setSearchMethod(kdtree);
	pfh.setRadiusSearch(0.05); // 注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!

	// 输出 pfh 特征
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	auto t2 = clock();
	pfh.compute(*pfhs);
	cout << "compute pfh feature, elapsed time " << double(clock() - t2) / CLOCKS_PER_SEC << "seconds" << endl;

	cout << pfhs->points.size() << endl;

	// 直方图可视化
	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*pfhs, 300);
	plotter.plot();

	return 0;
}