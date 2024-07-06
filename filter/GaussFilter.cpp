#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/convolution_3d.h> // 高斯滤波
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

#include "../my_functions.h"

using namespace std;

int main(int argc, char *argv[])
{
	// ---------------------------------------加载数据--------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file pcd\n");
		return (-1);
	}

	// -----------------------------基于高斯核函数的卷积滤波实现---------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(0.2);									 // 高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(4); // 设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.05);						 // 设置距离阈值，若点间距离大于阈值则不予考虑
	cout << "gauss kernel made" << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	// -------------------------------设置Convolution 相关参数-----------------------------
	cout << "convolution start" << endl;
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel); // 设置卷积核
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);

	pcl::PointCloud<pcl::PointXYZ>::Ptr gassFilter(new pcl::PointCloud<pcl::PointXYZ>);
	convolution.convolve(*gassFilter);
	cout << "convolution end" << endl;

	pcl::io::savePCDFileASCII("../../_pointclouds/GS.pcd", *gassFilter);
	// -------------------------------------显示点云--------------------------------------
	visualizeCloud(cloud, gassFilter);

	return 0;
}
