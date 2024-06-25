#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/search/kdtree.h>
#include<pcl/filters/convolution_3d.h>
#include "my_functions.h"


using namespace std;

/*
* 
*/

int main_gaussian()
{
	//------------加载数据------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ro.pcd", *cloud) == -1)
	{
		PCL_ERROR("couldn't read file pcd\n");
		return(-1);
	}
	//------------基于高斯核函数的卷积滤波实现---------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4); // 高斯函数标准方差 函数宽度
	kernel.setThresholdRelativeToSigma(4); // 设置相对 Sigma 参数的距离阈值
	kernel.setThreshold(0.05); // 设置距离阈值
	cout << "kernel made" << endl;

	// KDTree 建立
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	cout << "KDtree made" << endl;

	// 设置 convolution 相关参数
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>>
		convolution;

	convolution.setKernel(kernel);
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	cout << "convolution start" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr gassFilter(new pcl::PointCloud<pcl::PointXYZ>);
	convolution.convolve(*gassFilter);
	pcl::io::savePCDFileASCII("GS.pcd", *gassFilter);

	//-----------------------显示点云------------------------------------------
	visualizeCloud(cloud,gassFilter);

	return 0;
}