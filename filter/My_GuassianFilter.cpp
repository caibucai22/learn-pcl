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
	//------------��������------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ro.pcd", *cloud) == -1)
	{
		PCL_ERROR("couldn't read file pcd\n");
		return(-1);
	}
	//------------���ڸ�˹�˺����ľ���˲�ʵ��---------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4); // ��˹������׼���� �������
	kernel.setThresholdRelativeToSigma(4); // ������� Sigma �����ľ�����ֵ
	kernel.setThreshold(0.05); // ���þ�����ֵ
	cout << "kernel made" << endl;

	// KDTree ����
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	cout << "KDtree made" << endl;

	// ���� convolution ��ز���
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

	//-----------------------��ʾ����------------------------------------------
	visualizeCloud(cloud,gassFilter);

	return 0;
}