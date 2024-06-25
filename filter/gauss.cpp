#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/convolution_3d.h>  // 高斯滤波
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

using namespace std;

int main2()
{
	// ---------------------------------------加载数据--------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("./ro.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file pcd\n");
		return(-1);
	}

	// -----------------------------基于高斯核函数的卷积滤波实现---------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(0.2);//高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(4);//设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.05);//设置距离阈值，若点间距离大于阈值则不予考虑
	cout << "Kernel made" << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	cout << "KdTree made" << endl;

	// -------------------------------设置Convolution 相关参数-----------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//设置卷积核
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	cout << "Convolution Start" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr gassFilter(new pcl::PointCloud<pcl::PointXYZ>);
	convolution.convolve(*gassFilter);
	pcl::io::savePCDFileASCII("GS.pcd", *gassFilter);
	// -------------------------------------显示点云--------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
	viewer->setWindowName("高斯滤波");
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(gassFilter, "gassFilter", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "gassFilter", v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}

	return 0;
}

