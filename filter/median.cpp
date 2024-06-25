#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/median_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

using namespace std;

int main3()
{
	// ---------------------------------------加载数据--------------------------------------
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("./bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file pcd\n");
		return(-1);
	}*/

	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 5;
	cloud.height = 5;
	cloud.is_dense = false;
	cloud.resize(cloud.width, cloud.height);

	for (size_t i = 0; i < 5; ++i)
	{
		cloud(i, 0).z = static_cast<float> (i + 1);
		cloud(i, 1).z = static_cast<float> (i + 6);
		cloud(i, 2).z = static_cast<float> (10 - i);
		cloud(i, 3).z = static_cast<float> (5 - i);
		cloud(i, 4).z = static_cast<float> (100);
	}
	cloud(2, 4).z = 500;

	pcl::PointCloud<pcl::PointXYZ> medianOutput;
	pcl::MedianFilter<pcl::PointXYZ> medianFilter;
	medianFilter.setInputCloud(cloud.makeShared());
	medianFilter.setWindowSize(3);
	//medianFilter.setMaxAllowedMovement(10.0);

	medianFilter.filter(medianOutput);




	//int windowsize = medianFilter.getWindowSize();
	//for (int i = 0; i < cloud->points.size(); i += windowsize)
	//{
	//	cout << "窗口 " << i / windowsize << endl;

	//	for (int j = i; j < i + windowsize; j++) {
	//		if (j % windowsize == 0) {
	//			cout << cloud->points[j] << endl;
	//		}
	//	}

	////}

	cout << "中值滤波参数" << endl;
	cout << medianFilter.getWindowSize() << " " << medianFilter.getMaxAllowedMovement() << endl;


	pcl::io::savePCDFileASCII("median.pcd", medianOutput);

	// -------------------------------------显示点云--------------------------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
	//viewer->setWindowName("中值滤波");
	//int v1(0);
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	//int v2(0);
	//viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	//viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	//viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);

	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(medianOutput, "medianFilter", v2);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "medianFilter", v2);
	////viewer->addCoordinateSystem(1.0);
	////viewer->initCameraParameters();
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//	std::this_thread::sleep_for(std::chrono::microseconds(100000));
	//}

	return 0;
}

