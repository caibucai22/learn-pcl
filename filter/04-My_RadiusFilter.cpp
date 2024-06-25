#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/common/time.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <chrono>
#include <thread>

void visualizeCloud_(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
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

int main7(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("./ro.pcd", *cloud)) {
		PCL_ERROR("could not read file\n");
		return (-1);
	}

	// 半径滤波
	pcl::StopWatch time;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
	ror.setInputCloud(cloud);
	ror.setRadiusSearch(0.1);
	ror.setMinNeighborsInRadius(10); // 设置查询点的领域点集数小于10 删除
	ror.filter(*cloud_filtered);

	// info print
	cout << "the num of points: " << cloud->points.size() << endl;
	cout << "after filtering, the num of points: " << cloud_filtered->points.size() << endl;
	cout << "time: " << time.getTime() << endl;

	visualizeCloud_(cloud, cloud_filtered);
	return 0;

}