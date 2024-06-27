
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

int main07(int argc, char **argv)
{
	pcl::visualization::PCLVisualizer viewer("cloud viewer");

	pcl::visualization::CloudViewer viewer2("simple cloud viewer");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);

	viewer.addCoordinateSystem();
	viewer.addPointCloud(cloud, "cloud");
	viewer.spin();

	// 比较简单 可设置不多
	/*viewer2.showCloud(cloud);
	while (!viewer2.wasStopped())
	{

	}*/

	return 0;
}