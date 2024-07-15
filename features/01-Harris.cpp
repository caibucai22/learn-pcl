#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <cstdlib>
#include <iostream>
#include "../my_functions.h"

using namespace std;

int main(int argc, char *argv[])
{
	printArgs(argc, argv);

	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(argv[1], *input_cloud) < 0)
	{
		PCL_ERROR("load pcd failed");
		return -1;
	}
	std::cout << "load success, the num of points is " << input_cloud->points.size() << std::endl;

	// 下采样 减少计算量
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(input_cloud);
	filter.setLeafSize(0.1f, 0.1f, 0.1f);
	filter.filter(*filter_cloud);

	cout << "before dowsampling, " << input_cloud->points.size() << " points" << endl;
	cout << "after  dowsampling, " << filter_cloud->points.size() << " points" << endl;

	float r_normal;
	float r_keypoint;

	/*r_normal = stof(argv[2]);
	r_keypoint = stof(argv[3]);*/

	r_normal = 0.1;
	r_keypoint = 0.1;

	typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

	pcl::PointCloud<pcl::PointXYZI>::Ptr harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>());

	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> *harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>;

	//
	harris_detector->setRadius(r_normal);
	harris_detector->setRadiusSearch(r_keypoint);
	harris_detector->setInputCloud(filter_cloud);
	harris_detector->compute(*harris_keypoints);

	cout << "detect harris keypoits: " << harris_keypoints->size() << endl;
	pcl::PCDWriter writer;
	writer.write("harris_keypoints.pcd", *harris_keypoints, false);

	pcl::visualization::PCLVisualizer visu("clouds");
	visu.setBackgroundColor(255, 255, 255);
	visu.addPointCloud(harris_keypoints, ColorHandlerT3(harris_keypoints, 0.0, 0.0, 255.0), "harris_keypoints");
	visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "harris_keypoints");
	visu.addPointCloud(filter_cloud, "input_cloud");
	visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "input_cloud");
	// visu.spin();

	while (!visu.wasStopped())
	{
		visu.spinOnce();
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
	return 0;
}