#include<pcl/point_types.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/keypoints/harris_3d.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>
#include<pcl/filters/voxel_grid.h>

#include<vector>
#include<cstdlib>
#include<iostream>
#include"my_functions.h"

using namespace std;

int main04(int argc,  char* argv[])
{
	printArgs(argc, argv);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("./bunny.pcd", *input_cloud);

	//下采样
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(input_cloud);
	filter.setLeafSize(0.01f, 0.01f, 0.01f);
	filter.filter(*filter_cloud);

	cout << "采样前：" << input_cloud->points.size() << " 点" << endl;
	cout << "采样后：" << filter_cloud->points.size() << " 点" << endl;
	//visualizeCloud(input_cloud, filter_cloud);

	pcl::PCDWriter writer;
	float r_normal;
	float r_keypoint;

	/*r_normal = stof(argv[2]);
	r_keypoint = stof(argv[3]);*/

	r_normal = 0.03;
	r_keypoint = 0.01;

	typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

	pcl::PointCloud<pcl::PointXYZI>::Ptr harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>());

	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>* harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>;

	// 
	harris_detector->setRadius(r_normal);
	harris_detector->setRadiusSearch(r_keypoint);
	harris_detector->setInputCloud(filter_cloud);

	harris_detector->compute(*harris_keypoints);

	cout << "harris keypoits " << harris_keypoints->size() << " 个" << endl;
	writer.write("harris_keypoints.pcd", *harris_keypoints,false);


	pcl::visualization::PCLVisualizer visu3("clouds");
	visu3.setBackgroundColor(255, 255, 255);
	visu3.addPointCloud(harris_keypoints, ColorHandlerT3(harris_keypoints, 0.0, 0.0, 255.0), "harris_keypoits");
	visu3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "harris_keypoints");

	visu3.addPointCloud(filter_cloud, "input_cloud");
	visu3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "input_cloud");
	visu3.spin();

	/*while (!visu3.wasStopped())
	{
		visu3.spinOnce();
		pcl_sleep(0.01);
	}*/
	return 0;
}