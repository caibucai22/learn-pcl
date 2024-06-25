#include<iostream>
#include<vector>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<thread>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("Road - Cloud.pcd", *cloud) == -1)
	{
		PCL_ERROR("cloudn't read file");
		return -1;
	}

	// ½¨Á¢ kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);

	// k ½üÁÚ ËÑË÷
	pcl::PointXYZRGB searchPoint = cloud->points[1000];
	searchPoint.r = 0;
	searchPoint.g = 255;
	searchPoint.b = 0;

	int k = 100;
	vector<int> pointIdx(k);
	vector<float> pointDistance(k);

	cout << "search point: (" << searchPoint.x << ' ' << searchPoint.y << ' ' << searchPoint.z << " )" << endl;

	if (kdtree.nearestKSearch(searchPoint, k, pointIdx, pointDistance) > 0)
	{

	}

	return 0;
}