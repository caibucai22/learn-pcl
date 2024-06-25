#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>

#include<iostream>
#include<vector>
#include<ctime>


int main()
{
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// generate pointcloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		(*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		(*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		(*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;

	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	int k = 10;
	std::vector<int> pointIdxKNNSearch(k);
	std::vector<float> pointKNNSquaredDistance(k);

	std::cout << "k nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< " " << k << std::endl;

	if (kdtree.radiusSearch(searchPoint, k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	{
		for(std::size_t i= 0 ; i<pointIdxKNNSearch.size();++i)
			std::cout << "   " << (*cloud)[pointIdxKNNSearch[i]].x
			<< " " << (*cloud)[pointIdxKNNSearch[i]].y
			<< " " << (*cloud)[pointIdxKNNSearch[i]].z
			<< " square distance: " << pointKNNSquaredDistance[i] << std::endl;
	}

	// neighbors within radius search

	std::vector<int> pointIdxRadiusSearch(k);
	std::vector<float> pointRadiusSquaredDistance(k);

	std::cout << "radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< " " << k << std::endl;

	if (kdtree.nearestKSearch(searchPoint, k, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "   " << (*cloud)[pointIdxRadiusSearch[i]].x
			<< " " << (*cloud)[pointIdxRadiusSearch[i]].y
			<< " " << (*cloud)[pointIdxRadiusSearch[i]].z
			<< " square distance: " << pointRadiusSquaredDistance[i] << std::endl;
	}

	return 0;
}