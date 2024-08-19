#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>
#include "../my_functions.h"
#include <random>

using namespace std;

int main(int argc, char *argv[])
{
	// 添加高斯噪声
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	loadPCD_(argv[1], cloud);

	// 设置各个方向的均值和方差
	float xmean = 0, ymean = 0, zmean = 0;
	float xstddev = 0.002, ystddev = 0.002, zstddev = 0.002;

	// 生成高斯分布的点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr gauss_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::NormalGenerator<float>> generator;
	uint32_t seed = static_cast<uint32_t>(time(NULL));

	pcl::common::NormalGenerator<float>::Parameters x_params(xmean, xstddev, seed++);
	generator.setParametersForX(x_params);
	pcl::common::NormalGenerator<float>::Parameters y_params(ymean, ystddev, seed++);
	generator.setParametersForY(y_params);
	pcl::common::NormalGenerator<float>::Parameters z_params(zmean, zstddev, seed++);
	generator.setParametersForZ(z_params);

	generator.fill((*cloud).width, (*cloud).height, *gauss_cloud);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		gauss_cloud->points[i].x += cloud->points[i].x;
		gauss_cloud->points[i].y += cloud->points[i].y;
		gauss_cloud->points[i].z += cloud->points[i].z;
	}

	//======================================
	// std::default_random_engine generator(static_cast<uint32_t>(time(NULL)));
	// std::normal_distribution<float> dist_x(xmean, xstddev);
	// std::normal_distribution<float> dist_y(ymean, ystddev);
	// std::normal_distribution<float> dist_z(zmean, zstddev);

	// gauss_cloud->width = cloud->width;
	// gauss_cloud->height = cloud->height;
	// gauss_cloud->is_dense = cloud->is_dense;
	// gauss_cloud->points.resize(cloud->points.size());

	// for (size_t i = 0; i < cloud->points.size(); ++i)
	// {
	// 	gauss_cloud->points[i].x = cloud->points[i].x + dist_x(generator);
	// 	gauss_cloud->points[i].y = cloud->points[i].y + dist_y(generator);
	// 	gauss_cloud->points[i].z = cloud->points[i].z + dist_z(generator);
	// }
	cout << "add gauss noise success" << endl;

	visualizeCloud(cloud, gauss_cloud);

	return 0;
}