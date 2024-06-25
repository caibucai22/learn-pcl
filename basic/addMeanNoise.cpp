/*
* @author    : Csy
* @date      : 2023-12-15 11:18
* @brief     : 向点云添加均匀分布的噪声
*/

#include "my_functions.h"
#include<pcl/common/generate.h>
#include<pcl/common/random.h>
#include<pcl/common/common.h>

int main_addmeannoise()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	loadPCD("bunny.pcd", cloud);

	pcl::PointXYZ minPt, maxPt;
	// 设置均匀分布相关参数
	float xmin = minPt.x;
	float xmax = maxPt.x;
	float ymin = minPt.y;
	float ymax = maxPt.y;
	float zmin = minPt.z;
	float zmax = maxPt.z;

	int size = (*cloud).size() * 0.1; // 生成均匀分布噪声点个数

	// ------------ 生成随机均匀分布的点云数据-----------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float>> generator;
	uint32_t seed = static_cast<uint32_t> (time(NULL));

	pcl::common::UniformGenerator<float>::Parameters x_params(xmin, xmax, seed++);
	pcl::common::UniformGenerator<float>::Parameters y_params(ymin, ymax, seed++);
	pcl::common::UniformGenerator<float>::Parameters z_params(zmin, zmax, seed++);

	generator.setParametersForX(x_params);
	generator.setParametersForY(y_params);
	generator.setParametersForZ(z_params);

	generator.fill(size, 1, *uniform_cloud);
	*uniform_cloud += *cloud;
	pcl::io::savePCDFileBinary("bunny_unifom.pcd", *uniform_cloud);

	visualizeCloud(cloud, uniform_cloud);

	return 0;
}