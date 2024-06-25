/*
* @author    : Csy
* @date      : 2023-12-15 11:18
* @brief     : 根据双边滤波的定义 人为实现了 核心算法部分
*/

// 双边滤波 根据 点 与 其领域点的  距离和强度  计算 weight 然后 计算 weight 和 

#include"my_functions.h"
#include<pcl/kdtree/kdtree_flann.h>

inline double kernel(double x, double sigma)
{
	return (std::exp(-(x * x) / (2 * sigma * sigma)));
}

int main()
{
	std::string incloudFile = "box - Cloud.pcd";
	std::string outcloudFile = "box_filtered.pcd";

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::io::loadPCDFile(incloudFile.c_str(),*cloud);
	pcl::PointCloud<pcl::PointXYZI> bf_filtered = *cloud;

	float sigma_s = 0.1;
	float sigma_r = 0.03;

	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);

	tree->setInputCloud(cloud);

	pcl::Indices k_indices;
	std::vector<float> k_distances;

	for (int point_id = 0; point_id < cloud->size(); ++point_id)
	{
		float BF = 0;
		float W = 0;

		tree->radiusSearch(point_id, sigma_s * 2, k_indices, k_distances);

		for (std::size_t n_id = 0; n_id < k_indices.size(); ++n_id)
		{
			int id = k_indices.at(n_id);
			float dist = sqrt(k_distances.at(n_id));
			float intensity_dist = std::abs((*cloud)[point_id].intensity - (*cloud)[id].intensity);

			float w_a = kernel(dist, sigma_s);
			float w_b = kernel(intensity_dist, sigma_r);

			float weight = w_a + w_b;

			BF += weight * (*cloud)[id].intensity;
			W += weight;
		}
		bf_filtered[point_id].intensity = BF / W;
	}

	int count = 0;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (cloud->points[i].intensity != bf_filtered.points[i].intensity)
			++count;
	}
	printf("强度变换的点一共有%d个\n", count);

	// save
	pcl::io::savePCDFile(outcloudFile.c_str(), bf_filtered);

	// display

	return 0;
}