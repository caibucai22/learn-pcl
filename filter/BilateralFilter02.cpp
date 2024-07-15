/*
 * @author    : Csy
 * @date      : 2023-12-15 11:18
 * @brief     : 根据双边滤波的定义 人为实现了 核心算法部分; 基于法线+强度来滤波 点数不变 位置和强度会发生变化
 */

// 双边滤波 根据 点 与 其领域点的  距离和强度  计算 weight 然后 计算 weight 和

#include "../my_functions.h"
#include <pcl/kdtree/kdtree_flann.h>

inline double kernel(double x, double sigma)
{
	return (std::exp(-(x * x) / (2 * sigma * sigma)));
}

int main(int argc, char **argv)
{
	std::string outcloudFile = "bf_filter_output.pcd";

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::io::loadPCDFile(argv[1], *cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr bf_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(*cloud, *bf_filtered);

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
		(*bf_filtered)[point_id].intensity = BF / W;
	}

	int count = 0;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (cloud->points[i].intensity != (*bf_filtered).points[i].intensity)
			++count;
	}
	printf("the num of intensity changing points is %d\n", count);

	// save
	pcl::io::savePCDFile(outcloudFile.c_str(), *bf_filtered);

	// display
	visualizeCloud(cloud, bf_filtered);

	return 0;
}