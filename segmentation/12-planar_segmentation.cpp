#include<iostream>
#include<pcl/ModelCoefficients.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>

int main121(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 填充点云
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	// 生成数据 采用随机数填充点云的 x y 坐标，都位于 z 为 1的平面上
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}

	// 设置几个局外点 重新设置几个点的z值，使其偏离z 为1的平面
	cloud->points[0].z = 2.0;
	cloud->points[3].z = -2.0;
	cloud->points[6].z = 4.0;

	std::cerr << "point cloud data: " << cloud->points.size() << " points " << std::endl;

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		std::cerr << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
	}

	// 创建分割时所需要的模型系数对象 coefficients 及存储内点的点索引集合对象 inliers
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 可选配置，模型系数需要优化
	seg.setOptimizeCoefficients(true);
	// 必要的配置 设置分割模型类型，随机参数估计方法 距离阈值 输入点云
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);

	// 进行分割 存储模型系数 和 分割点云索引
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("could not estimate a planar model for the given dataset .");
		return - 1;
	}

	// 打印出平面模型
	std::cerr << "model coefficients: " << coefficients->values[0]
		<< " " << coefficients->values[1]
		<< " " << coefficients->values[2]
		<< " " << coefficients->values[3] << " \n";

	std::cerr << " model inliers: " << inliers->indices.size() << std::endl;
	for (size_t i = 0; i < inliers->indices.size(); i++)
		std::cerr << inliers->indices[i] << " " << cloud->points[inliers->indices[i]].x
		<< " " << cloud->points[inliers->indices[i]].y
		<< " " << cloud->points[inliers->indices[i]].z <<" \n";

	return 0;
}