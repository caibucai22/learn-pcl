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

	// ������
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	// �������� ��������������Ƶ� x y ���꣬��λ�� z Ϊ 1��ƽ����
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}

	// ���ü�������� �������ü������zֵ��ʹ��ƫ��z Ϊ1��ƽ��
	cloud->points[0].z = 2.0;
	cloud->points[3].z = -2.0;
	cloud->points[6].z = 4.0;

	std::cerr << "point cloud data: " << cloud->points.size() << " points " << std::endl;

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		std::cerr << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
	}

	// �����ָ�ʱ����Ҫ��ģ��ϵ������ coefficients ���洢�ڵ�ĵ��������϶��� inliers
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ���ã�ģ��ϵ����Ҫ�Ż�
	seg.setOptimizeCoefficients(true);
	// ��Ҫ������ ���÷ָ�ģ�����ͣ�����������Ʒ��� ������ֵ �������
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);

	// ���зָ� �洢ģ��ϵ�� �� �ָ��������
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("could not estimate a planar model for the given dataset .");
		return - 1;
	}

	// ��ӡ��ƽ��ģ��
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