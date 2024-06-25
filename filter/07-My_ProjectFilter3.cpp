#include"my_functions.h"
#include<pcl/ModelCoefficients.h>
#include<pcl/filters/project_inliers.h>
#include<pcl/features/normal_3d.h>
#include<pcl/segmentation/sac_segmentation.h>

using namespace std;

int main73()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("ro.pcd", *cloud);

	/*pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(7);
	coefficients->values = { -22.0899, 102.386, 140.61,-0.00935628, 0.999937, -0.00617214,2.06343 };*/
	// ֱ��ָ������ -> ��� ���߹��� Բ����ϲ���

	// ���߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	n.setSearchMethod(tree);
	n.setInputCloud(cloud);
	n.setKSearch(20);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	// Բ�����
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.setModelType(pcl::SACMODEL_CYLINDER); // ���÷ָ�ģ��ΪԲ����ģ��
	seg.setMethodType(pcl::SAC_RANSAC); // ���ò��� ransac �㷨���� ��������
	seg.setNormalDistanceWeight(0.3);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.1);
	seg.setRadiusLimits(2.0, 3.0);

	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	seg.segment(*inliers_cylinder, *coefficients_cylinder); // ִ�зָ�

	cout << "����һ�����꣺��" << coefficients_cylinder->values[0] << ","
		<< coefficients_cylinder->values[1] << ","
		<< coefficients_cylinder->values[2] << ")"
		<< endl;

	cout << "���߷�����������" << coefficients_cylinder->values[3] << ","
		<< coefficients_cylinder->values[4] << ","
		<< coefficients_cylinder->values[5] << ")"
		<< endl;

	cout << "Բ����뾶��" << coefficients_cylinder->values[6] << endl;

	// ����ͶӰ��Բ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_CYLINDER);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients_cylinder);
	proj.filter(*cloud_projected);

	visualizeCloud(cloud, cloud_projected);

	return 0;
}