#include<pcl/point_types.h>
#include<pcl/features/pfh.h>
#include<pcl/features/normal_3d.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/filters/voxel_grid.h>

#include<time.h>
#include<iostream>
#include "my_functions.h"

using namespace std;

int main01()
{
	cout << "hello pcl" << endl<<endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	// ��ȡ����
	pcl::io::loadPCDFile("bunny.pcd", *cloud);

	//�²���
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f, 0.01f, 0.01f);
	filter.filter(*filter_cloud);

	cout << "����ǰ��" << cloud->points.size() << " ��" << endl;
	cout << "������" << filter_cloud->points.size() << " ��" << endl;
	visualizeCloud(cloud, filter_cloud);

	// �򿪵��ƹ��Ʒ���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(filter_cloud);

	// kdtree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	// kdtreeflann ����������
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);

	normal_estimation.setSearchMethod(kdtree);

	//normal_estimation.setSearchMethod(tree_flann);


	normal_estimation.setRadiusSearch(0.05);
	auto t1 = clock();
	normal_estimation.compute(*normals);
	cout << "���߹��ƺ�ʱ " << double(clock() - t1) / CLOCKS_PER_SEC << " ��" << endl;
	 
	// ����pfh ���ƶ��� pfh, ������������ݼ� cloud �� ���� normals
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(filter_cloud);
	pfh.setInputNormals(normals);
	//�������������ΪPointNormal,��ִ��pfh.setInputNormals (cloud);

	// ���� kd �� ������δ���
	//pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
	// kdtreeflann ����������
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	pfh.setSearchMethod(kdtree);

	// ��� pfh ����
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh.setRadiusSearch(0.05);

	auto t2 = clock();
	pfh.compute(*pfhs);
	cout << "���� pfh ������ʱ: " << double(clock() - t2) / CLOCKS_PER_SEC <<"��" << endl;

	cout << pfhs->points.size() << endl;
	

	return 0;
}