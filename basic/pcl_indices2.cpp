#include<iostream>
#include<pcl/ModelCoefficients.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/statistical_outlier_removal.h>

using namespace std;
int main_indices2(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	// �����������
	pcl::PCDReader reader;
	// table_scene_lms400.pcd
	reader.read("./ro.pcd", *cloud_blob);
	std::cerr << "pointcloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
	// �����˲���
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob); // �����˲� ��ĵ��� ���õ� cloud_filtered_blob


	cout << cloud_filtered_blob->width * cloud_filtered_blob->height << endl;

	// ת��Ϊģ�����
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
	cout << "pointcloud after filtering: " << cloud_filtered->points.size() << " data points" << endl;

	// ���²������ݴ������
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("temp.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	// �����˲�����
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	int i = 0, nr_points = (int)cloud_filtered->points.size();

	//��������30%��ԭʼ��������ʱ
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			break;
		}
		// �����ڲ�
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		cout << "cloud_filtered" << cloud_filtered->size() << endl;

		cout << "-----------------" << endl;
		cout << cloud_p->points.size() << " data points." << endl;
		std::stringstream ss;
		ss << "temp_" << i << ".pcd";
		writer.write(ss.str(), *cloud_p, false);
		//
		extract.setNegative(true); // ��ȡ���
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);

		i++;

	}

	return 0;
}

//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//using namespace std;
//int
//main(int argc, char** argv)
//{
//	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//	// �����������
//	pcl::PCDReader reader;
//	reader.read("table_scene_lms400.pcd", *cloud_blob);
//	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
//	// �����˲�������:ʹ��Ҷ��СΪ1cm���²���
//	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//	sor.setInputCloud(cloud_blob);
//	sor.setLeafSize(0.01f, 0.01f, 0.01f);
//	sor.filter(*cloud_filtered_blob);//�����˲�(�²���)��ĵ��Ʒ��õ�cloud_filtered_blob
//
//	cout << cloud_filtered_blob->width * cloud_filtered_blob->height << endl;
//
//	// ת��Ϊģ�����
//	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);//���²�����ĵ���ת��ΪPoinCloud����
//	cout << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points." << endl;
//	// ���²���������ݴ������
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //����һ��PointIndices�ṹ��ָ��
//	// �����ָ����
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// ��ѡ
//	seg.setOptimizeCoefficients(true); //���öԹ��Ƶ�ģ�����Ż�����
//	// ��ѡ
//	seg.setModelType(pcl::SACMODEL_PLANE);//���÷ָ�ģ�����
//	seg.setMethodType(pcl::SAC_RANSAC);//����ʹ���Ǹ�����������Ʒ���
//	seg.setMaxIterations(1000);//��������
//	seg.setDistanceThreshold(0.01);//�����Ƿ�Ϊģ���ڵ�ľ�����ֵ
//	// �����˲�������
//	pcl::ExtractIndices<pcl::PointXYZ> extract;
//	int i = 0, nr_points = (int)cloud_filtered->points.size();
//	// ��������30%ԭʼ��������ʱ
//	while (cloud_filtered->points.size() > 0.3 * nr_points)
//	{
//		// �����µĵ����зָ����ƽ����ɲ���
//		seg.setInputCloud(cloud_filtered);
//		seg.segment(*inliers, *coefficients);
//		if (inliers->indices.size() == 0)
//		{
//			cout << "Could not estimate a planar model for the given dataset." << endl;
//			break;
//		}
//		// �����ڲ�
//		extract.setInputCloud(cloud_filtered);
//		extract.setIndices(inliers);
//		extract.setNegative(false);
//		extract.filter(*cloud_p);
//		cout << "cloud_filtered: " << cloud_filtered->size() << endl;//�����ȡ֮��ʣ���
//
//		cout << "----------------------------------" << endl;
//		//����
//		cout << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << endl;
//		std::stringstream ss;
//		ss << "table_scene_lms400_plane_" << i << ".pcd"; //��ÿһ�ε���ȡ���������ļ�����
//		writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
//		// �����˲�������
//		extract.setNegative(true);//��ȡ���
//		extract.filter(*cloud_f);//��������ȡ������浽cloud_f
//		cloud_filtered.swap(cloud_f);//��cloud_filtered��cloud_f����
//
//
//		i++;
//	}
//
//	cout << "cloud_filtered: " << cloud_filtered->size() << endl;
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg1(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg2(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::io::loadPCDFile("table_scene_lms400_plane_0.pcd", *cloud_seg1);
//	pcl::io::loadPCDFile("table_scene_lms400_plane_1.pcd", *cloud_seg2);
//	pcl::io::loadPCDFile("table_scene_lms400_downsampled.pcd", *cloud_voxel);
//	/*
//	//����ȡ�������ͳ��ѧ�˲�
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
//	sor1.setInputCloud(cloud_seg2);
//	sor1.setMeanK(50);
//	sor1.setStddevMulThresh(1);
//	sor1.filter(*cloud_f);
//	cout<<cloud_f->size()<<endl;
//  */
//
//	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
//	viewer->initCameraParameters();
//
//	int v1(0);
//	viewer->createViewPort(0, 0, 0.25, 1, v1);
//	viewer->setBackgroundColor(0, 0, 255, v1);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_voxel, 244, 89, 233);
//	viewer->addPointCloud(cloud_voxel, color1, "cloud_voxel", v1);
//
//	int v2(0);
//	viewer->createViewPort(0.25, 0, 0.5, 1, v2);
//	viewer->setBackgroundColor(0, 255, 255, v2);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_seg1, 244, 89, 233);
//	viewer->addPointCloud(cloud_seg1, color2, "cloud_seg1", v2);
//
//	int v3(0);
//	viewer->createViewPort(0.5, 0, 0.75, 1, v3);
//	viewer->setBackgroundColor(34, 128, 0, v3);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud_seg2, 244, 89, 233);
//	viewer->addPointCloud(cloud_seg2, color3, "cloud_seg2", v3);
//
//	int v4(0);
//	viewer->createViewPort(0.75, 0, 1, 1, v4);
//	viewer->setBackgroundColor(0, 0, 255, v4);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color4(cloud_filtered, 244, 89, 233);
//	viewer->addPointCloud(cloud_filtered, color4, "cloud_statical", v4);
//
//	// viewer->addCoordinateSystem();//�������ϵ
//
//	viewer->spin();
//
//	return (0);
//}

