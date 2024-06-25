#pragma once
#include<string>
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/common/time.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>


#include<chrono>
#include<thread>

// ��ӡ����
void printArgs(int argc, char** argv);

// û�л��� ����ʵ�� 

void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud);

void loadPCD(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr ptr);

// �²���
void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dest_cloud);

// ���Ȳ���
