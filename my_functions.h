#pragma once
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// #include <boost/smart_ptr/shared_ptr.hpp>

#include <chrono>
#include <thread>

// 打印参数
void printArgs(int argc, char **argv);

// 没有基于 泛型实现

void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud);

void loadPCD(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr ptr);

// 下采样
void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dest_cloud);

// 均匀采样
