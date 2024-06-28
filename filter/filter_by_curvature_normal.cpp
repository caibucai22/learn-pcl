/**
 * @file filter_by_curvature_normal.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-28
 *
 * @copyright Copyright (c) 2024 & ref https://liangzhouzz.github.io/2023/09/18/%E7%82%B9%E4%BA%91%E5%A4%9A%E5%B0%BA%E5%BA%A6%E6%BB%A4%E6%B3%A2/
 *
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include "../my_functions.h"

void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " cloud_file_path \n\n";
}

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (argc < 2 || pcl::io::loadPCDFile(argv[1], *cloud_ptr) == -1)
  {
    PCL_ERROR("load pcd failed");
    printUsage(argv[0]);
    return -1;
  }

  // 法向量和曲率估计
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud(cloud_ptr);
  ne.setViewPoint(10, 10, 10);
  ne.setInputCloud(cloud_ptr);
  ne.setNumberOfThreads(4);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(1);
  ne.compute(*normals_ptr);

  // 字段连接
  pcl::PointCloud<pcl::PointNormal>::Ptr cnormals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud_ptr, *normals_ptr, *cnormals);

  // 设置不同曲率阈值 cur_threshold  遍历点云 将点云放入不同的点云集合
  pcl::PointCloud<pcl::PointXYZ>::Ptr clouddownsample_1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clouddownsample_2(new pcl::PointCloud<pcl::PointXYZ>);
  float cur_threshold = 0.001;
  for (size_t i = 0; i < cnormals->points.size(); i++)
  {
    if (cnormals->at(i).curvature > cur_threshold)
    {
      clouddownsample_1->points.emplace_back(cloud_ptr->points[i]);
    }
    else
    {
      clouddownsample_2->points.emplace_back(cloud_ptr->points[i]);
    }
  }
  //
  clouddownsample_1->width = clouddownsample_1->points.size();
  clouddownsample_1->height = 1;
  clouddownsample_1->is_dense = true;

  clouddownsample_2->width = clouddownsample_2->points.size();
  clouddownsample_2->height = 1;
  clouddownsample_2->is_dense = true;

  // 大尺度降采样
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZ>);
  float leftSize = 0.25f;
  pcl::VoxelGrid<pcl::PointXYZ> down;
  down.setInputCloud(clouddownsample_2);
  down.setLeafSize(leftSize, leftSize, leftSize);
  down.filter(*clouddownsample_2);

  *cloud_add = *clouddownsample_1 + *clouddownsample_2;

  // visualize
  visualizeCloud(cloud_ptr, cloud_add);
  return 0;
}