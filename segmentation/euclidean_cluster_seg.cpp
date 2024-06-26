/**
 * @file euclidean_cluster_seg.cpp
 * @author your name (you@domain.com)
 * @brief 使用 k近邻 进行分割 https://blog.csdn.net/qq_36686437/article/details/107583476
 * @version 0.1
 * @date 2024-03-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h> // 采样方法
#include <pcl/sample_consensus/model_types.h>  // 采样模型
#include <pcl/ModelCoefficients.h>             // 模型系数
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include<boost/thread/thread.hpp>
#include <thread>
#include <chrono>
#include <vector>

using namespace std;

int main(int argc, char **argv)
{
  // 加载
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(argv[1], *cloud) < 0)
  {
    PCL_ERROR("load pcd file failed.");
    return -1;
  }

  // 体素滤波下采样
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(0.01f, 0.01f, 0.01f);
  grid.setInputCloud(cloud);
  grid.filter(*cloud_filtered);
  cout << "after filtering, there are " << cloud_filtered->points.size() << " points remained\n";

  // 创建平面模型分割对象并设置参数
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC); // 参数估计方法 随机采样一致性
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02); // 设置符合模型的内点阈值

  // ----------- 模型分割, 直到剩余点云数量在30%以上 确保模型点云较好
  int i = 0, nr_points = (int)cloud_filtered->points.size();
  while (cloud_filtered->points.size() > 0.3 * nr_points)
  {
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      cout << "could not estimate a planar model for the given dataset" << endl;
      break;
    }
    // 根据索引提取点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // 平面模型内点
    extract.filter(*cloud_plane);
    cout << "extracted plane: " << cloud_plane->points.size() << "  points.\n";

    // 移去平面内点 提取剩余点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // 桌子平面上的点云团 使用欧式聚类对点云聚类分割
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // 欧式聚类对象
  ec.setClusterTolerance(0.02);                      // 设置搜索半径
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_all(new pcl::PointCloud<pcl::PointXYZ>);

  // 循环访问点云索引 cluster_indices
  int j = 0;
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); it++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    // 将当前聚类写入到 cloud_cluster
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cout << "pointcloud representing the cluster: " << cloud_cluster->points.size() << "  data points." << endl;
    stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write(ss.str(), *cloud_cluster, false);
    j++;

    *cloud_cluster_all += *cloud_cluster;
  }

  pcl::io::savePCDFileASCII("cloud_cluster_all.pcd", *cloud_cluster_all);

  // 点云显示
  pcl::visualization::PCLVisualizer viewer("3d viewer");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.initCameraParameters();

  // 平面上点云 红色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_handler(cloud_plane,
                                                                                      255, 0, 0);
  viewer.addPointCloud(cloud_plane, cloud_plane_handler, "cloud plane");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud plane");
  // 平面外 绿色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cluster_handler(cloud_cluster_all,
                                                                                        0, 255, 0);
  viewer.addPointCloud(cloud_cluster_all, cloud_cluster_handler, "cloud cluster");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud cluster");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }

  return 0;
}