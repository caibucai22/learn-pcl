#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class KMeans
{
private:
  int m_maxIteration;
  int m_clusterNum;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_center; // 聚类中心点
public:
  KMeans(int k, int max_iteration) : m_clusterNum(k),
                                     m_maxIteration(max_iteration), m_center(new pcl::PointCloud<pcl::PointXYZ>){};
  ~KMeans(){};

  void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::Indices> &cluster_idx);
};