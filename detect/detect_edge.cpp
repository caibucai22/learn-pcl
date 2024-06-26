/**
 * @file detect_edge.cpp
 * @author your name (you@domain.com)
 * @brief 检测边界
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

int main()
{
  // 输入点云和法线
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
  pcl::PLYReader reader;
  reader.read("*.ply", *cloud);
  reader.read("*.ply", *normal);

  // 计算边界
  pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
  boundaries->resize(cloud->size()); // 初始化大小
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation;
  boundary_estimation.setInputCloud(cloud);
  boundary_estimation.setInputNormals(normal);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
  boundary_estimation.setSearchMethod(kdtree_ptr);
  boundary_estimation.setKSearch(30); // 设置k近邻数量
  boundary_estimation.setAngleThreshold(M_PI * 0.85);
  boundary_estimation.compute(*boundaries);

  // 可视化
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_visual->resize(cloud->size());

  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud_visual->points[i].x = cloud->points[i].x;
    cloud_visual->points[i].y = cloud->points[i].y;
    cloud_visual->points[i].z = cloud->points[i].z;
    if (boundaries->points[i].boundary_point != 0)
    {
      cloud_visual->points[i].r = 255;
      cloud_visual->points[i].g = 0;
      cloud_visual->points[i].b = 0;
    }
    else
    {
      cloud_visual->points[i].r = 255;
      cloud_visual->points[i].g = 255;
      cloud_visual->points[i].b = 255;
    }
  }
  pcl::visualization::CloudViewer viewer("view");
  viewer.showCloud(cloud_visual);

  return 0;
}