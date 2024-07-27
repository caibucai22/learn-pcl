/**
 * @file detect_edge.cpp
 * @author your name (you@domain.com)
 * @brief 检测边界
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024 & ref https://blog.csdn.net/suyunzzz/article/details/99610120
 *
 */

#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <chrono>

#include "../my_functions.h"

/**
 * @brief
 *
 * @param cloud
 * @param ne_radiusearch_r
 * @param be_radiusearch_r
 * @param boundary
 * @return int
 */
int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float ne_radiusearch_r, float be_radiusearch_r,
                    int k, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_boundary)
{
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);

  auto start1 = std::chrono::high_resolution_clock::now();
  // normal estimation
  ne.setInputCloud(cloud);
  // ne.setRadiusSearch(ne_radiusearch_r);
  // ne.setRadiusSearch(0.1);
  ne.setKSearch(k);
  // ne.setKSearch(10);
  ne.compute(*normals_ptr);

  std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - start1;
  std::cout << "normal estimation elapsed time: " << duration.count() << std::endl;

  std::cout << "radiusearch_r " << ne_radiusearch_r << std::endl;
  std::cout << "ksearch_k     " << k << std::endl;
  std::cout << "normals: " << normals_ptr->size() << std::endl;

  auto start2 = std::chrono::high_resolution_clock::now();
  boundEst.setInputCloud(cloud);
  boundEst.setInputNormals(normals_ptr);
  // boundEst.setRadiusSearch(be_radiusearch_r); // 可以k近邻 也可以radiussearch
  // boundEst.setRadiusSearch(0.1); // 可以k近邻 或者也可以radiussearch
  boundEst.setKSearch(k);
  // boundEst.setAngleThreshold(M_PI / 4);
  boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
  boundEst.compute(boundaries);
  std::chrono::duration<double> duration2 = std::chrono::high_resolution_clock::now() - start2;
  std::cout << "boundary estimation elapsed time: " << duration2.count() << std::endl;

  std::cout << "boundary points : " << boundaries.points.size() << std::endl;

  // 存储估计为边界的点云数据，将边界结果保存为 pcl:: PointXYZ 类型
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    if (boundaries[i].boundary_point > 0)
    {
      cloud_boundary->push_back(cloud->points[i]);
    }
  }
  auto start3 = std::chrono::high_resolution_clock::now();
  std::cout << "fuc consumed all " << std::chrono::duration<double>(start3 - start1).count() << std::endl;

  std::cout << "detect " << cloud_boundary->points.size() << " edge points" << std::endl;
  return 0;
}

void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " cloud_file_path [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h                 this help\n"
            << "--nr              radiussearch r for normal estimation\n"
            << "--br              radiussearch r for boundary estimation\n"
            << "--k               kssearch k for boundary estimation\n";
}

int main(int argc, char *argv[])
{
  // 输入点云和法线
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader; // PLYReader
  if (argc < 2 || reader.read(argv[1], *cloud) < 0)
  {
    PCL_ERROR("load pcd failed");
    printUsage(argv[0]);
    return -1;
  }
  printPointNum(cloud->points.size(), true);
  float ne_radiussearch_r = 0.1, be_radiusearch_r = 0.02;
  int k = 10;
  bool save = false;
  if (pcl::console::find_argument(argc, argv, "--nr") >= 0)
  {
    pcl::console::parse<float>(argc, argv, "--nr", ne_radiussearch_r);
  }

  if (pcl::console::find_argument(argc, argv, "--br") >= 0)
  {
    pcl::console::parse<float>(argc, argv, "--br", be_radiusearch_r);
  }

  if (pcl::console::find_argument(argc, argv, "--k") >= 0)
  {
    pcl::console::parse<int>(argc, argv, "--k", k);
  }

  if (pcl::console::find_argument(argc, argv, "--save") > 0)
  {
    save = true;
  }

  // 滤波
  pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
  estimateBorders(cloud, ne_radiussearch_r, be_radiusearch_r, k, boundary);
  if (save)
  {
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("detected_edge.pcd", *boundary, false);
  }

  visualizeCloud(cloud, boundary);

  return 0;
}