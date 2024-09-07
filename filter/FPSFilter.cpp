/**
 * @file FPSFilter.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-07-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/console/parse.h>

#include <vector>
#include <limits>
#include <iostream>
#include "../my_functions.h"

void my_fps(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr out_sample_cloud, int sample_size)
{
  std::vector<int> choseIdx;
  std::vector<float> remain2choseDistances(cloud->size(), std::numeric_limits<float>::max());
  // random chose, chose first here
  choseIdx.push_back(0);

  for (int i = 1; i < sample_size; i++)
  {
    int cur_loop_idx = -1;
    float max_mindistance = -1;
    pcl::PointXYZ lastP = cloud->points[choseIdx.back()];
    for (int j = 0; j < cloud->size(); j++)
    {
      pcl::PointXYZ curP = cloud->points[j];
      float dis = sqrt(pow(curP.x - lastP.x, 2) + pow(curP.y - lastP.y, 2) +
                       pow(curP.z - lastP.z, 2));
      if (dis < remain2choseDistances[j])
      {
        remain2choseDistances[j] = dis;
      }

      if (remain2choseDistances[j] > max_mindistance)
      {
        max_mindistance = remain2choseDistances[j];
        cur_loop_idx = j;
      }
    }
    choseIdx.push_back(cur_loop_idx);
  }

  pcl::PointIndices::Ptr chosedIndice(new pcl::PointIndices);
  chosedIndice->indices = choseIdx;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(chosedIndice);
  extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
  extract.filter(*out_sample_cloud);
}

void my_fps_max(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr out_sample_cloud, int sample_size)
{
  std::vector<int> choseIdx;
  std::vector<float> remain2choseDistances(cloud->size(), std::numeric_limits<float>::min());
  // random chose, chose first here
  choseIdx.push_back(0);

  for (int i = 1; i < sample_size; i++)
  {
    int cur_loop_idx = -1;
    float max_mindistance = -1; // max_max_distance
    pcl::PointXYZ lastP = cloud->points[choseIdx.back()];
    for (int j = 0; j < cloud->size(); j++)
    {
      pcl::PointXYZ curP = cloud->points[j];
      float dis = sqrt(pow(curP.x - lastP.x, 2) + pow(curP.y - lastP.y, 2) +
                       pow(curP.z - lastP.z, 2));
      if (dis > remain2choseDistances[j]) // 这样会有问题 会被 最远的点距离 , 无法捕捉到点云结构
      {
        remain2choseDistances[j] = dis;
      }

      if (remain2choseDistances[j] > max_mindistance)
      {
        max_mindistance = remain2choseDistances[j];
        cur_loop_idx = j;
      }
    }
    choseIdx.push_back(cur_loop_idx);
  }

  pcl::PointIndices::Ptr chosedIndice(new pcl::PointIndices);
  chosedIndice->indices = choseIdx;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(chosedIndice);
  extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
  extract.filter(*out_sample_cloud);
}

void farthestPointSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr out_sample_cloud, int sample_size)
{
  pcl::PointIndices::Ptr sample_indices(new pcl::PointIndices);

  sample_indices->indices.push_back(0);

  std::vector<float> minDistances(cloud->size(), std::numeric_limits<float>::max());

  for (int i = 1; i < sample_size; i++)
  {
    int farthestPointIdx = -1;
    float maxDistance = -1;

    for (size_t j = 0; j < cloud->size(); j++)
    {
      float distance = pcl::euclideanDistance(cloud->points[sample_indices->indices.back()],
                                              cloud->points[j]);
      // 更新该点到采样点集的最小距离
      if (distance < minDistances[j])
      {
        minDistances[j] = distance;
      }

      // 找距离最远的点
      if (minDistances[j] > maxDistance)
      {
        maxDistance = minDistances[j];
        farthestPointIdx = j;
      }
    }
    sample_indices->indices.push_back(farthestPointIdx);
  }
  // 根据采样点索引构建采样点云
  out_sample_cloud->width = sample_size;
  out_sample_cloud->height = 1;
  out_sample_cloud->is_dense = true;
  out_sample_cloud->points.resize(sample_size);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(sample_indices);
  extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
  extract.filter(*out_sample_cloud);
}

void printUsage(const char *progname)
{
  std::cout << "\n\nUsage: " << progname << " pcd --points 1000 [--save]\n"
            << "Usage: " << progname << " pcd --ratio  0.15 [--save]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h                 this help\n"
            << "--points           basic angle for spliting, default 30 \n"
            << "--ratio            ratio * num of points\n"
            << "--save             save result\n"
            << "--myimpl           execute my implement func\n";
}

int main(int argc, char **argv)
{

  int points = 1000;
  bool is_based_points = true;
  float ratio = 0.15;
  bool is_based_ratio = false;
  bool is_saved = false;
  bool call_myimpl = false;
  if (pcl::console::find_argument(argc, argv, "--points") > 0)
  {
    pcl::console::parse<int>(argc, argv, "--points", points);
    is_based_ratio = false;
  }
  if (pcl::console::find_argument(argc, argv, "--ratio") > 0)
  {
    pcl::console::parse<float>(argc, argv, "--ratio", ratio);
    is_based_points = false;
    is_based_ratio = true;
  }
  if (pcl::console::find_argument(argc, argv, "--save") > 0)
  {
    is_saved = true;
  }
  if (pcl::console::find_argument(argc, argv, "--myimpl") > 0)
  {
    call_myimpl = true;
  }

  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
  {
    printUsage(argv[0]);
    PCL_ERROR("load pcd failed");
    return -1;
  }

  std::cout << "load success, the num of points is " << cloud->points.size() << std::endl;

  int num_points = points != 1000 ? points : 1000;
  int num_points_based_ratio = int(cloud->points.size() * ratio);
  if (is_based_points && is_based_ratio)
  {
    PCL_ERROR("args --points and --ratio cannot exist at the same time ");
    return -1;
  }
  if (is_based_points)
  {
    farthestPointSampling(cloud, sampledCloud, num_points);
  }
  if (is_based_ratio)
  {
    farthestPointSampling(cloud, sampledCloud, num_points_based_ratio);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr myfps_min_sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr myfps_max_sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (call_myimpl)
  {
    std::cout << "[myfps] no visualize, plese check output file" << std::endl;
    my_fps(cloud, myfps_min_sampledCloud, is_based_ratio ? num_points_based_ratio : num_points);
    my_fps_max(cloud, myfps_max_sampledCloud, is_based_ratio ? num_points_based_ratio : num_points);

    pcl::io::savePCDFile("myfps(min)-output.pcd", *myfps_min_sampledCloud);
    pcl::io::savePCDFile("myfps(max)-output.pcd", *myfps_max_sampledCloud);

    std::cout << "myfps min sample cloud has " << myfps_min_sampledCloud->points.size() << " points." << std::endl;
    std::cout << "myfps max sample cloud has " << myfps_max_sampledCloud->points.size() << " points." << std::endl;
  }
  if (is_saved)
  {
    pcl::io::savePCDFile("FPS-outpu.pcd", *sampledCloud);
    std::cout << "save success" << std::endl;
  }

  std::cout << "sample cloud has " << sampledCloud->points.size() << " points." << std::endl;
  visualizeCloud(cloud, sampledCloud);
  return 0;
}