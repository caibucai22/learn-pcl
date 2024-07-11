/**
 * @file split_by_rho_theta_z.cpp
 * @author csy (caibucai22@gmail.com)
 * @brief 将点云转到极坐标系下 基于 theta 和 z 等划分
 * @version 0.1
 * @date 2024-07-10
 * @todo 更换命令行解析 实现z 自定义范围的 划分; 评估中心vs 其他心
 * @copyright Copyright (c) 2024
 *
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <algorithm>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <time.h>

#include "../my_functions.h"

// 计算点云在Z平面上的中心点
pcl::PointXYZ calculateCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::PointXYZ center(0.0, 0.0, 0.0);
  for (const auto &point : cloud->points)
  {
    center.x += point.x;
    center.y += point.y;
  }
  center.x /= cloud->points.size();
  center.y /= cloud->points.size();
  return center;
}

// 计算点相对于中心点的圆柱坐标 (ρ, θ)
void cartesianToCylindrical(const pcl::PointXYZ &point, const pcl::PointXYZ &center, float &rho, float &theta)
{
  float dx = point.x - center.x;
  float dy = point.y - center.y;
  rho = std::sqrt(dx * dx + dy * dy);
  theta = std::atan2(dy, dx);
  if (theta < 0)
  {
    theta += 2 * M_PI; // 转换到 [0, 2*PI] 范围内
  }
}

// 计算点的圆柱坐标
void cartesianToCylindrical(const pcl::PointXYZ &point, float &rho, float &theta, float &z)
{
  rho = std::sqrt(point.x * point.x + point.y * point.y);
  theta = std::atan2(point.y, point.x);
  z = point.z;
}

void splitPointCloudByTheta(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float angle_deg, std::vector<pcl::PointIndices::Ptr> &bins)
{
  // 计算点云在Z平面上的中心点
  pcl::PointXYZ center = calculateCenter(cloud);

  // 将角度转换为弧度
  float angle_rad = pcl::deg2rad(angle_deg);

  // 计算分割数量
  int num_bins = static_cast<int>(360.0 / angle_deg);

  // 初始化bins
  bins.resize(num_bins);
  for (int i = 0; i < num_bins; ++i)
  {
    bins[i].reset(new pcl::PointIndices);
  }

  // 对点云进行分割
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    float rho, theta;
    cartesianToCylindrical(cloud->points[i], center, rho, theta);

    // 计算分割索引
    int bin_index = static_cast<int>(theta / angle_rad);

    // 添加点索引到相应的分割
    bins[bin_index]->indices.push_back(i);
  }
}

void printPoint2Bin(std::vector<pcl::PointIndices::Ptr> &bins)
{
  // 输出每个bin中的点索引
  for (size_t i = 0; i < bins.size(); ++i)
  {
    std::cout << "Bin " << i << ": ";
    for (size_t j = 0; j < bins[i]->indices.size(); ++j)
    {
      std::cout << bins[i]->indices[j] << " ---> " << i << "\n";
    }
    std::cout << " " << std::endl;
  }
}

void extractPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointIndices::Ptr> &cloud_segments_indices,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &out_cloud_segments, bool save = false,
                       float angle = 30.0)
{
  pcl::PCDWriter writer;
  std::stringstream ss;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud(cloud);
  extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
  out_cloud_segments.resize(cloud_segments_indices.size());
  for (int i = 0; i < cloud_segments_indices.size(); i++)
  {
    out_cloud_segments[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setIndices(cloud_segments_indices[i]);

    extract.filter(*out_cloud_segments[i]);

    if (save)
    {
      ss.str("");
      ss << "cloud_split_" << std::to_string(i * angle) << ".pcd";
      writer.write(ss.str(), *out_cloud_segments[i]);
    }
  }
}

void extractPointCloudByPassThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float z_min, float z_max,
                                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &out_cloud_segments,
                                    std::string field = "z", const std::vector<float> &z_split = {},
                                    bool save = false)
{
  if (z_split.size() >= 2 && is_sorted(z_split.begin(), z_split.end()) && *z_split.begin() == 0.0 && *z_split.end() == 1.0)
  {
    pcl::PCDWriter writer;
    std::stringstream ss;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);
    out_cloud_segments.resize(out_cloud_segments.size());
    for (int i = 1; i < z_split.size(); i++)
    {
      out_cloud_segments[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
      pass.setFilterLimits(z_split[i - 1], z_split[i]);
      pass.filter(*out_cloud_segments[i]);
      if (save)
      {
        ss.clear();
        ss << "cloud_split_z" << std::to_string(z_split[i - 1]) << "_" << std::to_string(z_split[i]);
        writer.write(ss.str(), *out_cloud_segments[i]);
      }
    }
  }
  else
  {
    PCL_ERROR("z_split should be sorted and not null,starting with 0.0 and ending with 1.0");
    return;
  }
}

void computeMinMaxZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float &min_z, float &max_z)
{
  min_z = std::numeric_limits<float>::max();
  max_z = std::numeric_limits<float>::lowest();
  for (const auto &point : cloud->points)
  {
    if (point.z < min_z)
      min_z = point.z;
    if (point.z > max_z)
      max_z = point.z;
  }
}
void extractPointCloudByPassThrough(float angle, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &out_cloud_segments,
                                    float z_ratio = 1.0,
                                    float limit_min = 0.0, float limit_max = 1.0, std::string field = "z", bool save = false)
{
  if (z_ratio != 1.0)
  {
    pcl::PCDWriter writer;
    std::stringstream ss;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);

    // printf("[%f,%f] \n", limit_min, limit_max);
    float range = limit_max - limit_min;
    int n = ceil(1.0 / z_ratio);
    out_cloud_segments.resize(n);
    float start = 0.0;
    for (int i = 0; i < n; i++)
    {
      out_cloud_segments[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
      if (start + z_ratio > 1.0)
      {
        pass.setFilterLimits(limit_min + start * range, limit_max);
      }
      else
      {
        pass.setFilterLimits(limit_min + start * range, limit_min + (start + z_ratio) * range);
      }

      pass.filter(*out_cloud_segments[i]);
      if (save)
      {
        ss.str("");
        ss << "cloud_split_" << std::to_string(angle) << "_z"
           << "_" << ((start + z_ratio <= 1.0) ? std::to_string(start + z_ratio) : "1") << ".pcd";
        writer.write(ss.str(), *out_cloud_segments[i]);
      }
      start += z_ratio;
    }
  }
  else
  {
    return;
  }
}

void printUsage(const char *progname)
{
  std::cout << "\n\nUsage: " << progname << " pcd --big big_cloud --small small_cloud\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "--angle           basic angle for spliting, default 30 \n"
            << "--z_res           z axis for spliting, default 1, 0.5 mean [0,0.5,1]\n"
            << "--save            save split result\n";
}

int main(int argc, char **argv)
{
  if (argc < 2 || pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return -1;
  }

  printArgs(argc, argv);

  float angle = 30.0f; // 划分角度
  float z_res = 1.0;
  bool is_saved = false;
  if (pcl::console::find_argument(argc, argv, "--angle") > 0)
  {
    pcl::console::parse<float>(argc, argv, "--angle", angle);
  }
  if (pcl::console::find_argument(argc, argv, "--z_res") > 0)
  {
    pcl::console::parse<float>(argc, argv, "--z_res", z_res);
  }
  if (pcl::console::find_argument(argc, argv, "--save") > 0)
  {
    is_saved = true;
  }

  // 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file \n");
    return -1;
  }
  std::cout << "load success, the num of points is " << cloud->points.size() << std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments;
  std::vector<pcl::PointIndices::Ptr> cloud_segments_indices;

  std::cout
      << "start building indices ..." << std::endl;
  auto t1 = clock();
  splitPointCloudByTheta(cloud, angle, cloud_segments_indices);
  std::cout << "building indices over         "
            << "elapsed time " << double(clock() - t1) / CLOCKS_PER_SEC << " sec" << std::endl;

  std::cout
      << "start spliting based theta ..." << std::endl;
  auto t2 = clock();
  // theta extract
  extractPointCloud(cloud, cloud_segments_indices, cloud_segments, is_saved);
  std::cout << "spliting based theta over     "
            << "elapsed time " << double(clock() - t2) / CLOCKS_PER_SEC << " sec" << std::endl;

  // compute z limit
  float z_min, z_max;
  computeMinMaxZ(cloud, z_min, z_max);

  // z extract
  bool enable_z_extract = false;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> z_cloud_segments;
  if (z_res != 1.0)
  {
    std::cout
        << "start spliting based z axis ..." << std::endl;
    auto t3 = clock();
    enable_z_extract = true;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> z_single_pcd_split;
    for (int i = 0; i < cloud_segments.size(); i++)
    {
      extractPointCloudByPassThrough(angle * i, cloud_segments[i], z_single_pcd_split, z_res, z_min, z_max, "z", is_saved);
      z_cloud_segments.insert(z_cloud_segments.end(), z_single_pcd_split.begin(), z_single_pcd_split.end());
    }
    std::cout << "spliting based z axis over    "
              << "elapsed time " << double(clock() - t3) / CLOCKS_PER_SEC << " sec" << std::endl;
  }

  // 可视化
  std::cout
      << "start visualizing ..." << std::endl;
  pcl::visualization::PCLVisualizer viewer("Point Cloud Segmentation");
  viewer.setBackgroundColor(0, 0, 0);
  int viewport = 0;
  std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> color_handlers;
  std::vector<std::string> cloud_ids;
  for (size_t i = 0; i < (enable_z_extract ? z_cloud_segments.size() : cloud_segments.size()); ++i)
  {
    std::stringstream ss;
    ss << "segment_" << i;
    cloud_ids.push_back(ss.str());
    // std::cout << cloud_ids.back() << "--->" << (enable_z_extract ? z_cloud_segments[i] : cloud_segments[i])->points.size() << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler((enable_z_extract ? z_cloud_segments[i] : cloud_segments[i]),
                                                                                  rand() % 255, rand() % 255, rand() % 255);
    color_handlers.push_back(color_handler);
    viewer.addPointCloud<pcl::PointXYZ>((enable_z_extract ? z_cloud_segments[i] : cloud_segments[i]),
                                        color_handlers.back(), cloud_ids.back(), viewport);
  }

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  return 0;
}
