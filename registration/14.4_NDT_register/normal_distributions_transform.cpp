/**
 * @file normal_distributions_transform.cpp
 * @author your name (you@domain.com)
 * @brief 使用正态分布进行配准的实验
 * @version 0.1
 * @date 2024-03-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
// #include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR("couldn't read file room_scan2.pcd\n");
    return -1;
  }

  std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

  // 加载新视角得到的第二次扫描点云数据 作为 源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR("couldn't read file room_scan2.pcd\n");
    return (-1);
  }

  std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
  // 配准是 源点云 -> 目标点云 的变换矩阵估计
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size()
            << " data points from room_scan2.pcd" << std::endl;

  // 初始化正态分布 NDT 对象
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.01); // 设置终止条件 最小转换差异
  ndt.setStepSize(0.1);               // 为 more-thuente 线搜索设置最大步长
  ndt.setResolution(1.0);             // 设置NDT网格结构的分辨率

  ndt.setMaximumIterations(35);
  ndt.setInputSource(filtered_cloud);
  ndt.setInputTarget(target_cloud);

  // 设置 使用机器人测距法得到的粗略初始变换矩阵结果
  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  // 计算需要的刚体变换以便将输入的源点云 匹配到目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess);
  std::cout << "Normal distributions Transform has converged: " << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << std::endl;

  // 使用创建的变换对过滤的输入点云进行变换
  pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

  pcl::io::savePCDFileASCII("../../room_scan2_transfromed.pcd", *output_cloud);

  // 初始化点云可视化对象
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
      viewer_final(new pcl::visualization::PCLVisualizer("3D viewer"));
  viewer_final->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色

  // 对目标点云可视化 red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color(target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
  // 对转换后源点云 可视化 green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color(output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 1, "output cloud");

  //
  viewer_final->addCoordinateSystem(1.0); // display xyz axis
  viewer_final->initCameraParameters();   // init camera

  //
  while (!viewer_final->wasStopped())
  {
    viewer_final->spinOnce();
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  return 0;
}