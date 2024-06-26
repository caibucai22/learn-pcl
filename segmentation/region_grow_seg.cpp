#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <Windows.h>
#include <stdio.h>
#include <Psapi.h>

void printMemory()
{
  HANDLE hProcess;
  PROCESS_MEMORY_COUNTERS pmc;

  hProcess = GetCurrentProcess();
  printf("\nProces ID: %u\n", hProcess);

  if (NULL == hProcess)
    return;

  if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc)))
  {
    printf("\tPageFaultCount: 0x%08X\n", pmc.PageFaultCount);
    printf("\tPeakWorkingSetSize: 0x%08X\n", pmc.PeakWorkingSetSize);
    printf("\tWorkingSetSize: 0x%08X\n", pmc.WorkingSetSize);
    printf("\tQuotaPeakPagedPoolUsage: 0x%08X\n", pmc.QuotaPeakNonPagedPoolUsage);
    printf("\tQuotaPagedPoolUsage: 0x%08X\n", pmc.QuotaPagedPoolUsage);
    printf("\tQuotaNonPagedPoolUsage", pmc.QuotaNonPagedPoolUsage);
    printf("\tPagefileUsage:0x%08X\n", pmc.PagefileUsage);
    printf("\tPeakPagefileUsage: 0x%08X\n", pmc.PeakPagefileUsage);
  }

  CloseHandle(hProcess);
}

using namespace pcl::console;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << ".exe xx.pcd -kn 50 -bc 0 -fc 10.0 -nc 0 -st 30 -ct 0.05" << std::endl;
    return 0;
  }

  time_t start, end, diff[5], option;
  start = time(0);
  int K = 50;
  bool bool_cuting = false;
  // 平滑阈值 曲率阈值
  float far_cuting = 10, near_cuting = 0, smoothnessThreshold = 30.0, curvatureThreshold = 0.05;
  parse_argument(argc, argv, "-kn", K);
  parse_argument(argc, argv, "-bc", bool_cuting);
  parse_argument(argc, argv, "-fc", far_cuting);
  parse_argument(argc, argv, "-nc", near_cuting);
  parse_argument(argc, argv, "-st", smoothnessThreshold);
  parse_argument(argc, argv, "-ct", curvatureThreshold);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) < 0)
  {
    std::cout << "load pcd file failed." << std::endl;
    return -1;
  }

  end = time(0);
  diff[0] = difftime(end, start);
  PCL_INFO("Loading pcd file takes %d s\n", diff[0]);

  // 法线和表面曲率估计
  pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(K);
  ne.compute(*normals);
  end = time(0);

  diff[1] = difftime(end, start) - diff[0];
  PCL_INFO("Estimating normal takes %d s \n", diff[1]);

  // 直通滤波 可选
  pcl::IndicesPtr indices(new std::vector<int>);
  if (bool_cuting)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(near_cuting, far_cuting);
    pass.filter(*indices);
  }

  // 区域生长
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  if (bool_cuting)
    reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smoothnessThreshold / 180.0 * M_PI);
  reg.setCurvatureThreshold(curvatureThreshold);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  end = time(0);
  diff[2] = difftime(end, start) - diff[0] - diff[1];
  PCL_INFO("region growing takes %d s\n", diff[2]);

  std::cout << "the num of clusters: " << clusters.size() << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;

  // 保存聚类的点云
  // int j = 0;
  // for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
  // {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

  //   for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
  //   {
  //     cloud_cluster->points.push_back(cloud->points[*pit]);
  //   }
  //   cloud_cluster->width = cloud_cluster->points.size();
  //   cloud_cluster->height = 1;
  //   cloud_cluster->is_dense = true;

  //   std::cout << "PointCloud representing the cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
  //   std::stringstream ss;
  //   ss << "cloud_cluster_" << j << ".pcd";
  //   pcl::io::savePCDFile(ss.str(), *cloud_cluster);
  //   j++;
  // }

  printMemory();
  // 可视化
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  pcl::visualization::CloudViewer viewer("region growing seg");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped())
  {
    
  }
  return 0;
}