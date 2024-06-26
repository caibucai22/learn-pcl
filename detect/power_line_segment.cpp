#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <chrono>
#include <thread>

using namespace std;

std::vector<int> powerline_seg(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud,
                               double max_dist = 0.15, double threshould = 0.81)
{
  // 建立kd-tree 索引
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(input_cloud);
  pcl::Indices r_indices;

  std::vector<float> neighbor_square_distance;
  std::vector<int> lind_idx;

  for (size_t i = 0; i < input_cloud->points.size(); i++)
  {
    if (i % 1000 == 0)
    {
      std::cout << "num: " << i << endl;
    }

    if (kdtree.radiusSearch(input_cloud->points[i], max_dist, r_indices, neighbor_square_distance) > 3)
    {
      cout << "r_indices.size(): " << r_indices.size() << endl;
      // PCA 计算每个点的特征值
      pcl::PCA<pcl::PointXYZ> pca;
      pca.setInputCloud(input_cloud);

      pca.setIndices(std::make_shared<const pcl::Indices>(r_indices));
      pca.getEigenValues(); // 获取特征值

      float l1 = pca.getEigenValues()[0];
      float l2 = pca.getEigenValues()[1];
      float l3 = pca.getEigenValues()[2];
      // 计算每个点的线性特征
      float linear = (l1 - l2) / l1;
      // 设置阈值提取电力线
      if (linear > threshould)
      {
        lind_idx.push_back(i);
      }
    }
    else
    {
      // 半价max_dist 领域范围内没有点，则跳过该点
      continue;
    }
  }
  return lind_idx;
}

void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("display pointcloud"));
  viewer->setWindowName("powerline seg");
  viewer->setBackgroundColor(0, 0, 0);

  viewer->addPointCloud(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  viewer->addPointCloud(filter_cloud, "filtered cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered cloud");

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
}

int main(int argc, char **argv)
{
  printf("加载点云\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("xx.pcd", *cloud) == -1)
  {
    PCL_ERROR("load pcd file failed.");
    return -1;
  }

  printf("高程滤波\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr high(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z"); // z
  pass.setFilterLimits(-100.0, -1.0);

  pass.setNegative(true);
  pass.filter(*high);

  pcl::PointCloud<pcl::PointXYZ>::Ptr low(new pcl::PointCloud<pcl::PointXYZ>);
  pass.setNegative(false);
  pass.filter(*low);
  printf("高程滤波结束!!!");

  // 分割电力线
  printf("分割电力线\n");
  cout << "high point numbers: " << high->points.size() << endl;
  cout << "low point numbers: " << low->points.size() << endl;

  std::vector<int> power_line_idx;
  power_line_idx = powerline_seg(high, 0.5);
  printf("电力线分割结束\n");

  // 提取电力线
  pcl::ExtractIndices<pcl::PointXYZ> extr;
  extr.setInputCloud(high);

  extr.setIndices(std::make_shared<const std::vector<int>>(power_line_idx));
  pcl::PointCloud<pcl::PointXYZ>::Ptr power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extr.filter(*power_line_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  extr.setNegative(true);
  extr.filter(*output);

  // 保存电力线
  pcl::io::savePCDFileASCII("line_cloud.pcd", *power_line_cloud);
  // 其他部分
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *out_power_line_cloud = *low + *output; // 非电力线 + 过滤后的非电力线output
  pcl::io::savePCDFileASCII("out_line_cloud.pcd", *out_power_line_cloud);

  // 可视化结果
  visualize_cloud(power_line_cloud, out_power_line_cloud);

  return 0;
}