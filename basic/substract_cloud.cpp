/**
 * @file substract_cloud.cpp
 * @author Csy (caibucai22@gmail.com)
 * @brief 用于增量扫描点云 获取新增点云
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024 & ref https://blog.csdn.net/Dbojuedzw/article/details/129884789
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree.h>
#include <pcl/console/parse.h>
#include <omp.h>

/**
 * @brief cloud2 = cloud0 - cloud1
 *
 * @param cloud1 减数点云
 * @param cloud0 被减数点云
 * @param cloud2 差点云
 */
void CloudSubtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud0, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2)
{
  std::cout << "start substracting ..." << std::endl;
  float resolution = 0.25f; // 八叉树分辨率，根据点云需要自行调整
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

  // 添加cloud1到八叉树中
  octree.setInputCloud(cloud1->makeShared());
  octree.addPointsFromInputCloud();

  octree.switchBuffers();

  // 添加cloud0到八叉树中
  octree.setInputCloud(cloud0->makeShared());
  octree.addPointsFromInputCloud();
  std::cout << "octree build success" << std::endl;

  std::vector<int> newPointIdxVector; // 存储新加入点索引的向量
  octree.getPointIndicesFromNewVoxels(newPointIdxVector);
  std::cout << "get increment idx success" << std::endl;

  cloud2->width = newPointIdxVector.size();
  cloud2->height = 1;
  cloud2->is_dense = false;
  cloud2->points.resize(cloud2->width * cloud2->height);
  std::cout << "increment pcd, the num of points is " << cloud2->width << std::endl;

#pragma omp parallel for
  for (int i = 0; i < newPointIdxVector.size(); i++)
  {
    cloud2->points[i].x = cloud0->points[newPointIdxVector[i]].x;
    cloud2->points[i].y = cloud0->points[newPointIdxVector[i]].y;
    cloud2->points[i].z = cloud0->points[newPointIdxVector[i]].z;
  }

  std::cout << "substract success!" << std::endl;
}

void printUsage(const char *progname)
{
  std::cout << "\n\nUsage: " << progname << " --big big_cloud --small small_cloud\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "--big             big cloud\n"
            << "--small           small cloud\n";
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr big(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr small(new pcl::PointCloud<pcl::PointXYZ>);
  if (argc != 5 || pcl::console::find_argument(argc, argv, "--big") < 0 || pcl::console::find_argument(argc, argv, "--small") < 0)
  {
    printUsage(argv[0]);
    return -1;
  }
  if (pcl::io::loadPCDFile(argv[2], *big) == -1)
  {
    PCL_ERROR("load big pcd failed");
    return -1;
  }
  std::cout << "load big success, the num of points is " << big->points.size() << std::endl;

  if (pcl::io::loadPCDFile(argv[4], *small) == -1)
  {
    PCL_ERROR("load small pcd failed");
    return -1;
  }
  std::cout << "load small success, the num of points is " << small->points.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr ret(new pcl::PointCloud<pcl::PointXYZ>);
  CloudSubtraction(big, small, ret);

  pcl::io::savePCDFile("increment.pcd", *ret);

  return 0;
}