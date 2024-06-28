#include "kmeans.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <string>
using namespace std;

void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " cloud_file_path [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h                 this help\n"
            << "-iter              num of iterations,default 100\n"
            << "-cluster           num of clusters, default 100\n"
            << "-save              whether save each cluster\n";
}

void prase_args(int argc, char *argv[], int &iter, int &cluster)
{
  if (pcl::console::find_argument(argc, argv, "-h") >= 0 || argc < 2)
  {
    printUsage(argv[0]);
    return;
  }
  int mode = -1;
  if (pcl::console::find_argument(argc, argv, "-iter") >= 0)
  {
    pcl::console::parse<int>(argc, argv, "-iter", iter);
  }

  if (pcl::console::find_argument(argc, argv, "-cluster") >= 0)
  {
    pcl::console::parse<int>(argc, argv, "-cluster", cluster);
  }
}

int main(int argc, char *argv[])
{
  // 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (argc < 2 || pcl::io::loadPCDFile(argv[1], *cloud_ptr) == -1)
  {
    PCL_ERROR("loaded pcd file failed.");
    return -1;
  }
  pcl::console::print_highlight(("Loaded " + to_string(cloud_ptr->size()) + " points from pcd file.").c_str());
  cout << "\n"
       << endl;

  // K 均值聚类
  pcl::StopWatch time;
  int clusterNum = 100;
  int maxIter = 100;
  prase_args(argc, argv, maxIter, clusterNum);
  pcl::console::print_highlight("args: iters %d clusters %d \n", maxIter, clusterNum);
  KMeans kmeans(clusterNum, maxIter);
  std::vector<pcl::Indices> cluster_indices;
  pcl::console::print_highlight("Starting kmeans...\n");
  kmeans.extract(cloud_ptr, cluster_indices);

  pcl::console::print_highlight(("result \n clusters: " + to_string(cluster_indices.size()) + " \n").c_str());

  stringstream ss;
  ss << "elapsed time " << to_string(time.getTimeSeconds()) << " second\n";
  pcl::console::print_highlight(ss.str().c_str());


  bool save = false;
  if (pcl::console::find_argument(argc, argv, "-save") >= 0)
  {
    save = true;
  }

  if (save)
  {
    // 聚类结果保存
    pcl::console::print_highlight("Saving result...\n");
    int begin = 1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dbscan_all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
      // 获取聚类点云团的点
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dbscan(new pcl::PointCloud<pcl::PointXYZRGB>);
      // 同一团点云赋上同一种颜色
      uint8_t r = rand() % (256) + 0;
      uint8_t g = rand() % (256) + 0;
      uint8_t b = rand() % (256) + 0;

      for (auto pit = it->begin(); pit != it->end(); pit++)
      {
        pcl::PointXYZRGB point_db;
        point_db.x = cloud_ptr->points[*pit].x;
        point_db.y = cloud_ptr->points[*pit].y;
        point_db.z = cloud_ptr->points[*pit].z;

        point_db.r = r;
        point_db.g = g;
        point_db.b = b;
        cloud_dbscan->points.push_back(point_db);
      }
      // 分类保存
      if (!cloud_dbscan->empty())
        pcl::io::savePCDFileBinary("kmeans_" + std::to_string(begin) + ".pcd", *cloud_dbscan);
      begin++;
    }
  }
  return 0;
}