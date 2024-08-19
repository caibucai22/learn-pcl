#include <pcl/search/kdtree.h>
#include <pcl/console/parse.h>

#include <vector>
#include <queue>

#include "../my_functions.h"

void compute(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
             float radius, std::vector<float> &out_densities)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  std::vector<int> indices;
  std::vector<float> distances;
  out_densities.assign(cloud_in->points.size(), 0);

  for (size_t i = 0; i < cloud_in->size(); i++)
  {
    tree->radiusSearch(i, radius, indices, distances);
    out_densities[i] = indices.size();
  }
}

void density_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                        std::vector<float> &densities,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
                        std::vector<pcl::PointIndices::Ptr> &cluster_indices,

                        float radius, int minPts)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  std::vector<bool> visited(cloud_in->size(), false);
  for (size_t i = 0; i < cloud_in->size(); i++)
  {
    if (visited[i] || densities[i] < minPts)
    {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointIndices::Ptr cluster_indice(new pcl::PointIndices);
    std::queue<int> q;

    q.push(i);
    pcl::PointXYZ tmp_Point;
    while (!q.empty())
    {
      int idx = q.front();
      q.pop();
      // cluster->points.push_back((*cloud_in).points[idx]);
      // cluster->points.push_back({ cloud_in->points[idx].x,cloud_in->points[idx].y,cloud_in->points[idx].z });
      // tmp_Point = cloud_in->points[idx];
      // cluster->points.push_back(tmp_Point);
      cluster->points.push_back(cloud_in->points[idx]);
      cluster_indice->indices.push_back(idx);

      std::vector<int> indices;
      std::vector<float> distances;
      tree->radiusSearch(idx, radius, indices, distances);

      for (size_t j = 0; j < indices.size(); j++)
      {
        int nn_idx = indices[j];
        if (!visited[nn_idx] && densities[nn_idx] >= minPts)
        {
          q.push(nn_idx);
          visited[nn_idx] = true;
        }
      }
    }

    clusters.push_back(cluster);
    cluster_indices.push_back(cluster_indice);
  }
}

void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " cloud_file_path [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h                 this help\n"
            << "--radius           for computing number of neighbors, default 0.1\n"
            << "--points           minimal points for density clustering, default 3\n";
}

void prase_args(int argc, char *argv[], float &radius, int &minPts)
{
  if (pcl::console::find_argument(argc, argv, "-h") >= 0 || argc < 2)
  {
    printUsage(argv[0]);
    return;
  }
  if (pcl::console::find_argument(argc, argv, "--radius") >= 0)
  {
    pcl::console::parse<float>(argc, argv, "--radius", radius);
  }
  if (pcl::console::find_argument(argc, argv, "--points") >= 0)
  {
    pcl::console::parse<float>(argc, argv, "--points", radius);
  }
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  loadPCD_(argv[1], cloud);
  printPointNum(cloud->points.size(), true);
  float radius = 0.1;
  int minPts = 2;
  prase_args(argc, argv, radius, minPts);

  std::vector<float> densities;
  std::cout << "start computing density" << std::endl;
  compute(cloud, radius, densities);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  std::vector<pcl::PointIndices::Ptr> cluster_indices;
  density_clustering(cloud, densities,
                     clusters, cluster_indices,
                     radius, minPts);

  //------
  if (clusters.size() > 1)
  {
    std::cout << "clusters : " << clusters.size() << std::endl;
    visualizeClustering(clusters);
  }
  else
  {
    std::cout << "clustering failed" << std::endl;
    visualizePointCloudByVTK(cloud);
  }

  // pcl::visualization::PCLVisualizer viewer("Point Cloud Segmentation");
  // viewer.setBackgroundColor(0, 0, 0);
  // int viewport = 0;
  // std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> color_handlers;
  // std::vector<std::string> cloud_ids;
  // for (size_t i = 0; i < clusters.size(); ++i)
  // {
  //   std::stringstream ss;
  //   ss << "cluster_" << i;
  //   cloud_ids.push_back(ss.str());
  //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(clusters[i], rand() % 255, rand() % 255, rand() % 255);
  //   color_handlers.push_back(color_handler);
  //   viewer.addPointCloud<pcl::PointXYZ>((clusters[i]),
  //                                       color_handlers.back(), cloud_ids.back(), viewport);
  // }

  // while (!viewer.wasStopped())
  // {
  //   viewer.spinOnce();
  // }
  return 0;
}