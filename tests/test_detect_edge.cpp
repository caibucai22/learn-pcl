#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <queue>
#include <iostream>

void SegByLocalDensity（const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &clusters,
    std::vector<std::vector<int>> &gappointindices）
{
  // 首先初始化搜索领域半径radius和最小点minPts
  float radius = 0.8;
  int minPts = 90;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr treedense(new pcl::search::KdTree<pcl::PointXYZ>);
  treedense->setInputCloud(cloud);
  // 初始化向量，储存局部密度
  std::vector<int> nn_indices;                     // 储存点云中每个点的最邻索引
  std::vector<float> nn_dists;                     // 储存每个点的最邻距离
  std::vector<float> densiities(cloud->size(), 0); // 储存每个点的局部密度，并初始化为零

  for (size_t i = 0; i < cloud->size(); i++)
  {
    treedense->radiusSearch(i, radius, nn_indices, nn_dists); // 1.要查找最邻的点的索引，2.指定半径，3.储存最邻点索引的向量
    densiities[i] = nn_indices.size();                        // 赋值给densities
  }

  // 基于广度优先搜索
  std::vector<bool> visited(cloud->size(), false); // 用于记录每个点是否被访问过，初始化为每个点都没有访问过
  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (visited[i] || densiities[i] < minPts) //||或逻辑，只有当该点已经被访问，且点的密度大于minpts，才执行下面的处理
    {
      continue;
    }

    // visited[i]为true且densities[i]>=minPts
    pcl::PointCloud<pcl::PointXYZ> cluster; // 用于储存同一族的点，点云对象
    std::vector<int> singlegapindices;
    std::queue<int> q; // queue先进先出的数据结构
    // 使用一个队列q保存当前聚类的点，初始时加入起始点i
    q.push(i);         // 对queue的对象在push的时候会调用拷贝对象的拷贝构造函数，插入queue中的是对象的一份拷贝
    visited[i] = true; // 标记起始点已经被访问

    // 基于广度优先搜索算法对每个聚类进行拓展，BFS从起始点开始逐层搜索
    while (!q.empty()) // 如果q中没有元素则返回true
    {
      // 循环内部，每次取出队列中一个idx，并将其加入到当前聚类中
      // 然后查找其半径范围内的邻居点，并将未访问过且密度大于等于minpt的点放入队列中
      int idx = q.front(); // front返回queue中的第一元素的引用
      q.pop();             // 删除q中第一个元素-----------------用于退出循环
      cluster.push_back((*cloud)[idx]);
      singlegapindices.push_back(idx);
      std::vector<int> n_indices;
      std::vector<float> n_dists;
      treedense->radiusSearch(idx, radius, n_indices, n_dists);
      for (size_t j = 0; j < n_indices.size(); j++)
      {
        int neighbor_idx = n_indices[j];
        if (!visited[neighbor_idx] && densiities[neighbor_idx] >= minPts) // 没有被访问过且点的密度大于minpt
        {
          q.push(neighbor_idx);         // 加入到队列中
          visited[neighbor_idx] = true; // 标记已访问
        }
      }
    }

    clusters.push_back(cluster);
    gappointindices.emplace_back(singlegapindices);
  }

  int main()
  {
    std::cout << "hello world" << std::endl;
    return 0;
  }