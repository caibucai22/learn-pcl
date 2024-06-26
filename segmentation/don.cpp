/**
 * @file don.cpp
 * @author your name (you@domain.com)
 * @brief 法线微分
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/features/don.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;

/**
 * @brief Get the Colored Cloud object
 * 
 * @param input_ 
 * @param clusters_ 
 * @param r 
 * @param g 
 * @param b 
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_,
                                                       vector<pcl::PointIndices> clusters_, float r, float g, float b)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  if (!clusters_.empty())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
    srand(static_cast<unsigned int>(time(0)));
    vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size(); i_segment++)
    {
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;

    for (size_t i_point = 0; i_point < input_->points.size(); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = r;
      point.g = g;
      point.b = b;
      colored_cloud->points.push_back(point);
    }

    vector<pcl::PointIndices>::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin(); i_segment != clusters_.end(); i_segment++)
    {
      vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }
  return colored_cloud;
}

int main(int argc, char *argv[])
{
  int VISUAL = 1, SAVE = 0;
  double scale1 = 5, mean_radius;

  double scale2 = 10;

  double threshold = 0.3;
  double segradius = 10;

  pcl::PointCloud<PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
  pcl::io::loadPCDFile("region_growing_tutorial.pcd", *cloud);
  // 创建KD树
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized())
  {
    tree.reset(new pcl::search::OrganizedNeighbor<PointXYZRGB>());
  }
  else
  {
    tree.reset(new pcl::search::KdTree<PointXYZRGB>(false));
  }
  tree->setInputCloud(cloud);

  // 计算点云的平均半径 并根据相应的输入进行分割
  {
    int size_cloud = cloud->size();
    int step = size_cloud / 10;
    double total_distance = 0;
    int i, j = 1;
    for (i = 0; i < size_cloud; i += step, j++)
    {
      vector<int> pointIdxNkNSearch(2);
      vector<float> pointNKNSquaredDistance(2);
      tree->nearestKSearch(cloud->points[i], 2, pointIdxNkNSearch, pointNKNSquaredDistance);
      total_distance += pointNKNSquaredDistance[1] + pointNKNSquaredDistance[0];
    }
    mean_radius = sqrt((total_distance / j));
    cout << "mean radius of cloud is: " << mean_radius << endl;
    scale1 *= mean_radius;
    scale2 *= mean_radius;
    segradius *= mean_radius;
  }

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit(EXIT_FAILURE);
  }

  // 计算法线
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);

  // 设置视点
  ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

  // 小尺度辅助半径
  cout << "calculating normals for scale1..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<PointNormal>);
  ne.setNumberOfThreads(4);
  ne.setRadiusSearch(scale1);
  ne.compute(*normals_small_scale);

  // 大尺度辅助半径
  cout << "calculating normals for scale2..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<PointNormal>);
  ne.setNumberOfThreads(4);
  ne.setRadiusSearch(scale2);
  ne.compute(*normals_large_scale);

  // 可视化法线
  //-----------------------可视化法线--------------------------------
  if (VISUAL = 1)
  {
    cout << "click q key to quit the visualizer and continue！！" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> VN(new pcl::visualization::PCLVisualizer("Showing normals with different scale"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green(cloud, 0, 255, 0);
    int v1(0), v2(0);
    VN->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    VN->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    VN->setBackgroundColor(1, 1, 1);
    VN->addPointCloud(cloud, green, "small_scale", v1);
    VN->addPointCloud(cloud, green, "large_scale", v2);
    VN->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud, normals_small_scale, 100, mean_radius * 10, "small_scale_normal");
    VN->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud, normals_large_scale, 100, mean_radius * 10, "large_scale_normal");
    VN->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "small_scale", v1);
    VN->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "small_scale", v1);
    VN->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "large_scale", v1);
    VN->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "large_scale", v1);
    VN->spin();
  }

  // 为DON创造输出点云
  PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);
  cout << "calculating don..." << endl;

  // 创造don算子
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud(cloud);
  don.setNormalScaleLarge(normals_large_scale);
  don.setNormalScaleSmall(normals_small_scale);

  if (!don.initCompute())
  {
    cerr << "Error: cloud not initliaze DoN feature operator" << endl;
    exit(EXIT_FAILURE);
  }
  don.computeFeature(*doncloud);

  // 输出一些不同的曲率
  {
    cout << "" << endl;
  }

  // 展示大小尺度曲率差异
  if (VISUAL = 1)
  {
    cout << "click q key to quit the visualizer and continue！！" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> VC(new pcl::visualization::PCLVisualizer("Showing the difference of curvature of two scale"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k(doncloud, "curvature"); // PointCloudColorHandlerGenericField方法是将点云按深度值(“x”、“y”、"z"均可)的差异着以不同的颜色。
    VC->setBackgroundColor(1, 1, 1);
    VC->addPointCloud(doncloud, handler_k);
    VC->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    VC->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
    VC->spin();
  }
  // 保存 Don
  pcl::PCDWriter writer;
  if (SAVE == 1)
    writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);

  // 按照大小滤波
  cout << "filtering out don mag <= " << threshold << "..." << endl;
  pcl::ConditionOr<PointNormal>::Ptr range_cloud(new pcl::ConditionOr<PointNormal>());
  range_cloud->addComparison(pcl::FieldComparison<PointNormal>::ConstPtr(
      new pcl::FieldComparison<PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)));

  pcl::ConditionalRemoval<PointNormal> condrem;
  condrem.setInputCloud(doncloud);
  condrem.setCondition(range_cloud);
  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<PointNormal>);

  condrem.filter(*doncloud_filtered);
  doncloud = doncloud_filtered;
  cout << "filtered pointcloud has " << doncloud->points.size() << " points." << endl;
  if (SAVE == 1)
    writer.write("don_filtered.pcd", *doncloud, false);

  //--------展示保持相对较小曲率点的结果-------------
  if (VISUAL == 1)
  {
    cout << "click q key to quit the visualizer and continue！！" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Showing the results of keeping relative small curvature points"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k(doncloud, "curvature");
    MView->setBackgroundColor(1, 1, 1);
    MView->addPointCloud(doncloud, handler_k);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
    MView->spin();
  }

  cout << "clustering using euclideanclusterExtraction with tolerance <= " << segradius << endl;
  pcl::search::KdTree<PointNormal>::Ptr segtree(
      new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud(doncloud);

  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance(segradius);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(segtree);
  ec.setInputCloud(doncloud);
  ec.extract(cluster_indices);

  if (VISUAL == 1)
  { //----------------------可视化集群结果--------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*doncloud, *tmp_xyz);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = getColoredCloud(tmp_xyz, cluster_indices, 0, 255, 0);

    cout << "click q key to quit the visualizer and continue！！" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> Seg_View(new pcl::visualization::PCLVisualizer("visualize the clustering results"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbps(colored_cloud);
    Seg_View->setBackgroundColor(1, 1, 1);
    Seg_View->addPointCloud(colored_cloud, rgbps);
    Seg_View->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    Seg_View->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
    Seg_View->spin();
  }

  if (SAVE == 1)
  {
    // 保存聚类的点云
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
      pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointNormal>);

      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      {
        cloud_cluster_don->points.push_back(doncloud->points[*pit]);
      }
      cloud_cluster_don->width = cloud_cluster_don->points.size();
      cloud_cluster_don->height = 1;
      cloud_cluster_don->is_dense = true;

      std::cout << "PointCloud representing the cluster: " << cloud_cluster_don->points.size() << " data points." << std::endl;
      std::stringstream ss;
      ss << "don_cluster_" << j << ".pcd";
      writer.write(ss.str(),*cloud_cluster_don,false);
      j++;
    }
  }
}