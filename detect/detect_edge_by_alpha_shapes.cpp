#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>

#include "../my_functions.h"

using namespace std;

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  //-----------------加载点云----------------------
  pcl::PCDReader reader;
  if (argc < 2 || reader.read(argv[1], *cloud) < 0)
  {
    PCL_ERROR("load pcd failed");
    return -1;
  }

  pcl::console::TicToc time;
  time.tic();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud); // 输入点云为投影后的点云
  chull.setAlpha(0.35);       // 设置alpha值为0.1 平均点距的2到3倍
  chull.reconstruct(*cloud_hull);

  cout << "boundary points: " << cloud_hull->points.size() << endl;
  cout << "elapsed time:  " << time.toc() / 1000 << " seconds" << endl;
  pcl::PCDWriter writer;
  cloud_hull->width = cloud_hull->points.size();
  cloud_hull->height = 1;
  writer.write("hull_edge.pcd", *cloud_hull, false);
  //-----------------结果显示---------------------
  visualizeCloud(cloud, cloud_hull);

  return 0;
}
