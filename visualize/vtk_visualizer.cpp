#include "../my_functions.h"

int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (argc < 2 || pcl::io::loadPCDFile(argv[1], *cloud_ptr))
  {
    PCL_ERROR("load pcd failed");
    return -1;
  }
  visualizePointCloudByVTK(cloud_ptr);

  return 0;
}