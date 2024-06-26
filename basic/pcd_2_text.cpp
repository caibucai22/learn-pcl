#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void pcd2text(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, char *filename)
{
  FILE *wc = fopen(filename, "w");
  int sizepcd = cloud->points.size();
  for (int i = 0; i < sizepcd; i++)
  {
    fprintf(wc, "%f\t%f\t%f\n", cloud->points[i].x,
            cloud->points[i].y,
            cloud->points[i].z);
  }
  fclose(wc);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr txt2pcd(char *filename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  ifstream points_in(filename);
  pcl::PointXYZ tmpoint;
  if (points_in.is_open())
  {
    while (!points_in.eof())
    {
      points_in >> tmpoint.x >> tmpoint.y >> tmpoint.z;
      basic_cloud_ptr->points.push_back(tmpoint);
    }
  }
  basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
  basic_cloud_ptr->height = 1;
  return basic_cloud_ptr;
}