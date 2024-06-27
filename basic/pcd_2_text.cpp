#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <pcl/console/parse.h>
#include <iostream>

void pcd2text(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  FILE *wc = fopen("pcd2txt.txt", "w");
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
  std::ifstream points_in(filename);
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

void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " cloud/txt [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-m           1: pcd2txt 2: txt2pcd\n";
}

int main(int argc, char **argv)
{
  std::cout << argc << std::endl;

  if (pcl::console::find_argument(argc, argv, "-h") >= 0 || argc != 4)
  {
    printUsage(argv[0]);
    return 0;
  }
  int mode = -1;
  if (pcl::console::find_argument(argc, argv, "-m") >= 0)
  {
    pcl::console::parse<int>(argc, argv, "-m", mode);
  }
  else
  {
    printUsage(argv[0]);
    return 0;
  }

  std::cout << "txt/pcd: "
            << " " << argv[1] << " \n";
  std::cout << "-m     :"
            << " " << mode << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  char *filename;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  switch (mode)
  {
  case 1:
    if (pcl::io::loadPCDFile(argv[1], *basic_cloud_ptr) < 0)
    {
      PCL_ERROR("load pcd file failed.\n");
      return -1;
    }
    pcd2text(basic_cloud_ptr);
    break;
  case 2:
    filename = argv[1];
    ret_ptr = txt2pcd(filename);
    std::cout<<"point cloud size:"<<ret_ptr->size()<<std::endl;
    break;
  default:
    break;
  }
  return 0;
}