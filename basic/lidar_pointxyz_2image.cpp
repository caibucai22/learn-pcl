/**
 * @file lidar_pointxyz_2image.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <vector>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  if (pcl::io::loadPCDFile(argv[1], pointcloud) < 0)
  {
    PCL_ERROR("load pcd file failed.\n");
    return -1;
  }
  // 去除nan点
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(pointcloud, pointcloud, mapping);
  // 获取点云最值
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(pointcloud, min, max);

  printf("min = %f %f %f \n", min.x, min.y, min.z);
  printf("max = %f %f %f \n", max.x, max.y, max.z);

  // 定义图像宽高
  int img_rows = max.x - min.x + 1;
  int img_cols = max.y - min.y + 1;

  cv::Mat image1(img_rows, img_cols, CV_8UC1);

  for (int i = 0; i < image1.rows; i++)
  {
    for (int j = 0; j < image1.cols; j++)
    {
      image1.at<uchar>(i, j) = 0;
    }
  }

  // 根据点云高度对图像进行赋值
  for (int i = 0; i < pointcloud.points.size(); i++)
  {
    int image_i, image_j;
    if (pointcloud.points[i].x < 0)
    {
      image_i = -min.x + pointcloud.points[i].x;
    }
    else
    {
      image_i = -min.x + pointcloud.points[i].x;
    }

    if (pointcloud.points[i].y < 0)
    {
      image_j = -min.y + pointcloud.points[i].y;
    }
    else
    {
      image_j = -min.y + pointcloud.points[i].y;
    }
    if ((pointcloud.points[i].z > 0) || (pointcloud.points[i].z < 0) || (pointcloud.points[i].z == 0))
    {
      if (pointcloud.points[i].z - min.z < 255)
      {
        image1.at<uchar>(image_i, image_j) = uchar((pointcloud.points[i].z - min.z) / (max.z - min.z) * 255);
      }
    }
    else
    {
      image1.at<uchar>(image_i, image_j) = 0;
    }
  }
  // print image
  std::cout << image1 << std::endl;
  imwrite("0_255_level.png", image1);
  imshow("img", image1);
  waitKey(0);
  destroyAllWindows();

  return 0;
}
