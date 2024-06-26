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

#include <opencv2/opencv.hpp>

#include <vtkPlaneSource.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGB pointcolor;

int scale_to_255(int pixel, int min, int max)
{
  return int((((pixel - min) / float(max - min)) * 255));
}
/**
 * @brief
 *
 * @param cloud
 * @param res
 * @param side_range_left
 * @param side_range_right
 * @param fwd_range_behind
 * @param fwd_range_front
 * @param height_range_min
 * @param height_range_max
 */
void use_pcd_to_img(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    float res,
                    float side_range_left,
                    float side_range_right,
                    float fwd_range_behind,
                    float fwd_range_front,
                    float height_range_min,
                    float height_range_max)
{
  res = 1.0;
  pcl::PointXYZRGB min, max;
  pcl::getMinMax3D(*cloud, min, max);

  // 将点云的最值设置为框住点云的边界,自动设置边框大小,目的就是构建图片大小。
  // 在cloudcompare中绿色坐标轴代表y，红色坐标轴代表x，蓝色坐标轴代表z，因此我们想要的是水平面也就是XOZ面
  // 根据双目成像原理我认为点云的原点就是相机的位置。，所以我认为款选的是要固定的
  float side_range[] = {-1500, 800};
  float fwd_range[] = {-500, 500};
  float height_range[] = {-2, 2};

  //
  vector<int> indices;

  for (int i = 0; i < cloud->points.size(); i++)
  {
    if ((cloud->points[i].x > fwd_range[0] && cloud->points[i].x < fwd_range[1]) &&
        (cloud->points[i].y > height_range[0] && cloud->points[i].y < height_range[1]) &&
        (cloud->points[i].z > side_range[0] && cloud->points[i].z < side_range[1]))
    {
      indices.push_back(i);
    }
  }

  vector<int> x_img;
  vector<int> y_img;
  //
  for (int i = 0; i < indices.size(); i++)
  {
    x_img.push_back(-cloud->points[i].z / res);
    y_img.push_back(-cloud->points[i].x / res);
  }

  // 平移图像数据到点云左上角
  for (int i = 0; i < indices.size(); i++)
  {
    x_img[i] -= int(floor(side_range[0] / res));
    y_img[i] += int(ceil(fwd_range[1] / res));
  }

  // 厚度信息转为 255 并填充到对于img 的对应坐标中
  vector<int> pixel_values;
  for (int i = 0; i < indices.size(); i++)
  {
    if (cloud->points[i].y > height_range[0] && cloud->points[i].y < height_range[1])
      pixel_values.push_back(cloud->points[i].y);
  }

  for (int i = 0; i < indices.size(); i++)
  {
    pixel_values[i] = scale_to_255(pixel_values[i], height_range[0], height_range[1]);
  }

  // 创建图像大小
  int x_max = 1 + int((side_range[1] - side_range[0]) / res);
  int y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res);

  // opencv
  cv::Mat im = cv::Mat::zeros(x_max, y_max, CV_8U);
  for (int i = 0; i < x_img.size(); i++)
  {
    im.at<uchar>(x_img[i], y_img[i]) = pixel_values[i];
  }

  // 数据图片打印信息
  cout << "图像中x轴最小点坐标值为x_img min:" << *min_element(x_img.begin(), x_img.end()) << endl;
  cout << "图像中x轴最大点坐标值为x_img max:" << *max_element(x_img.begin(), x_img.end()) << endl;
  cout << "图像中y轴最小点坐标值为y_img min:" << *min_element(y_img.begin(), y_img.end()) << endl;
  cout << "图像中y轴最大点坐标值为y_img max:" << *max_element(y_img.begin(), y_img.end()) << endl;

  // 可视化
  cv::namedWindow("img_ret");
  cv::imshow("img", im);

  // 取反
  cv::Mat im_reverse;
  im_reverse = 255 - im;
  cv::namedWindow("img_reverse_ret");
  cv::imshow("img_reserve", im_reverse);
  cv::waitKey();

  // 图片保存
  std::vector<int> param;
  param.push_back(cv::IMWRITE_PXM_BINARY);
  cv::imwrite("img_reverse.bmp", im_reverse, param);
  cout << "save img done!" << endl;
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pointcolor>::Ptr cloud(new pcl::PointCloud<pointcolor>);
  pcl::io::loadPCDFile(argv[1], *cloud);

  float res = 0.5;
  float side_range_left = -100;
  float side_range_right = 100;

  float fwd_range_behind = -100;
  float fwd_range_front = 100;

  float height_range_min = -100;
  float height_range_max = 100;

  use_pcd_to_img(cloud,
                 res,
                 side_range_left,
                 side_range_right,
                 fwd_range_behind,
                 fwd_range_front,
                 height_range_min,
                 height_range_max);

  return 0;
}
