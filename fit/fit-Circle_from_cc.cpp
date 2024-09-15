/**
 * 基于cc 的代码实现 圆检测 也就是 圆拟合
 * cccorelib geometricalAnalysisTools detectCircle
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3D.h> // 拟合3D圆
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include "../my_functions.h"
#include "Neighbourhood.h"
#include <thread>
#include <limits>

#include <stdio.h>
#include <cstdio>

static bool Landau_Smith(const std::vector<Eigen::Vector2d> &xy, Eigen::Vector2d &center, float &radius);
void DetectCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f &center, Eigen::Vector3f &normal, float& radius, double &rms);

void FitCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // 读入点云
  Eigen::Vector3f center(0,0,0);
  Eigen::Vector3f normal(0,0,0);
  float radius = -1.0;
  double rms = 0.0;
  DetectCircle(cloud, center, normal, radius, rms);
  printf("Center (%f,%f,%f) \n", center[0], center[1], center[2]);
  printf("Normal (%f,%f,%f) \n", normal[0], normal[1], normal[2]);
  printf("radius %f \n", radius);
  printf("rms %f \n", rms);
}

void DetectCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f &center, Eigen::Vector3f &normal, float& radius, double &rms)
{
  center = Eigen::Vector3f(0, 0, 0);
  normal = Eigen::Vector3f(0, 0, 1.0);

  radius = std::numeric_limits<float>::quiet_NaN();
  rms = std::numeric_limits<double>::quiet_NaN();

  if (!cloud)
  {
    assert(false);
    return;
  }

  unsigned n = cloud->size();
  if (n < 4)
  {
    // not enough points
    return;
  }

  // fit plane
  Neighbourhood YK(cloud);
  if (!YK.getLSPlane())
  {
    return;
  }

  // fit a circle with each eigenvector and keep best best result
  const Eigen::Vector3f *eigenvectors[2]{
      YK.getLSPlaneX(),
      YK.getLSPlaneNormal()};
  std::vector<Eigen::Vector2d> pointsOnPlane;
  try
  {
    pointsOnPlane.resize(cloud->size());
  }
  catch (const std::bad_alloc &e)
  {
    std::cerr << e.what() << '\n';
    // not enough memory
    return;
  }

  // compute cloud gravity center
  const Eigen::Vector3f *G = YK.getGravityCenter();
  assert(G);
#ifdef DEBUG_TRACE
  FILE *fp = fopen("./circle_fit.txt", "wt");
#endif
  for (unsigned dim = 0; dim < 2; dim++)
  {
    const Eigen::Vector3f *eigenVector = eigenvectors[dim];
    assert(eigenVector);

    // compute a local coordiante system
    Eigen::Vector3f x = eigenVector->unitOrthogonal();
    Eigen::Vector3f y = eigenVector->cross(x);
#ifdef DEBUG_TRACE
    {
      fprintf(fp, "Dim %i \n", dim);
      fprintf(fp, "X = %f %f %f", x[0], x[1], x[2]);
      fprintf(fp, "Y = %f %f %f", y[0], y[1], y[2]);
      fprintf(fp, "Z = %f %f %f", eigenVector[0], eigenVector[1], eigenVector[2]);
    }
#endif

    // project the point cloud onto a 2d plane
    for (unsigned i = 0; i < n; i++)
    {
      pcl::PointXYZ p = cloud->points[i];
      Eigen::Vector3f *P = new Eigen::Vector3f(p.x, p.y, p.z);
      Eigen::Vector3f plocal = *P - *G;
      pointsOnPlane[i] = Eigen::Vector2d{plocal.dot(x), plocal.dot(y)};
    }
#ifdef DEBUG_TRACE
    {
      FILE *fpc = nullptr;
      switch (dim)
      {
      case 0:
        fpc = fopen("./circle_dim0.asc", "wt");
        break;
      case 1:
        fpc = fopen("./circle_dim1.asc", "wt");
        break;
      case 2:
        fpc = fopen("./circle_dim2.asc", "wt");
        break;
      }

      for (const CCVector2d &P2D : pointsOnPlane)
      {
        fprintf(fpc, "%f %f 0\n", P2D.x, P2D.y);
      }
      fclose(fpc);
    }
#endif

    // step 4: calculate the circle center and radius on the 2D plane using the Landau Smith algorithm
    Eigen::Vector2d thisCenter2D;
    float thisRadius = 0;
    if (!Landau_Smith(pointsOnPlane, thisCenter2D, thisRadius))
    {
      assert(false);
      // process failed
      return;
    }
#ifdef DEBUG_TRACE
    fprintf(fp, "Center (%f,%f) -radius %f \n" % thisCenter2D[0], thisCenter2D[1], thisRadius);
#endif

    // estimate RMS
    {
      double thisRMS = 0.0;
      for (const Eigen::Vector2d &P2D : pointsOnPlane)
      {
        double r = (P2D - thisCenter2D).norm();
        double error = thisRadius - r;
        thisRMS += error * error;
      }
      thisRMS = sqrt(thisRMS / n);
#ifdef DEBUG_TRACE
      fprintf(fp, "RMS = %f \n", thisRMS);
      fprintf(fp, "===================\n");
#endif
      if (dim == 0 || thisRMS < rms)
      {
        // reposition the circle center in 3D
        center = *G + static_cast<float>(thisCenter2D[0]) * x + static_cast<float>(thisCenter2D[1]) * y;
        normal = *eigenVector;
        radius = thisRadius;
        rms = thisRMS;
      }
    }
  }
#ifdef DEBUG_TRACE
  fclose(fp);
#endif
}

static bool Landau_Smith(const std::vector<Eigen::Vector2d> &xy, Eigen::Vector2d &center, float &radius)
{
  size_t N = xy.size();
  if (N < 3)
  {
    assert(false);
    return false;
  }

  double p1 = 0.0, p2 = 0.0, p3 = 0.0, p4 = 0.0, p5 = 0.0, p6 = 0.0, p7 = 0.0, p8 = 0.0, p9 = 0.0;

  for (size_t i = 0; i < N; ++i)
  {
    p1 += xy[i][0];
    p2 += xy[i][0] * xy[i][0];
    p3 += xy[i][0] * xy[i][1];
    p4 += xy[i][1];
    p5 += xy[i][1] * xy[i][1];
    p6 += xy[i][0] * xy[i][0] * xy[i][0];
    p7 += xy[i][0] * xy[i][1] * xy[i][1];
    p8 += xy[i][1] * xy[i][1] * xy[i][1];
    p9 += xy[i][0] * xy[i][0] * xy[i][1];
  }

  double a1 = 2 * (p1 * p1 - N * p2);
  double b1 = 2 * (p1 * p4 - N * p3);
  double a2 = b1;
  double b2 = 2 * (p4 * p4 - N * p5);
  double c1 = p2 * p1 - N * p6 + p1 * p5 - N * p7;
  double c2 = p2 * p4 - N * p8 + p4 * p5 - N * p9;

  center[0] = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);                                                                                                        // center along x
  center[1] = (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);                                                                                                        // center along y
  radius = static_cast<float>(sqrt(((p2 + p5) - (2 * p1 * center[0]) + (N * center[0] * center[0]) - (2 * p4 * center[1]) + (N * center[1] * center[1])) / N)); // circle radius

  return true;
}

int main(int argc, char **argv)
{
  //-------------------加载点云-------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(argv[1], *cloud) < 0)
  {
    PCL_ERROR("Couldn't read file \n");
    return -1;
  }
  printPointNum(cloud->points.size(), true);
  FitCircle(cloud);
  return 0;
}