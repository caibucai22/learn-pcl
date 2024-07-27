#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>

#include <Eigen/Core>

#include "../my_functions.h"

void pcaByPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projet_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  pca.project(*cloud, *pca_projet_cloud);
  std::cout << "eigenvectors: \n"
            << pca.getEigenVectors() << std::endl;
  std::cout << "eigenvalues: \n"
            << pca.getEigenValues() << std::endl;
}

void pcaByEigen(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                Eigen::Vector3f &eigenValuesPCA,
                Eigen::Matrix3f &eigenVectorsPCA,
                Eigen::Vector4f &pcaCentroid)
{
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenValuesPCA = eigen_solver.eigenvalues();

  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); // 校正主方向间垂直
  eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
  eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
  std::cout << "centroid(4x1):\n"
            << pcaCentroid << std::endl;
  std::cout << "eigenvalues  va(3x1):\n"
            << eigenValuesPCA << std::endl;
  std::cout << "eigvenVectors ve(3x3):\n"
            << eigenVectorsPCA << std::endl;
}

// 按特征值从大到小排列 特征向量
void projectToPCAPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix3f eigenVectorsPCA,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected, int *vector_idxs, bool save = true)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  int numPoints = cloud->points.size();

  Eigen::MatrixXf centeredPoints(numPoints, 3);
  // centering
  for (int i = 0; i < numPoints; ++i)
  {

    pcl::PointXYZ p = cloud->points.at(i);
    centeredPoints(i, 0) = p.x - centroid[0];
    centeredPoints(i, 1) = p.y - centroid[1];
    centeredPoints(i, 2) = p.z - centroid[2];
  }

  // projecting matrix
  int idx1 = vector_idxs[0], idx2 = vector_idxs[1];
  Eigen::Matrix3f projectionMatrix;
  projectionMatrix.col(0) = eigenVectorsPCA.col(idx1);
  projectionMatrix.col(1) = eigenVectorsPCA.col(idx2);
  projectionMatrix.col(2) = eigenVectorsPCA.col(idx1).cross(eigenVectorsPCA.col(idx2));

  // projecting
  Eigen::MatrixXf projected_points = centeredPoints * projectionMatrix;

  for (int i = 0; i < numPoints; i++)
  {
    // cloud_projected->points.push_back({projected_points(i, 0) + centroid[0],
    //                                    projected_points(i, 1) + centroid[1],
    //                                    centroid[2]});
    cloud_projected->points.push_back({projected_points(i, 0),
                                       projected_points(i, 1),
                                       0.0});
  }
  if (save)
  {
    cloud_projected->width = numPoints;
    cloud_projected->height = 1;
    pcl::PCDWriter writer;
    writer.write("projeted_to_PCA_plane.pcd", *cloud_projected, false);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloudToStandardCoordinates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const Eigen::Matrix3f &eigenVectorsPCA,
    const Eigen::Vector4f &pcaCentroid)
{
  Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
  transformMatrix.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  // transformMatrix.block<3, 3>(0, 0) = eigenVectorsPCA.inverse(); // 正交矩阵 转置=逆矩阵
  transformMatrix.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose() * pcaCentroid.head<3>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *transformedCloud, transformMatrix);

  return transformedCloud;
}

void computeXYZProjections(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  int num = cloud->points.size();
  // 投影到 XY 平面
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyProjection(new pcl::PointCloud<pcl::PointXYZ>);
  xyProjection->height = num;
  xyProjection->width = 1;
  // 投影到 YZ 平面
  pcl::PointCloud<pcl::PointXYZ>::Ptr yzProjection(new pcl::PointCloud<pcl::PointXYZ>);
  yzProjection->height = num;
  yzProjection->width = 1;
  // 投影到 XZ 平面
  pcl::PointCloud<pcl::PointXYZ>::Ptr xzProjection(new pcl::PointCloud<pcl::PointXYZ>);
  xzProjection->height = num;
  xzProjection->width = 1;

  for (const auto &point : cloud->points)
  {
    xyProjection->points.emplace_back(pcl::PointXYZ(point.x, point.y, 0.0f));
    xzProjection->points.emplace_back(pcl::PointXYZ(point.x, 0.0f, point.z));
    yzProjection->points.emplace_back(pcl::PointXYZ(0.0f, point.y, point.z));
  }

  // 保存投影结果
  pcl::PCDWriter writer;
  writer.write("xy.pcd", *xyProjection, false);
  writer.write("xz.pcd", *xzProjection, false);
  // 主方向投影
  writer.write("yz.pcd", *yzProjection, false);
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  loadPCD_(argv[1], cloud);
  printPointNum(cloud->points.size(), true);

  printf("PCA by PCL\n");
  pcaByPCL(cloud);

  std::cout << "\n=========\n"
            << std::endl;
  printf("PCA by Eigen\n");
  Eigen::Vector3f eigenValuesPCA;
  Eigen::Matrix3f eigenVectorsPCA;
  Eigen::Vector4f pcaCentroid;
  pcaByEigen(cloud, eigenValuesPCA, eigenVectorsPCA, pcaCentroid);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
  int chosed_plane_idxs[2] = {2, 1};
  // projectToPCAPlane(cloud, eigenVectorsPCA, cloud_projected, chosed_plane_idxs);
  pcl::PointCloud<pcl::PointXYZ>::Ptr stdAxisPointCloud = transformPointCloudToStandardCoordinates(
      cloud,
      eigenVectorsPCA,
      pcaCentroid);
  pcl::PCDWriter writer;
  writer.write("transfrom_to_stdAxis.pcd", *stdAxisPointCloud, false);
  computeXYZProjections(stdAxisPointCloud);

  // float dotProduct01 = eigenVectorsPCA.col(0).dot(eigenVectorsPCA.col(1));
  // float dotProduct02 = eigenVectorsPCA.col(0).dot(eigenVectorsPCA.col(2));
  // float dotProduct12 = eigenVectorsPCA.col(1).dot(eigenVectorsPCA.col(2));

  // std::cout << "Dot product between eigen vector 0 and 1: " << dotProduct01 << std::endl;
  // std::cout << "Dot product between eigen vector 0 and 2: " << dotProduct02 << std::endl;
  // std::cout << "Dot product between eigen vector 1 and 2: " << dotProduct12 << std::endl;

  // visualize
  // vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  // renderer->SetBackground(0.1, 0.1, 0.1); // 设置背景颜色

  // // 创建渲染窗口
  // vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  // renderWindow->AddRenderer(renderer);
  // renderWindow->SetSize(800, 600);

  // // 创建渲染窗口交互器
  // vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  // renderWindowInteractor->SetRenderWindow(renderWindow);

  // // visualizePointCloudByVTK(renderer, cloud);
  // // visualizePointCloudByVTK(renderer, cloud_projected);
  // drawPointCloudAndEigenVectors(renderer, cloud, eigenVectorsPCA, pcaCentroid);

  // renderWindow->Render();
  // renderWindowInteractor->Start();

  return 0;
}