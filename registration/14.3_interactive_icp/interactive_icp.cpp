
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
  printf("Rotation matrix : \n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector : \n");
  printf("t = < %6.3f %6.3f %6.3f > \n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *nothing)
{
  if (event.getKeySym() == "space" && event.keyDown())
    next_iteration = true;
}

int main(int argc, char *argv[])
{
  PointCloudT::Ptr cloud_in(new PointCloudT);
  PointCloudT::Ptr cloud_tr(new PointCloudT);
  PointCloudT::Ptr cloud_icp(new PointCloudT);

  if (argc < 2)
  {
    printf("Usage : \n");
    printf("\t\t file.ply number of ICP iterations \n", argv[0]);
    PCL_ERROR("prvoide one ply file \n");
    return -1;
  }
  int iterations = 1;
  if (argc > 2)
  {
    // pass the number of iteration as an arg
    iterations = atoi(argv[2]);
    if (iterations < 1)
    {
      PCL_ERROR("Number of initial iteartions must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic();
  if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0)
  {
    PCL_ERROR("Error loading cloud %s. \n", argv[1]);
    return (-1);
  }

  std::cout << "\nLoaded file " << argv[1] << "(" << cloud_in->points.size()
            << " points) in " << time.toc() << " ms\n"
            << std::endl;
  // 使用刚性矩阵变换原始点云
  // cloud_in
  // cloud_tr cloud_icp
  // cloud_tr 用于显示的备份

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
  // A rotation matrix
  double theta = M_PI / 8;
  transformation_matrix(0, 0) = std::cos(theta);
  transformation_matrix(0, 1) = -sin(theta);
  transformation_matrix(1, 0) = sin(theta);
  transformation_matrix(1, 1) = std::cos(theta);

  // A translation on Z axis (0.4 meters)
  transformation_matrix(2, 3) = 0.4;

  // Display in terminal the transformation matrix

  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix(transformation_matrix);

  // executing the transformation
  pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);

  *cloud_tr = *cloud_icp; // backup

  // ICP 对象创建 设置参数
  time.tic();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(iterations);
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(cloud_in);
  icp.align(*cloud_icp);
  icp.setMaximumIterations(1);

  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << "ms " << std::endl;

  if (icp.hasConverged())
  {
    std::cout << "\n ICP has converged, score is " << icp.getFitnessScore() << std::endl;
    std::cout << "\n ICP transformation " << iterations << " : cloud_icp->cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);
  }
  else
  {
    PCL_ERROR("\n ICP has not converged.\n");
    return (-1);
  }

  // visualization
  pcl::visualization::PCLVisualizer viewer("interactive ICP demo");
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  // color
  float bckgr_gray_level = 0.0;
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // original cloud white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in,
                                                                            (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
  viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
  viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP point cloud red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
  viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

  // add text descriptions in each view
  viewer.addText("White: original point cloud \nGreen: Matrix transformed point cloud", 10, 15, 16,
                 txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText("White: original point cloud \nRed: ICP aligned point cloud", 10, 15, 16,
                 txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str();
  viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1280, 1024);

  viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();

    if (next_iteration)
    {
      time.tic();

      icp.align(*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms " << std::endl;

      if (icp.hasConverged())
      {
        printf("\033[11A");
        printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());

        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

        transformation_matrix *= icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);

        ss.str("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str();
        viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
      }
      else
      {
        PCL_ERROR("\n ICP has not converged.\n");
        return -1;
      }
    }
    next_iteration = false;
  }
  return 0;
}