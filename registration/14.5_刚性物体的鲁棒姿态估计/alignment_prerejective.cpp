/**
 * @file alignment_prerejective.cpp
 * @author your name (you@domain.com)
 * @brief 刚性物体的鲁棒性姿态估计
 * @version 0.1
 * @date 2024-03-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;

typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

int main(int argc, char **argv)
{

  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  PointCloudT::Ptr scene(new PointCloudT);
  FeatureCloudT::Ptr object_features(new FeatureCloudT);
  FeatureCloudT::Ptr scene_features(new FeatureCloudT);

  if (argc != 3)
  {
    pcl::console::print_error("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return 1;
  }

  pcl::console::print_highlight("Loading point clouds... \n");
  if (pcl::io::loadPCDFile(argv[1], *object) < 0 || pcl::io::loadPCDFile<PointNT>(argv[2], *scene) < 0)
  {
    pcl::console::print_error("Error loading object/scene file! \n");
    return 1;
  }

  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(object);
  grid.filter(*object);
  grid.setInputCloud(scene);
  grid.filter(*scene);

  // 估计法线
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.01);
  nest.setInputCloud(scene);
  nest.compute(*scene);

  // 特征估计
  pcl::console::print_highlight("Estimating features... \n");
  FeatureEstimationT fest;
  fest.setRadiusSearch(0.025);
  fest.setInputCloud(object);
  fest.setInputNormals(object);
  fest.compute(*object_features);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);

  // 配准
  pcl::console::print_highlight("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(50000);
  align.setNumberOfSamples(3);
  align.setCorrespondenceRandomness(5); // 使用临近特征点的数目
  align.setSimilarityThreshold(0.9);
  align.setMaxCorrespondenceDistance(2.5f * leaf);
  align.setInlierFraction(0.25f);
  {
    pcl::ScopeTime t("Alignment");
    align.align(*object_aligned);
  }

  if (align.hasConverged())
  {
    // Print results
    printf("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size()); // Inliers: 1384/3432

    // show alignment
    pcl::visualization::PCLVisualizer visu("alignment");
    visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin();
  }
  else
  {
    pcl::console::print_error("Alignement failed!\n");
    return 1;
  }

  return 0;
}
