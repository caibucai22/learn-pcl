/**
 * @file narf_keypoint_extraction.cpp
 * @author your name (you@domain.com)
 * @brief NARF
 * @version 0.1
 * @date 2024-03-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
// #include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>

// types

typedef pcl::PointXYZ PointType;

// parameters
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; //
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

// help
void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
            << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
               "default "
            << support_size << ")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            << " (default " << (int)rotation_invariant << ")\n"
            << "-h           this help\n"
            << "\n\n";
}

void setViewPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                           look_at_vector[0], look_at_vector[1], look_at_vector[2],
                           up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char **argv)
{
  if (pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return 0;
  }
  if (pcl::console::find_argument(argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    cout << "setting unseen values in range image to maximum range readings.\n";
  }
  if (pcl::console::parse(argc, argv, "-o", rotation_invariant) >= 0)
  {
    cout << "switching rotation invariant feature version " << (rotation_invariant ? "on" : "off") << ".\n";
  }
  int tmp_coordinate_frame;
  if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
    cout << "using coordinate frame " << (int)coordinate_frame << ".\n";
  }
  if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
    cout << "setting support size to " << support_size << ".\n";

  if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
    cout << "setting angular resolution to " << angular_resolution << "deg.\n";

  angular_resolution = pcl::deg2rad(angular_resolution);

  // read pcd file

  pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
  if (!pcd_filename_indices.empty())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
    {
      cerr << "was not able to open file \"" << filename << "\".\n";
      printUsage(argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f(point_cloud.sensor_orientation_);
    // std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
    // if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
    //     std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
  }
  else
  {
    setUnseenToMaxRange = true;
    cout << "\n No *.pcd file given => Generating example point cloud. \n\n";
    for (float x = -0.5f; x <= 0.5f; x += 0.01f)
    {
      for (float y = -0.5f; y <= 0.5f; y += 0.01f)
      {
        PointType point;
        point.x = x;
        point.y = y;
        point.z = 2.0f - y;
        point_cloud.points.push_back(point);
      }
    }
    point_cloud.width = (int)point_cloud.points.size();
    point_cloud.height = 1;
  }

  // create rangeimage from the pointcloud
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  std::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage &range_image = *range_image_ptr;
  range_image.createFromPointCloud(point_cloud, angular_resolution,
                                   pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                   scene_sensor_pose, coordinate_frame,
                                   noise_level, min_range, border_size);
  range_image.integrateFarRanges(far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange();

  // show
  pcl::visualization::PCLVisualizer viewer("3D viewer");
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
  viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  // viewer.addCoordinateSystem (1.0f, "global");
  // PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  // viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters();
  setViewPose(viewer, range_image.getTransformationToWorldSystem());

  // show range image
  pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
  range_image_widget.showRangeImage(range_image);

  //
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage(&range_image);
  narf_keypoint_detector.getParameters().support_size = support_size;

  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute(keypoint_indices);
  std::cout << "found " << keypoint_indices.points.size() << " key points.\n";

  // show keypoints in range image widget
  // for (size_t i = 0; i < keypoint_indices.points.size(); i++)
  // {
  //   range_image_widget.markPoint(keypoint_indices.points[i] % range_image.width,
  //                                keypoint_indices.points[i] / range_image.width);
  // }

  // show keypoints in 3d viewer
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ> &keypoints = *keypoints_ptr;

  keypoints.points.resize(keypoint_indices.points.size());
  for (size_t i = 0; i < keypoint_indices.points.size(); i++)
  {
    keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
  }

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize(keypoint_indices.points.size());
  for (unsigned int i = 0; i < keypoint_indices.size(); ++i)
  {
    keypoint_indices2[i] = keypoint_indices.points[i];
  }

  pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
  narf_descriptor.getParameters().support_size = support_size;
  narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narf_descriptor.compute(narf_descriptors);

  cout << "Extracted " << narf_descriptors.size() << " descriptors for "
       << keypoint_indices.points.size() << " keypoints.\n";

  while (!viewer.wasStopped())
  {
    range_image_widget.spinOnce();
    viewer.spinOnce();
    // pcl_sleep(0.01);
    std::this_thread::sleep_for(std::chrono::milliseconds(100000));
  }

  return 0;
}