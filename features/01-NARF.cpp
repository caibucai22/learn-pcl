#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/features/range_image_border_extractor.h>
#include<pcl/keypoints/narf_keypoint.h>
#include<pcl/console/parse.h>
#include<pcl/common/file_io.h>
#include<pcl/visualization/range_image_visualizer.h>
#include<pcl/visualization/pcl_visualizer.h>

#include<iostream>
#include"my_functions.h"


typedef pcl::PointXYZ PointType;

// 全局变量
float angular_resolution = 0.5f; // 角坐标分辨率
float support_size = 0.2f; // 感兴趣的尺寸
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false; // 是否将所有不可见的点看作最大距离


void printUsage(const char* progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
        << "Options:\n"
        << "-------------------------------------------\n"
        << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
        << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
        << "-m           Treat all unseen points as maximum range readings\n"
        << "-s <float>   support size for the interest points (diameter of the used sphere - "
        << "default " << support_size << ")\n"
        << "-h           this help\n"
        << "\n\n";
}

int main03(int argc,char** argv)
{
    printArgs(argc, argv);
    
    // 解析命令行参数
    if (pcl::console::find_argument(argc, argv, "-h") >= 0){
        printUsage(argv[0]);
        return 0;
    }

    if (pcl::console::find_argument(argc, argv, "-m") >= 0) {
        setUnseenToMaxRange = true;
        std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0) {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-s", support_size) >= 0) {
        std::cout << "Setting support size to " << support_size << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0) {
        std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
    }
    angular_resolution = pcl::deg2rad(angular_resolution);

    // 读取 pcd , 没有指定，创建样本点
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
    if (!pcd_filename_indices.empty()) {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
        {
            std::cerr << "open file " << filename << " failed";
            return 0;
        }
        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
            point_cloud.sensor_origin_[1]
            , point_cloud.sensor_origin_[2])) *
            Eigen::Affine3f(point_cloud.sensor_orientation_);

        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1) {
            std::cout<< "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
        }
    }
    else {
        setUnseenToMaxRange = true;
        std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f) {
            for (float y = -0.5f; y < 0.5f; y+= 0.01f)
            {
                PointType point; point.x = x; point.y = y; point.z = 2.0f - y;
                point_cloud.points.push_back(point);
            }
        }
        point_cloud.width = point_cloud.size(); point_cloud.height = 1;
    }

    // 从点云数据 创建深度图像
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;

    range_image.createFromPointCloud(point_cloud,
        angular_resolution,
        pcl::deg2rad(360.0f),
        pcl::deg2rad(180.0f),
        scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // 打开3d 可视化窗口 并添加 点云
    pcl::visualization::PCLVisualizer viewer("3d viewer");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f,"global"); // 添加坐标系

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 1, "range image");

    viewer.initCameraParameters();

    // 显示深度图像
    pcl::visualization::RangeImageVisualizer range_image_widget("range image");
    range_image_widget.showRangeImage(range_image);

    // 提取 NARF 关键点
    pcl::RangeImageBorderExtractor range_image_border_extractor; // 创建边界提取器
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor); // 创建 NARF 对象
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices; //存储关键点索引
    narf_keypoint_detector.compute(keypoint_indices); 

    std::cout << "found " << keypoint_indices.size() << " key points .\n";

    // 3d viewer 窗口中显示关键点
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
    keypoints.resize(keypoint_indices.size());

    for (size_t i = 0; i < keypoint_indices.size(); i++)
    {
        keypoints[i].getVector3fMap() = range_image[keypoint_indices[i]].getVector3fMap();
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while (!viewer.wasStopped())
    {
        range_image_widget.spinOnce(); // 处理GUI 事件
        viewer.spinOnce();
        pcl_sleep(0.01);
    }

	return 0;
}