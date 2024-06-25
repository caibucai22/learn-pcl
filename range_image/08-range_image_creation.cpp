 
#include<pcl/range_image/range_image.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/range_image_visualizer.h>

int main081(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr;

	pcl::io::loadPCDFile("bunny.pcd", *cloud_ptr);
	 
	// angularResolution 模拟大的深度传感器角度分辨率
	float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // 弧度 1
	// maxAngleWidth 为模拟的深度传感器水平最大采样角
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // 弧度 360
	// maxAngleHeight 模拟传感器的垂直方向最大采样角度 都转为 弧度
	float maxAngleHeight = (float)(180.0f * (M_PI) / 180.0f); // 弧度 180
	
	// 传感器的采集位置
	Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	// 深度图像遵循坐标系统
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

	float noise_level = 0.00;
	float min_range = 0.0f;
	int border_size = 1;

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& rangeImage = *range_image_ptr;

	rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	/*
	* range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位） ：
    point_cloud为创建深度图像所需要的点云
    angular_resolution_x深度传感器X方向的角度分辨率
    angular_resolution_y深度传感器Y方向的角度分辨率
    pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
    pcl::deg2rad (180.0f)垂直最大采样角度
    scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
    coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
    noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
    min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
    border_size  设置获取深度图像边缘的宽度 默认为0
	*/
	std::cout << rangeImage << "\n";

	pcl::visualization::PCLVisualizer viewer("3d viewer");
	viewer.setBackgroundColor(1, 1, 1);

	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");*/

	// 添加原始点云
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_image_color_handler(cloud_ptr, 255, 100, 0);
	viewer.addPointCloud(cloud_ptr, org_image_color_handler, "original image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original image");

	viewer.initCameraParameters();
	viewer.addCoordinateSystem();


	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}


	return 0;
}