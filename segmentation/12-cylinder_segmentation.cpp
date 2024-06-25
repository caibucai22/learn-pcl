#include<pcl/ModelCoefficients.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/passthrough.h>
#include<pcl/features/normal_3d.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main123(int argc, char** argv)
{

	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //分割对象
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract; // 点提取对象
	pcl::ExtractIndices<pcl::Normal> extract_normals; // 法线提取对象
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	//dataset
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients),
		coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices),
		inliers_cylinder(new pcl::PointIndices);

	// read
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
	std::cerr << "pointcloud has: " << cloud->points.size() << " data points." << std::endl;

	// 直通滤波
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	std::cerr << " pointcloud after has: " << cloud_filtered->points.size() << " data points" << std::endl;

	// 过滤后点云进行法线估计
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// 设置分割
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);

	// 分割
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "plane coefficients: " << *coefficients_plane << std::endl;

	// 从点云中抽取分割的处在平面中的点集
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// 存储分割平面到点云文件
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	std::cerr << " pointcloud representing the planar component: "
		<< cloud_plane->points.size() << " data points."
		<< std::endl;
	writer.write("table_plane.pcd", *cloud_plane, false); // 分割得到的平面

	// 移除平面 提取剩余部分
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	// 针对圆柱体 创建分割对象
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// 获取 圆柱体 和 系数
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << " cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// 保存到文件
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);

	if (cloud_cylinder->points.empty())
		std::cerr << "can't find the cylindrical component." << std::endl;
	else {
		std::cerr << "pointcloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points. \n";
		writer.write("table_cylinder.pcd", *cloud_cylinder, false);
	}
	// 可视化
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("three"));
	int v1(0);
	int v2(1);
	int v3(2);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // xmin ymin xmax ymax 窗口坐标
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2); // 
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v3);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_red(cloud, 255, 0, 0);
	viewer->addPointCloud(cloud, cloud_out_red, "cloud_out1", v1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_green(cloud, 0, 255, 0);
	viewer->addPointCloud(cloud, cloud_out_red, "cloud_out2", v2); // 平面

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_blue(cloud, 0, 0, 255);
	viewer->addPointCloud(cloud, cloud_out_red, "cloud_out3", v3); // 园柱

	// 1. 阻塞式
	viewer->spin();

	//// 2. 非阻塞式
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce();
	//}

	return (0);
}