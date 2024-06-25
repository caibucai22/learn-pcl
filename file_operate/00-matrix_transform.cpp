
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/common/transforms.h>
#include<pcl/visualization/pcl_visualizer.h>

void showHelp(char* program_name)
{
	std::cout << std::endl;
	std::cout << "usage: " << program_name << "cloud_filename.[pcd][ply]" << std::endl;
	std::cout << " -h: Show this help";
}

int main000(int argc, char** argv)
{
	// show help
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
	{
		showHelp(argv[0]);
		return 0;
	}

	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

	if (filenames.size() != 1)
	{
		filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
		if (filenames.size() != 1) {
			showHelp(argv[0]);
			return -1;
		}
		else {
			file_is_pcd = true;
		}
	}
	// load file
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (file_is_pcd)
	{
		if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
		{
			std::cout << "error loading point cloud" << argv[filenames[0]] << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	else {
		if(pcl::io::loadPLYFile(argv[filenames[0]],*source_cloud) < 0)
		{
			std::cout << "error loading point cloud " << argv[filenames[0]] << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}

	// 创建 4x4 单位阵
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	float theta = M_PI / 4; // 旋转的角度
	transform_1(0, 0) = std::cos(theta);
	transform_1(0, 1) = -1.0*std::sin(theta);
	transform_1(1, 0) = std::sin(theta);
	transform_1(1, 1) = std::cos(theta);
	// row,column

	// x 方向平移
	transform_1(0, 3) = 2.5;

	// 打印 transformation
	printf("method #1 using a matrix4f \n");
	std::cout << transform_1 << std::endl;


	// 第二种 定义变换矩阵的方法
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// x 方向平移
	transform_2.translation() << 2.5, 0.0, 0.0;

	// 旋转
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	// 打印 transformation
	printf("\n method #2: using an affine3f\n");
	std::cout << transform_2.matrix() << std::endl;

	// 执行变换
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	// visualization
	printf("\n point cloud colors : white = original point cloud \n"
		"                       red =  transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("matrix transformation example");

	// 定义显示属性
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_handler(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, source_cloud_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20);
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	// 设定观察位置
	viewer.setPosition(800, 400);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}