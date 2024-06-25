#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// 显示点云自带的颜色信息
boost::shared_ptr<pcl::visualization::PCLVisualizer> colorHandler(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D cloud"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

// 根据点云字段进行上色
/*
x
y
z
normal_x （X方向上的法线）
normal_y （Y方向上的法线）
normal_z （Z方向上的法线）
rgb （颜色）
curvature （曲率）
*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> genericHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D cloud"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(cloud, "y");
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

// 自定义单一颜色
boost::shared_ptr<pcl::visualization::PCLVisualizer> customHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 255, 255);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

// 随机上色
boost::shared_ptr<pcl::visualization::PCLVisualizer> randomHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

int main05d()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::io::loadPCDFile("bunny.pcd", *cloud_rgb);

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> random_viewer = randomHandler(cloud);

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> custom_viewer = customHandler(cloud);

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> generic_viewer = genericHandler(cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> color_viewer = colorHandler(cloud_rgb); // 无法正常显示 读取 bunny.pcd

	// 显示2种方法
	// 1. 阻塞式
	color_viewer->spin();

	// 2. 非阻塞式
	/*while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}*/

	return 0;
}