#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>


int main05()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("show cloud"));
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("point cloud", 10, 10, "v1_text", v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filter cloud", 10, 10, "v2_text", v2);
	
	// 显示点云自带的颜色信息
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
	
	// 按照z 字段进行渲染
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");


	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, "cloud_processed", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
	
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}

	return 0;
}