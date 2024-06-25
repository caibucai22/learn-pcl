#include "my_functions.h"

void printArgs(int argc, char** argv)
{
	printf("argc: %d\n", argc);
	for (int i = 0; i < argc; i++)
		printf("arg %d is %s \n", i, argv[i]);
}

void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("show cloud"));
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("point cloud", 10, 10, "v1_text", v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filter cloud", 10, 10, "v2_text", v2);

	// °´ÕÕz ×Ö¶Î½øÐÐäÖÈ¾
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_processed", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
}

void loadPCD(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr ptr)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *ptr) == -1)
	{
		PCL_ERROR("Couldn't read pcd file \n");
		return;
	}
}

