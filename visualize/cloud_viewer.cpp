#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
	// 画球
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
	// 计数
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 300, 300, "text", 0);

	// FIXME: possible race condition here:
	user_data++;
}

int main(int argc, char *argv[])
{
	// 创建并读取点云
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (argc < 2 || pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("load pcd failed");
		return -1;
	}

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// 渲染点云
	viewer.showCloud(cloud);

	// 只会调用一次
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// 多次调用
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		// 检查线程结束没有
		user_data++;
	}
	return 0;
}
