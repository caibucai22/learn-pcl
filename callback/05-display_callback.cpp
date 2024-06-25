#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

struct callback_args
{
	bool *isShow;
	pcl::PointCloud<pcl::PointXYZ>::Ptr origin_points;
	pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
};

// 按键事件 回调函数
void kb_callback(const pcl::visualization::KeyboardEvent &event, void *args)
{
	if (event.keyDown() && event.getKeyCode() == 'a')
	{
		std::cout << " 'a' has pressed" << std::endl;
		struct callback_args *data = (struct callback_args *)args;
		if (*(data->isShow))
		{
			data->viewer_ptr->removePointCloud("cloud");
			*(data->isShow) = false;
			std::cout << "remove" << std::endl;
		}
		else
		{
			data->viewer_ptr->addPointCloud(data->origin_points, "cloud");
			*(data->isShow) = true;
			std::cout << "add" << std::endl;
		}
	}
}

int main061(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::console::print_highlight("load cloud \n");

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	viewer->addPointCloud(cloud);

	// 初始化参数
	bool isShow = true;
	struct callback_args kb_args;
	kb_args.isShow = &isShow;
	kb_args.origin_points = cloud;
	kb_args.viewer_ptr = viewer;

	// 设置回调函数
	viewer->registerKeyboardCallback(kb_callback, (void *)&kb_args);
	viewer->spin();

	return 0;
}