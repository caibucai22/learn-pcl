#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

struct callback_args
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr origin_points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr chosed_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewer_Ptr;
};

void ap_callback(const pcl::visualization::AreaPickingEvent &event, void *args)
{
	struct callback_args *data = (struct callback_args *)args;
	std::vector<int> indices;

	if (!event.getPointsIndices(indices))
		return;
	for (int i = 0; i < indices.size(); ++i)
	{
		data->chosed_points_3d->push_back(data->origin_points->points[indices[i]]);
	}

	// draw clicked points in red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->chosed_points_3d, 255, 0, 0);
	data->viewer_Ptr->removePointCloud("chosed_points");
	data->viewer_Ptr->addPointCloud(data->chosed_points_3d, red, "chosed_points");
	data->viewer_Ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "chosed_points");
	std::cout << "selected " << indices.size() << " points, now sum is " << data->chosed_points_3d->size() << std::endl;
}

int main063(int argc, char **argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::console::print_highlight("load cloud! \n");

	pcl::visualization::PCLVisualizer viewer;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
	viewer.addPointCloud(cloud, green, "cloud");

	pcl::console::print_highlight("init start \n");
	struct callback_args cb_args;
	cb_args.origin_points = cloud;
	pcl::console::print_highlight("init origin_points \n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr chosed_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
	cb_args.chosed_points_3d = chosed_points_3d;
	pcl::console::print_highlight("init chosed_points_3d \n");

	// 发生异常 未debug 提示 读取访问权限冲突
	cb_args.viewer_Ptr = pcl::visualization::PCLVisualizer::Ptr(&viewer);

	pcl::console::print_highlight("init end \n");

	viewer.registerAreaPickingCallback(ap_callback, (void *)&cb_args);

	pcl::console::print_highlight("press x enter selected model, then press q ... \n");

	viewer.spin();

	system("pause");

	return 0;
}