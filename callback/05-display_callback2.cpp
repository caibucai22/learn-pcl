#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

struct callback_args
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
};

void pp_callback(const pcl::visualization::PointPickingEvent &event, void *args)
{
	struct callback_args *data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	int index = event.getPointIndex();
	std::cout << "index: " << index << std::endl;
	pcl::PointXYZ current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);

	// draw clicked points in red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);
	data->viewer_ptr->removePointCloud("clicked_points");
	data->viewer_ptr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << "selected point is (" << current_point.x << "," << current_point.y << "," << current_point.z << ")" << std::endl;
}

int main062()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::console::print_highlight("load cloud !\n");

	pcl::visualization::PCLVisualizer viewer;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
	viewer.addPointCloud(cloud, green, "cloud");

	// add point picking callback to viewer
	struct callback_args cb_args;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewer_ptr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
	viewer.registerPointPickingCallback(pp_callback, (void *)&cb_args);
	pcl::console::print_highlight("shift+click on three floor points, then press 'Q' ... \n");

	viewer.spin();

	system("pause");
	return 0;
}