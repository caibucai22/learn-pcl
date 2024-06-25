#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
using namespace std;

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
	//-----------------------显示点云-----------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("point clouds", 10, 10, "v1_text", v1);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
	// 按照z字段进行渲染,将z改为x或y即为按照x或y字段渲染
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);

	viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int
main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//读入点云数据
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("E:/data/ro.pcd", *cloud);
	cout << "Cloud before filtering:\n " << *cloud << endl;
	// -----------------统计滤波-------------------
	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	// 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //设置待滤波的点云
	sor.setMeanK(50);           //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
	//即：当判断点的k近邻平均距离(mean distance)大于全局的1倍标准差+平均距离(global distances mean and standard)，则为离群点。


	sor.filter(*cloud_filtered); //存储内点
	cout << "Cloud after filtering: \n" << *cloud_filtered << endl;
	// 保存点云
	pcl::PCDWriter writer;
	// writer.write<pcl::PointXYZ> ("inliers.pcd", *cloud_filtered, true);
	 // 可视化
	VisualizeCloud(cloud, cloud_filtered);

	return (0);
}

