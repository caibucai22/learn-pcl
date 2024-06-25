#include<pcl/io/io.h>
#include<pcl/point_types.h>
#include<pcl/features/pfh.h>
#include<pcl/common/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/features/normal_3d.h> // 估计法线

int main104(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	// 估计法线
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_tree(new pcl::search::KdTree<pcl::PointXYZ>);


	ne.setSearchMethod(normal_tree);
	ne.setInputCloud(cloud);
	ne.setRadiusSearch(.03);
	ne.compute(*normals);

	// 创建 PFH 估计对象
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;

	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);

	//如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	// 创建kdtree 给 pfh
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pfh.setSearchMethod(tree);

	// 输出
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	//使用半径在5厘米范围内的所有邻元素。

	//注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
	
	pfh.setRadiusSearch(0.05);

	pfh.compute(*pfhs);

	// 可视化
	pfh.compute(*pfhs);

	// 直方图可视化
	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*pfhs, 300);
	plotter.plot();


	return 0;
}