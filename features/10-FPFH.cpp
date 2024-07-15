#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/features/normal_3d.h> // 估计法线
#include <time.h>

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("load pcd failed");
		return -1;
	}
	std::cout << "load success, the num of points is " << cloud->points.size() << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	// 估计法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_tree(new pcl::search::KdTree<pcl::PointXYZ>);

	ne.setSearchMethod(normal_tree);
	ne.setInputCloud(cloud);
	ne.setRadiusSearch(.03);
	auto t1 = clock();
	ne.compute(*normals);
	cout << "normal estimation, elapsed time " << double(clock() - t1) / CLOCKS_PER_SEC << " seconds" << endl;

	// 创建 PFH 估计对象
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;

	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);

	// 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	// 创建kdtree 给 pfh
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	fpfh.setSearchMethod(tree);

	// 输出
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

	// 使用半径在5厘米范围内的所有邻元素。

	// 注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
	fpfh.setRadiusSearch(0.05);

	auto t2 = clock();
	fpfh.compute(*fpfhs);
	cout << "compute fpfh feature, elapsed time " << double(clock() - t2) / CLOCKS_PER_SEC << " seconds" << endl;

	// 直方图可视化
	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*fpfhs, 300);
	plotter.plot();

	return 0;
}