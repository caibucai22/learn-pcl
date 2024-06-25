#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/features/integral_image_normal.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/point_types.h>
#include<pcl/features/normal_3d.h>

int main101()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_lms400_downsampled.pcd", *cloud);

	// 创建法线估计向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// 创建一个空的kdtree对象，并把它传递给法线估计向量
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	//存储法线
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);

	// 存储特征值为点云
	pcl::PCDWriter writer;
	writer.write<pcl::Normal>("table_cloud_normals.pcd", *cloud_normals, false);

	// 可视化
	pcl::visualization::PCLVisualizer viewer("pcl viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}