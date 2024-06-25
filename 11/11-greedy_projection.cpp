#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h> // 贪婪投影三角化算法

int main113(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("bun0.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	// normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); // 用于法线估计的kdtree
	tree->setInputCloud(cloud);

	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);		// knn
	ne.compute(*normals); // 估计法线存储到其中

	// 连接字段 xyz 和 normal
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	// 定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// init objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // 定义三角化对象
	pcl::PolygonMesh triangles;																// 存储最终三角化的网络模型

	// set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025); // 设置连接点之间的最大距离 即 三角形最大边长

	// 设置参数
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);	// 设置样本点可搜索的领域个数
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度 45
	gp3.setMinimumAngle(M_PI / 18);				// 设置三角化后得到的三角形内角的最小的角度为10
	gp3.setMaximumAngle(2 * M_PI / 3);		// 设置三角化后得到的三角形内角的最小的角度为120
	gp3.setNormalConsistency(false);			// 设置参数保证法线 朝向一致

	// get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// 附加顶点信息
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	return 0;
}