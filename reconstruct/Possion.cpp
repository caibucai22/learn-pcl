#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("./bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read pcd file!\n");

	}

	//----------------------------------法线估计------------------------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	n.compute(*normals);
	//-------------------------------连接法线和坐标---------------------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//---------------------------------泊松重建-------------------------------------
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	pn.setDepth(6);              // 设置将用于表面重建的树的最大深度
	pn.setMinDepth(2);
	pn.setScale(1.25);           // 设置用于重建的立方体的直径与样本的边界立方体直径的比值
	pn.setSolverDivide(3);       // 设置块高斯-塞德尔求解器用于求解拉普拉斯方程的深度。
	pn.setIsoDivide(6);          // 设置块等表面提取器用于提取等表面的深度
	pn.setSamplesPerNode(10);    // 设置每个八叉树节点上最少采样点数目
	pn.setConfidence(false);     // 设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理
	pn.setManifold(false);       // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); // 设置是否输出为多边形(而不是三角化行进立方体的结果)。

	//--------------------------------保存重建结果-----------------------------------
	pcl::PolygonMesh mesh;
	pn.performReconstruction(mesh);
	// pcl::io::savePLYFile("object_mesh.ply", mesh);
	 //------------------------------可视化重建结果----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("泊松曲面重建");
	viewer->addPolygonMesh(mesh, "my");
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}

