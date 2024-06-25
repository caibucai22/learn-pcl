#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>//贪婪投影三角化算法类定义的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
int
main()
{
	// --------------------------------加载点云数据---------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("./bunny.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//----------------------------------法线估计-----------------------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//-----------------------------连接XYZ和法向量字段-----------------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//-------------------------------定义搜索树对象--------------------------------
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//-------------------------------贪婪投影三角化--------------------------------
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	pcl::PolygonMesh triangles;          // 存储最终三角化的网格模型
	gp3.setSearchRadius(0.015);          // 设置连接点之间的最大距离（即三角形的最大边长）
	gp3.setMu(2.5);                      // 设置被样本点搜索其临近点的最远距离，为了适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100); // 设置样本点可搜索的邻域个数
	gp3.setMaximumSurfaceAngle(M_PI / 4);// 设置某点法线方向偏离样本点法线方向的最大角度
	gp3.setMinimumAngle(M_PI / 18);      // 设置三角化后得到三角形内角的最小角度
	gp3.setMaximumAngle(2 * M_PI / 3);   // 设置三角化后得到三角形内角的最大角度
	gp3.setNormalConsistency(false);     // 设置该参数保证法线朝向一致

	// Get result
	gp3.setInputCloud(cloud_with_normals);// 设置输入包含法线的点云
	gp3.setSearchMethod(tree2);           // 设置搜索方式
	gp3.reconstruct(triangles);           // 重建提取三角化
	////cout << triangles;
	//pcl::io::saveVTKFile("mesh.vtk", triangles);
	//--------------------------------附加顶点信息-------------------------------
	vector<int> parts = gp3.getPartIDs();
	vector<int> states = gp3.getPointStates();
	//---------------------------------结果可视化--------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud, 255, 255, 255);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, color_in, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);

	viewer->addPolygonMesh(triangles, "my", v2);

	//viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}

