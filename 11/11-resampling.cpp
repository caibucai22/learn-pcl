#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h> // 使用 search 空间下的 Kdtree

int main111(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("ism_train_cat.pcd", *cloud);

	// 创建 Kdtree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// 创建输出 pointnormal 类型的 存储
	pcl::PointCloud<pcl::PointNormal> mls_points;

	//
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// 设置参数
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	// reconstruct
	mls.process(mls_points);
	// save output
	if (mls_points.size() > 0)
	{
		pcl::io::savePCDFileASCII("target-mls.pcd", mls_points);
	}
	else
	{
		std::cerr << "保存数据为空." << std::endl;
	}
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}

	return 0;
}