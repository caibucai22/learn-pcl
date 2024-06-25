#include "my_functions.h"
#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/filters/conditional_removal.h>
using namespace std;

int main10()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("./bunny.pcd", *cloud) != 0)
	{
		return -1;
	}
	cout << "the num of points: " << cloud->points.size() << endl;
	// 计算法向和曲率
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setNumberOfThreads(4); // 设置openMP的线程数
	n.setSearchMethod(tree);
	n.setKSearch(10);
	n.compute(*normals);
	// 连接xyz和法向量字段
	pcl::PointCloud<pcl::PointNormal>::Ptr cnormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cnormals);
	// 创建滤波器
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_conditional(new pcl::PointCloud<pcl::PointNormal>);
	float threshold = 0.01;

	// 创建条件定义对象
	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cloud(new pcl::ConditionOr<pcl::PointNormal>());

	range_cloud->addComparison(pcl::FieldComparison < pcl::PointNormal >::ConstPtr(
		new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)
	));

	// 条件滤波器
	pcl::ConditionalRemoval<pcl::PointNormal> condrem;
	condrem.setCondition(range_cloud);
	condrem.setInputCloud(cnormals);
	condrem.setKeepOrganized(false);
	condrem.filter(*cloud_conditional);

	// 获取滤波后点云XYZ坐标
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_conditional, *finalCloud);

	cout << "after filter the num of points: " << finalCloud->points.size() << endl;
	visualizeCloud(cloud, finalCloud);

	return 0;
}