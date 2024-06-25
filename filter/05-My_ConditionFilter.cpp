#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/common/time.h>
#include<pcl/filters/conditional_removal.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<chrono>
#include<thread>

#include "my_functions.h"

using namespace std;

int
main9(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("./ro.pcd", *cloud)) {
		PCL_ERROR("could not read file\n");
		return (-1);
	}
	// 条件滤波
	// 定义条件
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud(new pcl::ConditionAnd<pcl::PointXYZ>);

	range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 13.0)));
	range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 15.0)));

	// 曲率条件
	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cloud2(pcl::ConditionOr<pcl::PointNormal>);

	/*range_cloud2->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new
		pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, 1)));*/
	pcl::ConditionalRemoval<pcl::PointXYZ> cr;
	cr.setInputCloud(cloud);
	cr.setCondition(range_cloud);
	cr.setKeepOrganized(true); // 保存点云结构 数目不变 用nan代替
	cr.filter(*cloud_filtered);

	// info print
	cout << "the num of points: " << cloud->points.size() << endl;
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);

	cout << "after filtering, the num of points: " << cloud_filtered->points.size() << endl;

	visualizeCloud(cloud, cloud_filtered);

}