#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

#include "../my_functions.h"

using namespace std;

int main(int argc, char **argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile(argv[1], *cloud))
	{
		PCL_ERROR("could not read file\n");
		return (-1);
	}
	cout << "load success, the num of points: " << cloud->points.size() << endl;

	// condition
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud(new pcl::ConditionAnd<pcl::PointXYZ>);

	range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.1)));
	range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 1)));

	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cloud2(pcl::ConditionOr<pcl::PointNormal>);

	/*range_cloud2->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new
		pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, 1)));*/

	pcl::ConditionalRemoval<pcl::PointXYZ> cr;
	cr.setInputCloud(cloud);
	cr.setCondition(range_cloud);
	cr.setKeepOrganized(true);
	cr.filter(*cloud_filtered);

	// info print
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);

	cout << "after filtering, the num of points: " << cloud_filtered->points.size() << endl;

	visualizeCloud(cloud, cloud_filtered);
}