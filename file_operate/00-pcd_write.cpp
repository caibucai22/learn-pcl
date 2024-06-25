#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<iostream>


int main002(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Ìî³äÊý¾Ý
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	// 
	for (auto& point : cloud)
	{
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("pcd_write.pcd", cloud);
	std::cerr << "saved " << cloud.points.size() << " data points to pcd_write.pcd." << std::endl;


	for (const auto& point : cloud)
	{
		std::cerr << " £¨" << point.x << "," << point.y << "," << point.z << ")" << std::endl;
	}

	return 0;
}