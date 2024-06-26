#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

int main(int, char **)
{
	typedef pcl::PointXYZ PointType;
	typedef pcl::PointCloud<PointType> CloudType;

	CloudType::Ptr cloud(new CloudType);
	cloud->is_dense = false;

	PointType p;
	for (unsigned int i = 0; i < 5; i++)
	{
		p.x = p.y = p.z = static_cast<float>(i);
		cloud->push_back(p);
	}
	std::cout << "cloud has " << cloud->points.size() << " points." << std::endl;

	pcl::PointIndices indices;
	indices.indices.push_back(0);
	indices.indices.push_back(2);

	pcl::ExtractIndices<PointType> extr;
	extr.setInputCloud(cloud);
	extr.setIndices(std::make_shared<const pcl::PointIndices>(indices));
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	extr.filter(*output);

	std::cout << "output has " << output->points.size() << " points." << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>);

	extr.setNegative(true);
	extr.filter(*cloud_other);

	std::cout << "other has " << cloud_other->points.size() << " points." << std::endl;

	return 0;
}