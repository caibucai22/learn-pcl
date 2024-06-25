#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// 随机填充无序点云
	cloud_in->width = 5;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cout << "saved " << cloud_in->points.size() << "data points to input \n";

	for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		std::cout << " " << cloud_in->points[i].x
							<< " " << cloud_in->points[i].y
							<< " " << cloud_in->points[i].z << "\n";
	}
	*cloud_out = *cloud_in;

	// 实现一个简单的刚体变换
	for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	}

	std::cout << "\n\nafter transformation: \n";
	for (size_t i = 0; i < cloud_out->points.size(); i++)
	{
		std::cout << " " << cloud_out->points[i].x
							<< " " << cloud_out->points[i].y
							<< " " << cloud_out->points[i].z << "\n";
	}

	// 创建iterative closest point的实例

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);

	// 创建 final 对象 保存配准后的源点云

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "\n\nhas converged: "
						<< icp.hasConverged()
						<< " score " << icp.getFitnessScore() << "\n";

	const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
	std::cout << matrix << std::endl;
	return 0;
}