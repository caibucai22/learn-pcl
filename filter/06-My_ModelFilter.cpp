#include "my_functions.h"
#include<pcl/filters/model_outlier_removal.h>

using namespace std;

int
main61()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// generate cloud data
	int noise_size = 500;
	int sphere_data_size = 10000;
	cloud->width = noise_size + sphere_data_size;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	// add noise
	for (size_t i = 0; i < noise_size; ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

	}
	// add sphere
	double rand_x1 = 1;
	double rand_x2 = 1;
	for (size_t i = noise_size; i < noise_size + sphere_data_size; ++i)
	{
		while (pow(rand_x1, 2) + pow(rand_x2, 2) >= 1)
		{
			rand_x1 = (rand() % 100) / (50.0f) - 1;
			rand_x2 = (rand() % 100) / (50.0f) - 1;
		}
		double pre_calc = sqrt(1 - pow(rand_x1, 2) - pow(rand_x2, 2));
		cloud->points[i].x = 2 * rand_x1 * pre_calc;
		cloud->points[i].y = 2 * rand_x2 * pre_calc;
		cloud->points[i].z = 1 - 2 * (pow(rand_x1, 2) + pow(rand_x2, 2));
		rand_x1 = 1;
		rand_x2 = 1;
	}

	// filter sphere
	// generate model
	// model parameter for this sphere
	// x 0 y 0 z 0 r 1
	pcl::ModelCoefficients sphere_coeff;
	sphere_coeff.values.resize(4);
	sphere_coeff.values[0] = 0;
	sphere_coeff.values[1] = 0;
	sphere_coeff.values[2] = 0;
	sphere_coeff.values[3] = 1;

	pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
	sphere_filter.setModelCoefficients(sphere_coeff);
	sphere_filter.setThreshold(0.05);
	sphere_filter.setModelType(pcl::SACMODEL_SPHERE);
	sphere_filter.setInputCloud(cloud);
	sphere_filter.filter(*cloud_sphere_filtered);

	std::cerr << "sphere before filtering: " << cloud->points.size() << std::endl;

	std::cerr << "sphere after filtering: " << cloud_sphere_filtered->points.size() << std::endl;

	visualizeCloud(cloud, cloud_sphere_filtered);
	return 0;
}