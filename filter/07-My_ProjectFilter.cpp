#include "my_functions.h"
#include<pcl/ModelCoefficients.h>
#include<pcl/filters/project_inliers.h>

using namespace std;

int
main71()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("ro.pcd", *cloud);
	//  ���� ax+by+cz+d=0 ƽ�� ϵ��a=b=d=0 ,c=1 Ҳ�� x-yƽ�� Z����ص�ȫ��ͶӰ��x-y����
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// �����˲���
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	visualizeCloud(cloud, cloud_projected);

	return 0;
}