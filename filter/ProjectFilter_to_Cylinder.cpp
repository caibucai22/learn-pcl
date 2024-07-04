#include "../my_functions.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (argc < 2 || pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud))
	{
		PCL_ERROR("load pcd failed");
		return -1;
	}

	/*pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(7);
	coefficients->values = { -22.0899, 102.386, 140.61,-0.00935628, 0.999937, -0.00617214,2.06343 };*/
	// 直接指定参数 -> 替代 法线估计 圆柱拟合部分

	// 法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	n.setSearchMethod(tree);
	n.setInputCloud(cloud);
	n.setKSearch(20);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	// 圆柱拟合
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.setModelType(pcl::SACMODEL_CYLINDER); // 设置分割模型为圆柱体模型
	seg.setMethodType(pcl::SAC_RANSAC);				// 设置采用 ransac 算法进行 参数估计
	seg.setNormalDistanceWeight(0.3);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.1);
	seg.setRadiusLimits(2.0, 3.0);

	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	seg.segment(*inliers_cylinder, *coefficients_cylinder); // 执行分割

	cout << "axis coordinate :(" << coefficients_cylinder->values[0] << ","
			 << coefficients_cylinder->values[1] << ","
			 << coefficients_cylinder->values[2] << ")"
			 << endl;

	cout << "axis direction vector :(" << coefficients_cylinder->values[3] << ","
			 << coefficients_cylinder->values[4] << ","
			 << coefficients_cylinder->values[5] << ")"
			 << endl;

	cout << "cylinder radius :(" << coefficients_cylinder->values[6] << ")" << endl;

	// 点云投影到圆柱
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_CYLINDER);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients_cylinder);
	proj.filter(*cloud_projected);

	visualizeCloud(cloud, cloud_projected);

	return 0;
}