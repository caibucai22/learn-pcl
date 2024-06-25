#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

int main112(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
			cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
			cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);

	// 建立过滤器消除杂散的NaN
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z"); // 设置分割字段为 z 字段
	pass.setFilterLimits(0, 1.1); // 设置分割阈值 (0,1.1)
	pass.filter(*cloud_filtered);
	std::cerr << "point cloud after filtering has"
						<< cloud_filtered->points.size() << "data points. \n";

	// 分割
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // 存储分割后的点云

	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 设置优化系数
	seg.setOptimizeCoefficients(true);
	// mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	std::cerr << "pointcloud after segmentation has: "
						<< inliers->indices.size() << " inliers." << std::endl;

	// project the model inliers
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE); // 设置投影模型
	proj.setIndices(inliers);
	proj.setInputCloud(cloud_filtered);
	proj.setModelCoefficients(coefficients); // 将估计得到的平面coefficients参数设置为投影平面模型系数
	proj.filter(*cloud_projected);

	std::cerr << "pointcloud after projection has: "
						<< cloud_projected->points.size() << " data points." << std::endl;

	// 存储提取多边形上的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);

	std::cerr << "concave hull has: " << cloud_hull->points.size()
						<< " data points." << std::endl;

	pcl::PCDWriter writer;
	writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

	return 0;
}