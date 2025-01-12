#include "../my_functions.h"

using namespace std;

struct sphere
{
	float centerX;
	float centerY;
	float centerZ;
	float radius;
};

using PointT = pcl::PointXYZRGB;

int main(int argc, char **argv)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("cloud not read pcd file");
		return -1;
	}
	cout << "the num of point: " << cloud->points.size() << endl;

	sphere SP;
	SP.centerX = 0.1181;
	SP.centerY = -0.1363;
	SP.centerZ = -0.1567;
	SP.radius = 5.00;

	// 计算投影后点的坐标
	pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		PointT points;
		float d = cloud->points[i].getVector3fMap().norm();
		points.x = (cloud->points[i].x) * SP.radius / d + SP.centerX;
		points.y = (cloud->points[i].y) * SP.radius / d + SP.centerY;
		points.z = (cloud->points[i].z) * SP.radius / d + SP.centerZ;
		points.rgb = cloud->points[i].rgb;
		cloud_projected->push_back(points);
	}
	cout << "after projecting the num of point: " << cloud_projected->points.size() << endl;
	pcl::io::savePCDFileBinary("cloud_projected.pcd", *cloud_projected);

	visualizeCloud(cloud, cloud_projected);

	return 0;
}