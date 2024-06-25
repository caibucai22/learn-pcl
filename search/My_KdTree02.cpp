#include "my_functions.h"
#include<pcl/kdtree/kdtree_flann.h>
#include<string>
#include<vector>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PCDReader reader;
	reader.read("data//A3 - cloud.pcd", *cloud);
	cout << "the num of points: " << cloud->size();

	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
	kdtree.setInputCloud(cloud);

	pcl::PointXYZRGBA searchPoint;
	searchPoint.x = 0.0;
	searchPoint.y = 0.0;
	searchPoint.z = 0.0;

	// ---------------- 可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->setWindowName("k近邻连线可视化");
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud");
	string lineId = "line";

	stringstream ss;

	//--------------------k近邻搜索
	int k = 10;
	vector<int> pointIdx;
	vector<float> distance;

	cout << " search point (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")" << endl;

	if (kdtree.nearestKSearch(searchPoint, k, pointIdx, distance) > 0)
	{
		for (size_t i = 0; i < pointIdx.size(); ++i)
		{
			ss << i;
			lineId += ss.str();
			viewer->addLine<pcl::PointXYZRGBA>(cloud->points[pointIdx[i]], searchPoint, 0, 255, 0, lineId);
		}
	}


	// --------------------半径邻搜索
	vector<int> pointIdx2;
	vector<float> distance2;
	float radius = 0.05;

	cout << " search point (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")" << endl;
	cout << " search radius: " << radius << endl;

	if (kdtree.radiusSearch(searchPoint, radius, pointIdx2, distance2) > 0)
	{
		for (size_t i = 0; i < pointIdx2.size(); ++i)
		{
			ss << i;
			lineId += ss.str();
			viewer->addLine<pcl::PointXYZRGBA>(cloud->points[pointIdx2[i]], searchPoint, 1, 58, 82, lineId);
		}
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}

	return 0;
}