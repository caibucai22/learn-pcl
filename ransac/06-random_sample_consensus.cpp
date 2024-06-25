#include<iostream>
#include<pcl/console/parse.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/point_types.h>
#include<pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<pcl/sample_consensus/sac_model_sphere.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>

#include<chrono>
#include<thread>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3d viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem(1.0, "global");
	viewer->initCameraParameters();
	return (viewer);
}

int main062(int argc, char** argv)
{

	// 初始化点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 填充点云数据
	cloud->width = 500;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
		{
			cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
			cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
			if (i % 5 == 0)
			{
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
			}
			else if (i % 2 == 0)
			{
				cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
					- (cloud->points[i].y * cloud->points[i].y));
			}
			else
			{
				cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
					- (cloud->points[i].y * cloud->points[i].y));
			}

		}
		else
		{
			cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
			cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
			if (i % 2 == 0)
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
			else
			{
				cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
			}
		}
	}

	std::vector<int> inliers;

	// 创建随机采样一致性对象
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

	if (pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}else if (pcl::console::find_argument(argc, argv, "-sf") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	// 复制估算模型的所有的局内点到save_cloud 中
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *save_cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
		viewer = simpleVis(save_cloud);
	else
	{
		viewer = simpleVis(cloud);
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	return 0;
}