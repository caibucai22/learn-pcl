/*
 * @author    : Csy
 * @date      : 2023-12-15 11:18
 * @brief     : 点云 双边滤波 完成后 点数不变 只改变强度
 */

#include "../my_functions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/console/time.h>
#include <omp.h> // without 0.0001306s vs. 0.0001512s

int main(int argc, char *argv[])
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

	if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) < 0)
	{
		PCL_ERROR("load pcd failed");
		return -1;
	}
	std::cout << "load success, the num of points is " << cloud->points.size() << std::endl;
	std::cout << "fields: " << pcl::getFieldsList(*cloud) << std::endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	// ----------------------------建立 kdtree
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(cloud);

	pcl::BilateralFilter<pcl::PointXYZI> bf;
	bf.setInputCloud(cloud);
	// bf.setSearchMethod();
	bf.setHalfSize(0.1);
	bf.setStdDev(0.03);
	bf.filter(*filter_cloud);
	// pcl::io::savePCDFileBinary("box_filtered.pcd", *filter_cloud);

	int count = 0;
	int nthreads, tid;
#pragma omp parallel
	// cout << "Get Thread Num: " << omp_get_thread_num() << endl;
	printf("total %d thread running for loop \n", omp_get_thread_num());

	pcl::console::TicToc time; time.tic(); 
#pragma omp parallel for
	for (int i = 0; i < cloud->size(); ++i)
	{
		if (cloud->points[i].intensity != filter_cloud->points[i].intensity)
			++count;
	}
	cout<<time.toc()/1000<<"s"<<endl;
	printf("the num of intensity changing points is %d\n", count);

	visualizeCloud(cloud, filter_cloud);
	return 0;
}