#include<pcl/io/io.h>
#include<pcl/point_types.h>
#include<pcl/features/pfh.h>
#include<pcl/common/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/features/normal_3d.h> // ���Ʒ���

int main104(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	// ���Ʒ���
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_tree(new pcl::search::KdTree<pcl::PointXYZ>);


	ne.setSearchMethod(normal_tree);
	ne.setInputCloud(cloud);
	ne.setRadiusSearch(.03);
	ne.compute(*normals);

	// ���� PFH ���ƶ���
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;

	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);

	//�������������ΪPointNormal,��ִ��pfh.setInputNormals (cloud);

	// ����kdtree �� pfh
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pfh.setSearchMethod(tree);

	// ���
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	//ʹ�ð뾶��5���׷�Χ�ڵ�������Ԫ�ء�

	//ע�⣺�˴�ʹ�õİ뾶����Ҫ���ڹ��Ʊ��淨��ʱʹ�õİ뾶!!!
	
	pfh.setRadiusSearch(0.05);

	pfh.compute(*pfhs);

	// ���ӻ�
	pfh.compute(*pfhs);

	// ֱ��ͼ���ӻ�
	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*pfhs, 300);
	plotter.plot();


	return 0;
}