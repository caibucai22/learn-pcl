#include<pcl/ModelCoefficients.h>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/features/normal_3d.h>
#include<pcl/kdtree/kdtree.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_clusters.h>

int main122(int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("table_scene_lms400.pcd", *cloud);
	std::cout << "pointcloud before filtering has" << cloud->points.size() << " data points." << std::endl;

	// creating the filtering object downsampling the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg; // �ڵ����ϴ��� 3D�������� ������������Ϊһ��ռ��е�΢С3D��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered); // �˲�

	std::cout << "point cloud after filtering has: " << cloud->points.size() << " data points." << std::endl;

	// ����ƽ��ģ�ͷָ���� �����ò���
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE); //�ָ�ģ��
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02); // ������ֵ

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points) // �˲�ֹͣ���� ������ 30%
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "could not estimate a plannar model for the given dataset" << std::endl;
			break;
		}
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "point cloud representing the planar component: " << cloud_plane->points.size() << " data points."
			<< std::endl;

		// ��ȡʣ�����
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;


	}

	// 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); //���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(100); // ���þ�����������ٵ����Ŀ
	ec.setMaxClusterSize(25000); // ���þ�����Ҫ������Ŀ
	ec.setSearchMethod(tree); // ���õ��Ƶ���������
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices); //  �ӵ�������ȡ���࣬����������������

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;	
		}
		std::cout << "point cloud representing the cluster: " << cloud_cluster->points.size() << " data points.\n";
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
		j++;
	}
	return 0;
	

}