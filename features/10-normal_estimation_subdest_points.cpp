#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d.h>

int main102()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);

	std::vector<int> indices(std::floor(cloud->size() / 10));
	for (size_t i = 0; i < indices.size(); i++)
	{
		indices[i] = i;
	}

	// ���� ne
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	//��������
	pcl::shared_ptr<std::vector<int>> indice_ptr(new std::vector<int>(indices)); 
	ne.setIndices(indice_ptr);

	// ���� tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// output
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// �ڰ뾶3cm ��Χ�ڼ��� ����
	ne.setRadiusSearch(.03);

	ne.compute(*cloud_normals);

	return 0;
}