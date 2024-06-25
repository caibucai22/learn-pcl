#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/features/integral_image_normal.h>
#include<pcl/visualization/cloud_viewer.h>

int main103()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);

	// 创建法线估计向量
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	/****************************************************************************************
		三种法线估计方法
		 COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
		AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
		AVERAGE_DEPTH——CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
		********************************************************************************************/

	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);

	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);

	// 可视化
	pcl::visualization::PCLVisualizer viewer("pcl viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	

	return 0;
}