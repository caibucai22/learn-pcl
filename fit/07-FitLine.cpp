#include<pcl/io/pcd_io.h>
#include<pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_line.h>
#include<pcl/filters/project_inliers.h>

using namespace std;

int main74()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("data//L.pcd", *cloud) < 0)
	{
		PCL_ERROR("点云文件不存在");
		return -1;
	}
	cout << "加载点云个数为：" << cloud->points.size() << endl;

	//-------------空间直线拟合--------------------
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr modelLine
	(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelLine);
	ransac.setDistanceThreshold(0.01);
	ransac.setMaxIterations(1000);
	ransac.computeModel();

	vector<int> inliers;
	ransac.getInliers(inliers);

	Eigen::VectorXf lineCoefs; // 模型参数
	ransac.getModelCoefficients(lineCoefs);

	// ----------------------投影到拟合直线-----------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr lineProCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(6);

	coefficients->values[0] = lineCoefs[0];
	coefficients->values[1] = lineCoefs[1];
	coefficients->values[2] = lineCoefs[2];
	coefficients->values[3] = lineCoefs[3];
	coefficients->values[4] = lineCoefs[4];
	coefficients->values[5] = lineCoefs[5];

	// ----------------------创建滤波器对象------------------
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_LINE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*lineProCloud);
	pcl::io::savePCDFileBinary("lineProCloud.pcd", *lineProCloud);


	return 0;
}