#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
																			PointCloudT &adjacent_supervoxel_centers,
																			std::string supervoxel_name,
																			pcl::visualization::PCLVisualizer::Ptr &viewer);

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		pcl::console::print_error("Syntax is: %s <pcd-file> \n "
															"--NT Dsables the single cloud transform \n"
															"-v <voxel resolution>\n-s <seed resolution>\n"
															"-c <color weight> \n-z <spatial weight> \n"
															"-n <normal_weight>\n",
															argv[0]);
		return (1);
	}

	PointCloudT::Ptr cloud(new PointCloudT);
	// 解析参数
	pcl::console::print_highlight("Loading point cloud...\n");
	if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}

	bool disable_transform = pcl::console::find_switch(argc, argv, "--NT");

	float voxel_resolution = 0.008f;
	bool voxel_res_specified = pcl::console::find_switch(argc, argv, "-v");
	if (voxel_res_specified)
		pcl::console::parse(argc, argv, "-v", voxel_resolution);

	float seed_resolution = 0.1f;
	bool seed_res_specified = pcl::console::find_switch(argc, argv, "-s");
	if (seed_res_specified)
		pcl::console::parse(argc, argv, "-s", seed_resolution);

	float color_importance = 0.2f;
	if (pcl::console::find_switch(argc, argv, "-c"))
		pcl::console::parse(argc, argv, "-c", color_importance);

	float spatial_importance = 0.4f;
	if (pcl::console::find_switch(argc, argv, "-z"))
		pcl::console::parse(argc, argv, "-z", spatial_importance);

	float normal_importance = 1.0f;
	if (pcl::console::find_switch(argc, argv, "-n"))
		pcl::console::parse(argc, argv, "-n", normal_importance);

	//////////////////////////////  //////////////////////////////
	////// This is how to use supervoxels
	//////////////////////////////  //////////////////////////////

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	if (disable_transform)
		super.setUseSingleCameraTransform(false);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);

	// 存储超体素
	std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

	pcl::console::print_highlight("Extracting supervoxels!\n");
	super.extract(supervoxel_clusters);
	pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
	viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

	PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
	viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

	PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
	// We have this disabled so graph is easy to see, uncomment to see supervoxel normals
	// viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

	pcl::console::print_highlight("Getting supervoxel adjacency\n");
	// 构造邻接图
	std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	// To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
	for (auto label_itr = supervoxel_adjacency.cbegin(); label_itr != supervoxel_adjacency.cend();)
	{
		// 获取标签
		std::uint32_t supervoxel_label = label_itr->first;
		// 根据标签索引得到超体素
		pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

		
		// 遍历该超体素相邻超体素并以其相邻超体素中心为点集构造点云，用于后续可视化，这里的相邻超体素在多重映射容器中具有相同的键值
		PointCloudT adjacent_supervoxel_centers;
		for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
		{
			pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
			adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
		}
		// Now we make a name for this polygon
		std::stringstream ss;
		ss << "supervoxel_" << supervoxel_label;
		// This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
		// 绘制超体素与其相邻超体素
		addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
		// Move iterator forward to next label
		label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	return (0);
}

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
																			PointCloudT &adjacent_supervoxel_centers,
																			std::string supervoxel_name,
																			pcl::visualization::PCLVisualizer::Ptr &viewer)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	// Iterate through all adjacent points, and add a center point to adjacent point pair
	for (auto adjacent_itr = adjacent_supervoxel_centers.begin(); adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
	{
		points->InsertNextPoint(supervoxel_center.data);
		points->InsertNextPoint(adjacent_itr->data);
	}
	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// Add the points to the dataset
	polyData->SetPoints(points);
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);
	cells->InsertNextCell(polyLine);
	// Add the lines to the dataset
	polyData->SetLines(cells);
	viewer->addModelFromPolyData(polyData, supervoxel_name);
}