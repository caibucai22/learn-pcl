#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionToViewer(PointT &supervoxel_center,
                                     PointCloudT &adjacent_supervoxel_centers,
                                     std::string supervoxel_name,
                                     pcl::visualization::PCLVisualizer::Ptr &viewer);

int main(int argc, char **argv)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>("desk.pcd", *cloud))
    {
        pcl::console::print_error("load pcd file failed.");
        return -1;
    }

    // 构造超体素
    bool disable_transform = pcl::console::find_switch(argc, argv, "--NT");
    float voxel_resolution = 0.008f;
    bool voxel_res_specified = pcl::console::find_switch(argc, argv, "-v");
    if (voxel_res_specified)
    {
        pcl::console::parse(argc, argv, "-v", voxel_resolution);
    }

    float seed_resolution = 0.1f;
    bool seed_res_specified = pcl::console::find_switch(argc, argv, "-s");
    if (seed_res_specified)
        pcl::console::parse(argc, argv, "-s", seed_resolution);

    float color_importance = 0.2f;
    if (pcl::console::find_switch(argc, argv, "-c"))
        pcl::console::parse(argc, argv, "-c", color_importance);
    float spatial_importance = 1.0f;
    if (pcl::console::find_switch(argc, argv, "-z"))
        pcl::console::parse(argc, argv, "-z", spatial_importance);
    float normal_importance = 1.0f;
    if (pcl::console::find_switch(argc, argv, "-n"))
        pcl::console::parse(argc, argv, "-n", normal_importance);

    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    if (disable_transform)
        super.setUseSingleCameraTransform(false);
    super.setInputCloud(cloud);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    pcl::console::print_highlight("Extracting supervoxel");
    super.extract(supervoxel_clusters);
    pcl::console::print_info("foud %n supervoxels\n", supervoxel_clusters.size());

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
    viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

    PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
    viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

    PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
    // viewer->addPointCloudNormals<pcl::PointNormal>(sv_normal_cloud,1,0.05f,"supervoxel_normals");

    pcl::console::print_highlight("getting supervoxel adjacency\n");
    std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacenecy;
    super.getSupervoxelAdjacency(supervoxel_adjacenecy);

    for (auto label_itr = supervoxel_adjacenecy.cbegin(); label_itr != supervoxel_adjacenecy.cend();)
    {
        std::uint32_t supervoxel_label = label_itr->first;
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

        PointCloudT adjacent_supervoxel_centers;
        for (auto adjacent_itr = supervoxel_adjacenecy.equal_range(supervoxel_label).first;
             adjacent_itr != supervoxel_adjacenecy.equal_range(supervoxel_label).second; adjacent_itr++)
        {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
            adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
        }
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        addSupervoxelConnectionToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
        label_itr = supervoxel_adjacenecy.upper_bound(supervoxel_label);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}

void addSupervoxelConnectionToViewer(PointT &supervoxel_center,
                                     PointCloudT &adjacent_supervoxel_centers,
                                     std::string supervoxel_name,
                                     pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyline = vtkSmartPointer<vtkPolyLine>::New();

    for (auto adjacent_itr = adjacent_supervoxel_centers.begin();
         adjacent_itr != adjacent_supervoxel_centers.end(); adjacent_itr++)
    {
        points->InsertNextPoint(supervoxel_center.data);
        points->InsertNextPoint(adjacent_itr->data);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyline->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
    for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
    {
        polyline->GetPointIds()->SetId(i, i);
    }
    cells->InsertNextCell(polyline);
    polyData->SetLines(cells);
    viewer->addModelFromPolyData(polyData, supervoxel_name);
}