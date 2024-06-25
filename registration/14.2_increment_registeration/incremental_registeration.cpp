#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2; // 左右视点 ID

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name; // 文件名称

  PCD() : cloud(new PointCloud){};
};

struct PCDComparator
{
  bool operator()(const PCD &p1, const PCD &p2)
  {
    return (p1.f_name < p2.f_name);
  }
};

class MyPointRrepresentation : public pcl::PointRepresentation<PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
  MyPointRrepresentation()
  {
    nr_dimensions_ = 4; // 点的维度
  }
  // 重载 copyToFloatArray 方法将点转化为四维数组
  virtual void copyToFloatArray(const PointNormalT &p, float *out) const
  {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void showCloudLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud("vp1_target");
  p->removePointCloud("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h(cloud_target, 0, 255, 0);
  p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO("Press q to begin the regression");
  p->spin();
}

void showCloudRight(const PointCloudWithNormals::Ptr cloud_target,
                    const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud("source");
  p->removePointCloud("target");

  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
  if (!tgt_color_handler.isCapable())
  {
    PCL_WARN("cannot create curvature color handler");
  }

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
  if (!src_color_handler.isCapable())
  {
    PCL_WARN("cannot create curvature color handler");
  }

  p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
{
  std::string extension(".pcd");
  // 第一个参数是命令本身，从第二个参数开始解析
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string(argv[i]);
    if (fname.size() < extension.size())
      continue;
    std::transform(fname.begin(), fname.end(), fname.begin(), (int (*)(int))tolower);
    // 检查参数是否为一个pcd 后缀的文件
    if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
    {
      // 加载点云并保存在总体的点云列表中
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile(argv[i], *m.cloud);
      // 从点云中移除NAN点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

      models.push_back(m);
    }
  }
}

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  PointCloud::Ptr src(new PointCloud);
  PointCloud::Ptr tgt(new PointCloud);

  pcl::VoxelGrid<PointT> grid;

  if (downsample)
  {
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setInputCloud(cloud_src);
    grid.filter(*src);

    grid.setInputCloud(cloud_tgt);
    grid.filter(*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // 计算表面法向量和曲率
  PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est; // 点云法线估计对象
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);

  MyPointRrepresentation point_representation;
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  // 配准
  pcl::IterativeClosestPoint<PointNormalT, PointNormalT> reg; //
  reg.setTransformationEpsilon(1e-6);                         //
  // 设置点表示
  reg.setPointRepresentation(std::make_shared<const MyPointRrepresentation>(point_representation)); // boost:: -> std

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  //
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targeetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

  reg.setMaximumIterations(2);

  for (int i = 0; i < 30; i++)
  {
    PCL_INFO("Iteration Nr. %d.\n", i);

    points_with_normals_src = reg_result;

    // estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each iter
    Ti = reg.getFinalTransformation() * Ti;

    if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();

    showCloudRight(points_with_normals_tgt, points_with_normals_src);
  }

  targeetToSource = Ti.inverse(); // deidao

  pcl::transformPointCloud(*cloud_tgt, *output, targeetToSource);

  p->removePointCloud("source");
  p->removePointCloud("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
  p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

  PCL_INFO("Press q to continue registeration. \n");
  p->spin();

  p->removePointCloud("source");
  p->removePointCloud("target");

  *output += *cloud_src;

  final_transform = targeetToSource;
}

int main(int argc, char **argv)
{
  std::cout << "hello PCL world!" << std::endl;

  std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
  loadData(argc, argv, data);

  // 检查输入
  if (data.empty())
  {
    PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR("[*] - multiple files can be added. The registeration results of (i,i+1) will be registered against (i+2), etc");
    return EXIT_FAILURE;
  }

  PCL_INFO("Loaded %d datasets.", (int)data.size());

  // 创建PCL可视化对象
  p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise incremental registeration example");
  p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

  PointCloud::Ptr result(new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

  for (size_t i = 0; i < data.size(); i++)
  {
    source = data[i - 1].cloud;
    target = data[i].cloud;

    showCloudLeft(source, target);

    PointCloud::Ptr temp(new PointCloud);
    PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(),
             data[i].f_name.c_str(), target->points.size());

    pairAlign(source, target, temp, pairTransform, true); // pairTransform 返回从目标点云target 到 source 的变换矩阵

    // 把当前两两配准的点云temp 转化到全局坐标系下 返回result
    pcl::transformPointCloud(*temp, *result, GlobalTransform);

    // 用当前的两组点云之间的变换更新全局变换
    GlobalTransform = GlobalTransform * pairTransform;
    // 保存转换到第一个点云坐标下的当前配准后的两组点云result 到文件i.pcd
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile(ss.str(), *result, true);
  }

  return 0;
}