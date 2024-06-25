//
//#include<pcl/point_types.h>
//#include<pcl/point_cloud.h>
//#include<pcl/io/openni2_grabber.h>
//#include<pcl/visualization/cloud_viewer.h>
//
//#include<pcl/compression/octree_pointcloud_compression.h>
//
//#include<iostream>
//#include<stdio.h>
//#include<sstream>
//#include<stdlib.h>
//
//#ifdef WIN32
//#define sleep(x) Sleep((x)*1000)
//#endif
//
//class SimpleOpenNIViewer
//{
//public:
//	pcl::visualization::CloudViewer viewer;
//	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
//	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
//
//	SimpleOpenNIViewer() :viewer("point cloud compression example")
//	{
//
//	}
//
//	// 回调函数
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
//	{
//		if (!viewer.wasStopped())
//		{
//			// 存储压缩点云的字节流对象
//			std::stringstream compressData;
//			// 存储输出点云
//			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
//
//			// 压缩点云
//			PointCloudEncoder->encodePointCloud(cloud, compressData);
//
//			// 解压缩点云
//			PointCloudDecoder->decodePointCloud(compressData, cloudOut);
//
//			// 可视化解压缩的点云
//			viewer.showCloud(cloudOut);
//		}
//	}
//	void run()
//	{
//		bool showStatistics = true;
//		// 压缩选项 详情在 /io/include/pcl/compression/compression_profiles.h
//		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
//
//		// 初始化压缩和解压缩对象
//		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
//		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
//		//创建从openNI 获取点云的抓取对象
//		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
//
//		// 建立回调函数
//		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f 
//			= boost::bind(&SimpleOpenNIViewer::cloud_cb_,this,_1);
//
//		// 建立回调函数和回调信息绑定
//		boost::signals2::connection c = interface->registerCallback(f);
//
//		// 开始接收点云的数据
//		interface->start();
//
//		while (! viewer.wasStopped())
//		{
//			sleep(1);
//		}
//		interface->stop();
//
//		// 删除
//		delete (PointCloudEncoder);
//		delete (PointCloudDecoder);
//	}
//};
//
//int main031(int argc, char** argv)
//{
//	SimpleOpenNIViewer v;
//	v.run();
//}