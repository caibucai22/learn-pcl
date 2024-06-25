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
//	// �ص�����
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
//	{
//		if (!viewer.wasStopped())
//		{
//			// �洢ѹ�����Ƶ��ֽ�������
//			std::stringstream compressData;
//			// �洢�������
//			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
//
//			// ѹ������
//			PointCloudEncoder->encodePointCloud(cloud, compressData);
//
//			// ��ѹ������
//			PointCloudDecoder->decodePointCloud(compressData, cloudOut);
//
//			// ���ӻ���ѹ���ĵ���
//			viewer.showCloud(cloudOut);
//		}
//	}
//	void run()
//	{
//		bool showStatistics = true;
//		// ѹ��ѡ�� ������ /io/include/pcl/compression/compression_profiles.h
//		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
//
//		// ��ʼ��ѹ���ͽ�ѹ������
//		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
//		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
//		//������openNI ��ȡ���Ƶ�ץȡ����
//		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
//
//		// �����ص�����
//		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f 
//			= boost::bind(&SimpleOpenNIViewer::cloud_cb_,this,_1);
//
//		// �����ص������ͻص���Ϣ��
//		boost::signals2::connection c = interface->registerCallback(f);
//
//		// ��ʼ���յ��Ƶ�����
//		interface->start();
//
//		while (! viewer.wasStopped())
//		{
//			sleep(1);
//		}
//		interface->stop();
//
//		// ɾ��
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