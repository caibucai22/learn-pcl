//#include<pcl/point_cloud.h>
//#include<pcl/point_types.h>
//#include<pcl/io/openni2_grabber.h>
//#include<pcl/common/time.h>
//#include<thread>
//#include<chrono>
//
////��SimpleOpenNIProcessor  �Ļص���������Ϊ�ڻ�ȡ����ʱ�������ݽ��д���Ļص������ķ�װ��
////�ڱ����в�û��ʲô����ֻ��ʵʱ���ڱ�׼����豸��ӡ����Ϣ��
//
//class SimpleOpenNIProcessor
//{
//public:
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
//	{
//		static unsigned count = 0;
//		static double last = pcl::getTime();
//		if (++count == 30)
//		{
//			double now = pcl::getTime();
//			std::cout << "distance of center pixel : " << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z
//				<< " mm. Average framerate: " << double(count) / double(now - last) << "HZ" << std::endl;
//			count = 0;
//			last = now;
//		}
//	}
//	void run()
//	{
//		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(); // ����openni �ɼ�����
//
//		// ����ص�����
//		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
//			boost::bind(&SimpleOpenNIProcessor::cloud_cb_, this);
//		boost::signals2::connection c = interface->registerCallback(f);
//
//		interface->start();
//		while (true)
//		{
//			std::this_thread::sleep_for(std::chrono::microseconds(100000));
//		}
//		interface->stop();
//	}
//};
//
//int main004()
//{
//	SimpleOpenNIProcessor v;
//	v.run();
//	return 0;
//}