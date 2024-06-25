//#include<pcl/point_cloud.h>
//#include<pcl/point_types.h>
//#include<pcl/io/openni2_grabber.h>
//#include<pcl/common/time.h>
//#include<thread>
//#include<chrono>
//
////类SimpleOpenNIProcessor  的回调函数，作为在获取数据时，对数据进行处理的回调函数的封装，
////在本例中并没有什么处理，只是实时的在标准输出设备打印处信息。
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
//		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(); // 创建openni 采集对象
//
//		// 定义回调函数
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