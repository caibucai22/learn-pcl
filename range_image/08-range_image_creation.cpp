 
#include<pcl/range_image/range_image.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/range_image_visualizer.h>

int main081(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr;

	pcl::io::loadPCDFile("bunny.pcd", *cloud_ptr);
	 
	// angularResolution ģ������ȴ������Ƕȷֱ���
	float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // ���� 1
	// maxAngleWidth Ϊģ�����ȴ�����ˮƽ��������
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // ���� 360
	// maxAngleHeight ģ�⴫�����Ĵ�ֱ�����������Ƕ� ��תΪ ����
	float maxAngleHeight = (float)(180.0f * (M_PI) / 180.0f); // ���� 180
	
	// �������Ĳɼ�λ��
	Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	// ���ͼ����ѭ����ϵͳ
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

	float noise_level = 0.00;
	float min_range = 0.0f;
	int border_size = 1;

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& rangeImage = *range_image_ptr;

	rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	/*
	* range_image.createFromPointCloud���������Ľ��� ���漰�ĽǶȶ�Ϊ����Ϊ��λ�� ��
    point_cloudΪ�������ͼ������Ҫ�ĵ���
    angular_resolution_x��ȴ�����X����ĽǶȷֱ���
    angular_resolution_y��ȴ�����Y����ĽǶȷֱ���
    pcl::deg2rad (360.0f)��ȴ�������ˮƽ�������Ƕ�
    pcl::deg2rad (180.0f)��ֱ�������Ƕ�
    scene_sensor_pose���õ�ģ�⴫������λ����һ������任����Ĭ��Ϊ4*4�ĵ�λ����任
    coordinate_frame���尴����������ϵͳ��ϰ��  Ĭ��ΪCAMERA_FRAME
    noise_level  ��ȡ���ͼ�����ʱ���ڽ���Բ�ѯ�����ֵ��Ӱ��ˮƽ
    min_range ������С�Ļ�ȡ���룬С����С�Ļ�ȡ�����λ��Ϊ��������ä��
    border_size  ���û�ȡ���ͼ���Ե�Ŀ�� Ĭ��Ϊ0
	*/
	std::cout << rangeImage << "\n";

	pcl::visualization::PCLVisualizer viewer("3d viewer");
	viewer.setBackgroundColor(1, 1, 1);

	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");*/

	// ���ԭʼ����
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_image_color_handler(cloud_ptr, 255, 100, 0);
	viewer.addPointCloud(cloud_ptr, org_image_color_handler, "original image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original image");

	viewer.initCameraParameters();
	viewer.addCoordinateSystem();


	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}


	return 0;
}