#include <points_transform/sycl_internal.h>

template void sycl::ros::Interface::transform_points<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>
(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &points, const Eigen::Matrix4d &trans);

template<typename T, typename allocator>
void sycl::ros::Interface::transform_points(std::vector<T,allocator> &points, const Eigen::Matrix4d &trans) {

		const std::size_t points_size = points.size();
		const std::size_t trans_size = trans.size();

		auto device_transPtr = sycl::malloc_device<double>(trans_size, q_);
		auto device_pointsPtr = sycl::malloc_device<T>(points_size, q_);

		q_.memcpy(device_transPtr, &trans.data()[0], trans_size * sizeof(double)).wait();
		q_.memcpy(device_pointsPtr, &points.data()[0], points_size * sizeof(T)).wait();

		q_.parallel_for<PointParallel>(sycl::range{points_size}, [=](sycl::id<1> idx) {
			device_pointsPtr[idx].x = device_transPtr[0]*device_pointsPtr[idx].x+device_transPtr[4]*device_pointsPtr[idx].y
							+device_transPtr[8]*device_pointsPtr[idx].z+device_transPtr[12];
			device_pointsPtr[idx].y = device_transPtr[1]*device_pointsPtr[idx].x+device_transPtr[5]*device_pointsPtr[idx].y
							+device_transPtr[9]*device_pointsPtr[idx].z+device_transPtr[13];
			device_pointsPtr[idx].z = device_transPtr[2]*device_pointsPtr[idx].x+device_transPtr[6]*device_pointsPtr[idx].y
							+device_transPtr[10]*device_pointsPtr[idx].z+device_transPtr[14];

		}).wait();

		q_.memcpy(&points.data()[0], device_pointsPtr, points_size * sizeof(T)).wait();
		free(device_pointsPtr, q_);
		free(device_transPtr, q_);

};

void sycl::ros::Interface::transfrom_pointCloud2(sensor_msgs::msg::PointCloud2 &cloud_ros, const Eigen::Matrix4d &trans,
																								sensor_msgs::msg::PointCloud2 &cloud_out) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCLPointCloud2Ptr cloud2(new pcl::PCLPointCloud2);
	pcl::moveFromROSMsg<pcl::PointXYZ>(cloud_ros, *cloud);
	transform_points<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>(cloud->points,trans);
	pcl::toROSMsg<pcl::PointXYZ>(*cloud, cloud_out);

}
