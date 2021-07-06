//
// Created by db on 22.06.2021.
//

#ifndef ROS2_MASTER_SYCL_INTERNAL_H
#define ROS2_MASTER_SYCL_INTERNAL_H

#include <CL/sycl.hpp>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

namespace sycl { namespace ros {
		class PointParallel;

		class CUDASelector : public sycl::device_selector {
		public:
				int operator()(const cl::sycl::device &Device) const override {
					const std::string DriverVersion = Device.get_info<sycl::info::device::driver_version>();
					if (Device.is_gpu() && (DriverVersion.find("CUDA") != std::string::npos)) {
						return 1;
					};
					return -1;
				}
		};

class Interface  {
public:
		/// Costructor for Interface class
		/// \param device_selector Specify place of calculation execution - gpu or cpu
		Interface(const sycl::device_selector &device_selector):
		q_(device_selector){};

		///
		/// \param cloud_ros Input pointCloud2
		/// \param trans Transformation used to transform pointCloud2
		/// \param cloud_out Result cloud
		void transfrom_pointCloud2(sensor_msgs::msg::PointCloud2 &cloud_ros, const Eigen::Matrix4d &trans,
															sensor_msgs::msg::PointCloud2 &cloud_out);
private:
		///
		sycl::queue q_;

		/// Function transforms points
		/// \tparam T Type of the point. Should contain of .x, .y, .z
		/// \tparam allocator Allocator for \p T
		/// \param points Vectors of points to transform
		/// \param trans Transformation matrix used to transform \p points
		template<typename T, typename allocator = std::allocator<T>>
		void transform_points(std::vector<T,allocator> &points, const Eigen::Matrix4d &trans);
};




}}
#endif //ROS2_MASTER_SYCL_INTERNAL_H
