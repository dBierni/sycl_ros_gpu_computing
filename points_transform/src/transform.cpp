#include <rclcpp/rclcpp.hpp>
#include "points_transform/transform.hpp"
#include <chrono>
#include <iostream>

void TransformServer::transform_points_no_gpu(sensor_msgs::msg::PointCloud2 &cloud_ros, const Eigen::Matrix4d &trans,
                             sensor_msgs::msg::PointCloud2 &cloud_out) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCLPointCloud2Ptr cloud2(new pcl::PCLPointCloud2);
  pcl::moveFromROSMsg<pcl::PointXYZ>(cloud_ros, *cloud);

  for(std::size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = trans.data()[0] * cloud->points[i].x + trans.data()[4] * cloud->points[i].y
      +trans.data()[8]*cloud->points[i].z+trans.data()[12];
    cloud->points[i].y = trans.data()[1]*cloud->points[i].x+trans.data()[5]*cloud->points[i].y
      +trans.data()[9]*cloud->points[i].z+trans.data()[13];
    cloud->points[i].z = trans.data()[2]*cloud->points[i].x+trans.data()[6]*cloud->points[i].y
      +trans.data()[10]*cloud->points[i].z+trans.data()[14];
  }
  pcl::toROSMsg<pcl::PointXYZ>(*cloud, cloud_out);

}
void TransformServer::server_function(const std::shared_ptr<internal_msgs::srv::PointCloud::Request> request,
																			std::shared_ptr<internal_msgs::srv::PointCloud::Response> response) {
              request->cloud.data.size());

  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block<3,3>(0,0) =  Eigen::Quaterniond(request->transformation.rotation.w,
                                              request->transformation.rotation.x,
                                              request->transformation.rotation.y,
                                              request->transformation.rotation.z).toRotationMatrix();

  trans.block<3,1>(0,3) = Eigen::Vector3d(request->transformation.translation.x,
                                          request->transformation.translation.y,
                                          request->transformation.translation.z);

  auto start = std::chrono::high_resolution_clock::now();
  if(request->use_gpu.data)
    sycl_.transfrom_pointCloud2(request->cloud, trans, response->cloud);
  else
    transform_points_no_gpu(request->cloud, trans, response->cloud);
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
  RCLCPP_INFO(this->get_logger(), "Point Cloud size: %i, transformed in %i[ms] ", this->get_name(),duration.count());
  response->time.data = duration.count();
};
