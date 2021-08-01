#include <rclcpp/rclcpp.hpp>
#include "points_transform/transform.hpp"


void TransformServer::server_function(const std::shared_ptr<internal_msgs::srv::PointCloud::Request> request,
																			std::shared_ptr<internal_msgs::srv::PointCloud::Response> response) {

  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block<3,3>(0,0) =  Eigen::Quaterniond(request->transformation.rotation.w,
                                              request->transformation.rotation.x,
                                              request->transformation.rotation.y,
                                              request->transformation.rotation.z).toRotationMatrix();

  trans.block<3,1>(0,3) = Eigen::Vector3d(request->transformation.translation.x,
                                          request->transformation.translation.y,
                                          request->transformation.translation.z);
  sycl_.transfrom_pointCloud2(request->cloud, trans, response->cloud);
};
