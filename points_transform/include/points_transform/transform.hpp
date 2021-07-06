#include <points_transform/sycl_internal.h>
#include <internal_msgs/srv/point_cloud.hpp>
#include <points_transform/sycl_internal.h>

class TransformServer : public rclcpp::Node {
public:
		TransformServer(std::string name, const sycl::device_selector &device_selector) : Node(name), sycl_{device_selector} {
			srv_ = this->create_service<internal_msgs::srv::PointCloud>(name+"/transform",
																																	std::bind(&TransformServer::server_function,
																																						this, std::placeholders::_1,
																																						std::placeholders::_2));
		};
private:

		rclcpp::Service<internal_msgs::srv::PointCloud>::SharedPtr srv_;
		sycl::ros::Interface sycl_;

		void server_function( const std::shared_ptr<internal_msgs::srv::PointCloud::Request> request,
													std::shared_ptr<internal_msgs::srv::PointCloud::Response> response);
};
