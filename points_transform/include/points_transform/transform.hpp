#include <sycl_internal/sycl_internal.hpp>
#include <internal_msgs/srv/point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>


class TransformServer : public rclcpp::Node {
public:
		TransformServer(std::string name, const sycl::device_selector &device_selector) : Node(name), sycl_{device_selector} {
			srv_ = this->create_service<internal_msgs::srv::PointCloud>(name+"/transform",
																																	std::bind(&TransformServer::server_function,
																																						this, std::placeholders::_1,
																																						std::placeholders::_2));
			RCLCPP_INFO(this->get_logger(), "START Node %s", this->get_name());
		};

private:
		rclcpp::Service<internal_msgs::srv::PointCloud>::SharedPtr srv_;
		sycl::ros::Interface sycl_;

		void server_function( const std::shared_ptr<internal_msgs::srv::PointCloud::Request> request,
													std::shared_ptr<internal_msgs::srv::PointCloud::Response> response);

		void transform_points_no_gpu(sensor_msgs::msg::PointCloud2 &cloud_ros, const Eigen::Matrix4d &trans,
                                 sensor_msgs::msg::PointCloud2 &cloud_out);
};
