#include "points_transform/transform.hpp"



int main(int argc, char * argv[]) {
	sycl::ros::CUDASelector gpu_cuda_select;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TransformServer>("points_transform", gpu_cuda_select));
	rclcpp::shutdown();
	return 0;
}
