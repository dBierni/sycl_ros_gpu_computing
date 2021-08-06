#include "rclcpp/rclcpp.hpp"
#include <internal_msgs/srv/point_cloud.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <functional>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/db/Robotics/ROSWORKSPACES/ros2_ws/src/sycl_ros_gpu_computing/points_transform/data/scan000.pcd", *cloud) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
    }


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("transform_client");
  RCLCPP_INFO(node->get_logger(), "START Node %s", node->get_name());

  rclcpp::Client<internal_msgs::srv::PointCloud>::SharedPtr client =
          node->create_client<internal_msgs::srv::PointCloud>("points_transform/transform");

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher =
          node->create_publisher<sensor_msgs::msg::PointCloud2>("Cloud2", 10);

  auto request = std::make_shared<internal_msgs::srv::PointCloud::Request>();
  pcl::toROSMsg<pcl::PointXYZ>(*cloud, request->cloud);

  tf2::Quaternion diffQuaternion;
  diffQuaternion.setRPY( 0, 0, 1.0f*M_PI/180.0 );  // Create this quaternion from roll/pitch/yaw (in radians)
  request->transformation.rotation.x = diffQuaternion.x();
  request->transformation.rotation.y = diffQuaternion.y();
  request->transformation.rotation.z = diffQuaternion.z();
  request->transformation.rotation.w = diffQuaternion.w();
  std::vector<int> time_res;
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  float angle = 0;
  float angleRadsign = 1.0;
  request->use_gpu.data = true;
  while(rclcpp::ok()) {
    auto result = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
      request->cloud = result.get()->cloud;
      result.get()->cloud.header.frame_id = "/base_link";
      result.get()->cloud.header.stamp = rclcpp::Clock().now();
      time_res.push_back(result.get()->time.data);
      angle+=1;
      publisher->publish(result.get()->cloud);

    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service add_two_ints");
    }


//    if(angle >= 359){
//      angleRadsign*=-1.0;
//      diffQuaternion.setRPY( 0, 0, angleRadsign*1.0f*M_PI/180.0 );
//      angle = 0;
//      int average_time = std::accumulate(time_res.begin(), time_res.end(), 0) / time_res.size();
//      std::string name_type = "no_gpu";
//      if(request->use_gpu.data)
//        name_type = "sycl_gpu";
//
//      RCLCPP_INFO(node->get_logger(), "Avergae time(samples: %i)by function(%s):%i microseconds. For cloud size:%i ",
//                  time_res.size(),name_type.c_str(), average_time, result.get()->cloud.data.size());
//      request->use_gpu.data = !request->use_gpu.data;
//      time_res.clear();
//    }


    std::this_thread::sleep_for(1ms);
  }
  rclcpp::shutdown();
  return 0;
}