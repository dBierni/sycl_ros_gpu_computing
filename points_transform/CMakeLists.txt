cmake_minimum_required(VERSION 3.5)
project(points_transform)

set(BUILD_CLIENT_TEST_NODE true)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(internal_msgs)
find_package(PCL)
find_package(Eigen3)


# All stuff related to SYCL
add_library(sycl_internal STATIC src/sycl_internal.cpp)
target_compile_options(sycl_internal PUBLIC -fsycl -fsycl-targets=nvptx64-nvidia-cuda-sycldevice)
target_link_libraries(sycl_internal sycl Eigen3::Eigen ${PCL_LIBRARIES} )
target_link_options(sycl_internal PUBLIC -fsycl -fsycl-targets=nvptx64-nvidia-cuda-sycldevice)
ament_target_dependencies(sycl_internal sensor_msgs)
target_include_directories(sycl_internal
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ROS2 Server
add_library(transformServer STATIC src/transform.cpp)
target_link_libraries(transformServer sycl_internal )
ament_target_dependencies(transformServer rclcpp sensor_msgs internal_msgs)
target_include_directories(transformServer PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Node
add_executable(pointTransformNode src/transform_node.cpp)
target_link_libraries(pointTransformNode transformServer sycl_internal )
#ament_target_dependencies(pointTransformNode rclcpp  sensor_msgs internal_msgs geometry_msgs)

set(INSTALL_TARGETS  ${pointTransformNode} ${sycl_internal} ${transformServer})

# Test ROS2 client
if(BUILD_CLIENT_TEST_NODE)
add_executable(pointTransformClient src/transform_client_test.cpp)
target_link_libraries(pointTransformClient ${PCL_LIBRARIES})
ament_target_dependencies(pointTransformClient rclcpp std_msgs sensor_msgs internal_msgs)
list(APPEND INSTALL_TARGETS ${pointTransformClient})
endif()

install(TARGETS
        ${INSTALL_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
