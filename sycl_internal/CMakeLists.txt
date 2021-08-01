cmake_minimum_required(VERSION 3.5)
project(sycl_internal)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(internal_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# All stuff related to SYCL
add_library(${PROJECT_NAME} SHARED src/sycl_internal.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE "AMENT_INDEX_CPP_BUILDING_DLL")
target_compile_options(${PROJECT_NAME} PUBLIC -fsycl -fsycl-targets=nvptx64-nvidia-cuda-sycldevice)
target_link_libraries(${PROJECT_NAME} sycl Eigen3::Eigen ${PCL_LIBRARIES} )
target_link_options(${PROJECT_NAME} PUBLIC -fsycl -fsycl-targets=nvptx64-nvidia-cuda-sycldevice)
ament_target_dependencies(${PROJECT_NAME} sensor_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

#ament_export_interfaces(export_sycl_internal HAS_LIBRARY_TARGET)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL Eigen3 internal_msgs geometry_msgs rclcpp)

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
#        DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        )


ament_export_dependencies(rosidl_default_runtime)
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

#ament_export_libraries(${PROJECT_NAME})
ament_package()
