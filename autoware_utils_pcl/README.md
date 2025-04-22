# autoware_utils_pcl

## Overview

The **autoware_utils** library is a comprehensive toolkit designed to facilitate the development of autonomous driving applications.
This package provides essential utilities for point cloud.
It is extensively used in the Autoware project to handle common tasks such as point cloud transformations.

## Design

- **`pcl_conversion.hpp`**: Efficient conversion and transformation of PointCloud2 messages to PCL point clouds.
- **`transforms.hpp`**: Efficient methods for transforming and manipulating point clouds.

## Example Code Snippets

### Efficient Point Cloud Conversion with pcl_conversion.hpp

```cpp
#include <autoware_utils_pcl/pcl_conversion.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pcl_conversion_node");

  // Load point cloud data
  sensor_msgs::msg::PointCloud2 cloud_in; // Assume this is populated with data
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  // Define transformation matrix
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Populate transform matrix with actual values

  // Convert and transform point cloud
  autoware_utils::transform_point_cloud_from_ros_msg(cloud_in, pcl_cloud, transform);

  rclcpp::shutdown();
  return 0;
}
```
