# autoware_utils Library

## Overview

The **autoware_utils** library is a comprehensive toolkit designed to facilitate the development of autonomous driving applications. This library provides essential utilities for geometry, mathematics, ROS (Robot Operating System) expansions, diagnostics, and more. It is extensively used in the Autoware project to handle common tasks such as geometric calculations, data normalization, message conversions, performance monitoring, and point cloud transformations.

### Design

#### Geometry Module

The geometry module provides classes and functions for handling 2D and 3D points, vectors, polygons, and performing geometric operations:

- **`boost_geometry.hpp`**: Integrates Boost.Geometry for advanced geometric computations, defining point, segment, box, linestring, ring, and polygon types.
- **`alt_geometry.hpp`**: Implements alternative geometric types and operations for 2D vectors and polygons, including vector arithmetic, polygon creation, and various geometric predicates.
- **`ear_clipping.hpp`**: Provides algorithms for triangulating polygons using the ear clipping method.
- **`gjk_2d.hpp`**: Implements the GJK algorithm for fast intersection detection between convex polygons.
- **`sat_2d.hpp`**: Implements the SAT (Separating Axis Theorem) algorithm for detecting intersections between convex polygons.
- **`random_concave_polygon.hpp` and `random_convex_polygon.hpp`**: Generate random concave and convex polygons for testing purposes.
- **`pose_deviation.hpp`**: Calculates deviations between poses in terms of lateral, longitudinal, and yaw angles.
- **`boost_polygon_utils.hpp`**: Utility functions for manipulating polygons, including:
  - Checking if a polygon is clockwise.
  - Rotating polygons around the origin.
  - Converting poses and shapes to polygons.
  - Expanding polygons by an offset.
- **`geometry.hpp`**: Comprehensive geometric operations, including:
  - Distance calculations between points and segments.
  - Curvature computation.
  - Pose transformations and interpolations.
  - Intersection checks for convex polygons using GJK.
  - Conversion between different coordinate systems.

#### ROS Module

The ROS module provides utilities for working with ROS messages and nodes:

- **`debug_publisher.hpp`**: A helper class for publishing debug messages with timestamps.
- **`diagnostics_interface.hpp`**: An interface for publishing diagnostic messages.
- **`msg_covariance.hpp`**: Indices for accessing covariance matrices in ROS messages.
- **`msg_operation.hpp`**: Overloaded operators for quaternion messages.
- **`parameter.hpp`**: Simplifies parameter retrieval and declaration.
- **`polling_subscriber.hpp`**: A subscriber class with different polling policies (latest, newest, all).
- **`processing_time_publisher.hpp`**: Publishes processing times as diagnostic messages.
- **`published_time_publisher.hpp`**: Tracks and publishes the time when messages are published.
- **`self_pose_listener.hpp`**: Listens to the self-pose of the vehicle.
- **`update_param.hpp`**: Updates parameters from remote nodes.
- **`uuid_helper.hpp`**: Utilities for generating and managing UUIDs.
- **`wait_for_param.hpp`**: Waits for parameters from remote nodes.
- **`debug_traits.hpp`**: Traits for identifying debug message types.

#### System Module

The system module provides low-level utilities for performance monitoring and error handling:

- **`backtrace.hpp`**: Prints backtraces for debugging.
- **`lru_cache.hpp`**: Implements an LRU (Least Recently Used) cache.
- **`stop_watch.hpp`**: Measures elapsed time for profiling.
- **`time_keeper.hpp`**: Tracks and reports the processing time of various functions.

## Usage

### Including Headers

To use the Autoware Utils library in your project, include the necessary headers at the top of your source files:

```cpp
#include "autoware_utils/geometry/boost_geometry.hpp"
#include "autoware_utils/math/accumulator.hpp"
#include "autoware_utils/ros/debug_publisher.hpp"
```

or you can include `autoware_utils/autoware_utils.hpp` for all features:

```cpp
#include "autoware_utils/autoware_utils.hpp"
```

### Example Code Snippets

#### Using Vector2d from alt_geometry.hpp

```cpp
#include "autoware_utils/geometry/alt_geometry.hpp"

using namespace autoware_utils::alt;

int main() {
  Vector2d vec1(3.0, 4.0);
  Vector2d vec2(1.0, 2.0);

  // Compute the dot product
  double dot_product = vec1.dot(vec2);

  // Compute the norm
  double norm = vec1.norm();

  return 0;
}
```

### Detailed Usage Examples

#### Update Parameters Dynamically with update_param.hpp

```cpp
#include "autoware_utils/ros/update_param.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("param_node");

  double param_value = 0.0;
  std::vector<rclcpp::Parameter> params = node->get_parameters({"my_param"});

  if (autoware_utils::update_param(params, "my_param", param_value)) {
    RCLCPP_INFO(node->get_logger(), "Updated parameter value: %f", param_value);
  } else {
    RCLCPP_WARN(node->get_logger(), "Parameter 'my_param' not found.");
  }

  rclcpp::shutdown();
  return 0;
}
```

#### Logging Processing Times with ProcessingTimePublisher

```cpp
#include "autoware_utils/ros/processing_time_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <map>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("processing_time_node");

  // Initialize ProcessingTimePublisher
  autoware_utils::ProcessingTimePublisher processing_time_pub(node.get(), "~/debug/processing_time_ms");

  // Simulate some processing times
  std::map<std::string, double> processing_times = {
    {"node1", 0.1}, {"node2", 0.2}, {"node3", 0.3}
  };

  // Publish processing times
  processing_time_pub.publish(processing_times);

  rclcpp::shutdown();
  return 0;
}
```

#### Manipulating Polygons with boost_polygon_utils.hpp

```cpp
#include "autoware_utils/geometry/boost_polygon_utils.hpp"
#include "autoware_utils/geometry/boost_geometry.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("polygon_node");

  // Create a polygon
  autoware_utils::Polygon2d polygon;
  // Assume polygon is populated with points

  // Rotate the polygon by 90 degrees
  autoware_utils::Polygon2d rotated_polygon = autoware_utils::rotate_polygon(polygon, M_PI / 2);

  // Expand the polygon by an offset
  autoware_utils::Polygon2d expanded_polygon = autoware_utils::expand_polygon(polygon, 1.0);

  // Check if the polygon is clockwise
  bool is_clockwise = autoware_utils::is_clockwise(polygon);

  rclcpp::shutdown();
  return 0;
}
```

#### Handling Debug Message Types with debug_traits.hpp

```cpp
#include "autoware_utils/ros/debug_publisher.hpp"
#include "autoware_utils/ros/debug_traits.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("debug_node");

  // Initialize DebugPublisher
  autoware_utils::DebugPublisher debug_pub(node, "/debug");

  // Publish a debug message with custom type
  float debug_data = 42.0;
  debug_pub.publish<autoware_internal_debug_msgs::msg::Float32Stamped>("example", debug_data);

  rclcpp::shutdown();
  return 0;
}
```
