# autoware_utils_geometry

## Overview

The **autoware_utils** library is a comprehensive toolkit designed to facilitate the development of autonomous driving applications.
This package provides essential utilities for geometry.
It is extensively used in the Autoware project to handle common tasks such as geometric calculations and message conversions.

## Design

The message modules.

- **`covariance.hpp`**: Indices for accessing covariance matrices in ROS messages.
- **`operation.hpp`**: Overloaded operators for quaternion messages.

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

## Example Code Snippets

### Using Vector2d from alt_geometry.hpp

```cpp
#include "autoware_utils/geometry/alt_geometry.hpp"

using namespace autoware_utils_geometry::alt;

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

### Manipulating Polygons with boost_polygon_utils.hpp

```cpp
#include "autoware_utils/geometry/boost_polygon_utils.hpp"
#include "autoware_utils/geometry/boost_geometry.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("polygon_node");

  // Create a polygon
  autoware_utils_geometry::Polygon2d polygon;
  // Assume polygon is populated with points

  // Rotate the polygon by 90 degrees
  autoware_utils_geometry::Polygon2d rotated_polygon = autoware_utils_geometry::rotate_polygon(polygon, M_PI / 2);

  // Expand the polygon by an offset
  autoware_utils_geometry::Polygon2d expanded_polygon = autoware_utils_geometry::expand_polygon(polygon, 1.0);

  // Check if the polygon is clockwise
  bool is_clockwise = autoware_utils_geometry::is_clockwise(polygon);

  rclcpp::shutdown();
  return 0;
}
```
