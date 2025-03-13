// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_UTILS_GEOMETRY__BOOST_POLYGON_UTILS_HPP_
#define AUTOWARE_UTILS_GEOMETRY__BOOST_POLYGON_UTILS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware_utils_geometry
{
bool is_clockwise(const Polygon2d & polygon);
Polygon2d inverse_clockwise(const Polygon2d & polygon);
geometry_msgs::msg::Polygon rotate_polygon(
  const geometry_msgs::msg::Polygon & polygon, const double & angle);
/// @brief rotate a polygon by some angle around the origin
/// @param[in] polygon input polygon
/// @param[in] angle angle of rotation [rad]
/// @return rotated polygon
Polygon2d rotate_polygon(const Polygon2d & polygon, const double angle);
Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);
Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);
Polygon2d to_polygon2d(const autoware_perception_msgs::msg::DetectedObject & object);
Polygon2d to_polygon2d(const autoware_perception_msgs::msg::TrackedObject & object);
Polygon2d to_polygon2d(const autoware_perception_msgs::msg::PredictedObject & object);
Polygon2d to_footprint(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_to_front,
  const double base_to_rear, const double width);
double get_area(const autoware_perception_msgs::msg::Shape & shape);
Polygon2d expand_polygon(const Polygon2d & input_polygon, const double offset);
}  // namespace autoware_utils_geometry

#endif  // AUTOWARE_UTILS_GEOMETRY__BOOST_POLYGON_UTILS_HPP_
