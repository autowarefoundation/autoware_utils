// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS_VISUALIZATION__MARKER_HELPER_HPP_
#define AUTOWARE_UTILS_VISUALIZATION__MARKER_HELPER_HPP_

#include <rclcpp/time.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <optional>
#include <string>

namespace autoware_utils_visualization
{
inline geometry_msgs::msg::Point create_marker_position(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

inline geometry_msgs::msg::Quaternion create_marker_orientation(
  double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;
  return quaternion;
}

inline geometry_msgs::msg::Vector3 create_marker_scale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

inline std_msgs::msg::ColorRGBA create_marker_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::msg::Marker create_default_marker(
  const std::string & frame_id, const rclcpp::Time & now, const std::string & ns, const int32_t id,
  const int32_t type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::Marker create_deleted_default_marker(
  const rclcpp::Time & now, const std::string & ns, const int32_t id);

void append_marker_array(
  const visualization_msgs::msg::MarkerArray & additional_marker_array,
  visualization_msgs::msg::MarkerArray * marker_array,
  const std::optional<rclcpp::Time> & current_time = {});

}  // namespace autoware_utils_visualization

#endif  // AUTOWARE_UTILS_VISUALIZATION__MARKER_HELPER_HPP_
