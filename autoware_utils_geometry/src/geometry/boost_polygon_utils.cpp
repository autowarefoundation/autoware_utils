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

#include "autoware_utils_geometry/boost_polygon_utils.hpp"

#include "autoware_utils_geometry/geometry.hpp"

#include <tf2/utils.hpp>

#include <boost/geometry/geometry.hpp>

namespace
{
namespace bg = boost::geometry;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

void append_point_to_polygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

void append_point_to_polygon(Polygon2d & polygon, const Point2d & point)
{
  bg::append(polygon.outer(), point);
}

/*
 * NOTE: Area is negative when footprint.points is clock wise.
 *       Area is positive when footprint.points is anti clock wise.
 */
double get_polygon_area(const geometry_msgs::msg::Polygon & footprint)
{
  double area = 0.0;

  for (size_t i = 0; i < footprint.points.size(); ++i) {
    size_t j = (i + 1) % footprint.points.size();
    area += 0.5 * (footprint.points.at(i).x * footprint.points.at(j).y -
                   footprint.points.at(j).x * footprint.points.at(i).y);
  }

  return area;
}

double get_rectangle_area(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>(dimensions.x * dimensions.y);
}

double get_circle_area(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>((dimensions.x / 2.0) * (dimensions.x / 2.0) * M_PI);
}
}  // namespace

namespace autoware_utils_geometry
{
bool is_clockwise(const Polygon2d & polygon)
{
  const int n = polygon.outer().size();
  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }

  return sum < 0.0;
}

Polygon2d inverse_clockwise(const Polygon2d & polygon)
{
  auto output_polygon = polygon;
  boost::geometry::reverse(output_polygon);
  return output_polygon;
}

geometry_msgs::msg::Polygon rotate_polygon(
  const geometry_msgs::msg::Polygon & polygon, const double & angle)
{
  const double cos = std::cos(angle);
  const double sin = std::sin(angle);
  geometry_msgs::msg::Polygon rotated_polygon;
  for (const auto & point : polygon.points) {
    auto rotated_point = point;
    rotated_point.x = cos * point.x - sin * point.y;
    rotated_point.y = sin * point.x + cos * point.y;
    rotated_polygon.points.push_back(rotated_point);
  }
  return rotated_polygon;
}

Polygon2d rotate_polygon(const Polygon2d & polygon, const double angle)
{
  Polygon2d rotated_polygon;
  const boost::geometry::strategy::transform::rotate_transformer<
    boost::geometry::radian, double, 2, 2>
    rotation(-angle);
  boost::geometry::transform(polygon, rotated_polygon, rotation);
  return rotated_polygon;
}

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape)
{
  Polygon2d polygon;

  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto point0 = autoware_utils_geometry::calc_offset_pose(
                          pose, shape.dimensions.x / 2.0, shape.dimensions.y / 2.0, 0.0)
                          .position;
    const auto point1 = autoware_utils_geometry::calc_offset_pose(
                          pose, -shape.dimensions.x / 2.0, shape.dimensions.y / 2.0, 0.0)
                          .position;
    const auto point2 = autoware_utils_geometry::calc_offset_pose(
                          pose, -shape.dimensions.x / 2.0, -shape.dimensions.y / 2.0, 0.0)
                          .position;
    const auto point3 = autoware_utils_geometry::calc_offset_pose(
                          pose, shape.dimensions.x / 2.0, -shape.dimensions.y / 2.0, 0.0)
                          .position;

    append_point_to_polygon(polygon, point0);
    append_point_to_polygon(polygon, point1);
    append_point_to_polygon(polygon, point2);
    append_point_to_polygon(polygon, point3);
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    const double radius = shape.dimensions.x / 2.0;
    constexpr int circle_discrete_num = 6;
    for (int i = 0; i < circle_discrete_num; ++i) {
      geometry_msgs::msg::Point point;
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(circle_discrete_num)) *
                  radius +
                pose.position.x;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(circle_discrete_num)) *
                  radius +
                pose.position.y;
      append_point_to_polygon(polygon, point);
    }
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const double poly_yaw = tf2::getYaw(pose.orientation);
    const auto rotated_footprint = rotate_polygon(shape.footprint, poly_yaw);
    for (const auto rel_point : rotated_footprint.points) {
      geometry_msgs::msg::Point abs_point;
      abs_point.x = pose.position.x + rel_point.x;
      abs_point.y = pose.position.y + rel_point.y;

      append_point_to_polygon(polygon, abs_point);
    }
  } else {
    throw std::logic_error("The shape type is not supported in autoware_utils.");
  }

  // NOTE: push back the first point in order to close polygon
  if (!polygon.outer().empty()) {
    append_point_to_polygon(polygon, polygon.outer().front());
  }

  return is_clockwise(polygon) ? polygon : inverse_clockwise(polygon);
}

autoware_utils_geometry::Polygon2d to_polygon2d(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  return autoware_utils_geometry::to_polygon2d(
    object.kinematics.pose_with_covariance.pose, object.shape);
}

autoware_utils_geometry::Polygon2d to_polygon2d(
  const autoware_perception_msgs::msg::TrackedObject & object)
{
  return autoware_utils_geometry::to_polygon2d(
    object.kinematics.pose_with_covariance.pose, object.shape);
}

autoware_utils_geometry::Polygon2d to_polygon2d(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  return autoware_utils_geometry::to_polygon2d(
    object.kinematics.initial_pose_with_covariance.pose, object.shape);
}

Polygon2d to_footprint(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_to_front,
  const double base_to_rear, const double width)
{
  using autoware_utils_geometry::calc_offset_pose;
  Polygon2d polygon;
  const auto point0 = calc_offset_pose(base_link_pose, base_to_front, width / 2.0, 0.0).position;
  const auto point1 = calc_offset_pose(base_link_pose, base_to_front, -width / 2.0, 0.0).position;
  const auto point2 = calc_offset_pose(base_link_pose, -base_to_rear, -width / 2.0, 0.0).position;
  const auto point3 = calc_offset_pose(base_link_pose, -base_to_rear, width / 2.0, 0.0).position;

  append_point_to_polygon(polygon, point0);
  append_point_to_polygon(polygon, point1);
  append_point_to_polygon(polygon, point2);
  append_point_to_polygon(polygon, point3);
  append_point_to_polygon(polygon, point0);

  return is_clockwise(polygon) ? polygon : inverse_clockwise(polygon);
}

double get_area(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return get_rectangle_area(shape.dimensions);
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return get_circle_area(shape.dimensions);
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    return get_polygon_area(shape.footprint);
  }

  throw std::logic_error("The shape type is not supported in autoware_utils.");
}

// NOTE: The number of vertices on the expanded polygon by boost::geometry::buffer
//       is larger than the original one.
//       This function fixes the issue.
Polygon2d expand_polygon(const Polygon2d & input_polygon, const double offset)
{
  // NOTE: input_polygon is supposed to have a duplicated point.
  const size_t num_points = input_polygon.outer().size() - 1;

  Polygon2d expanded_polygon;
  for (size_t i = 0; i < num_points; ++i) {
    const auto & curr_p = input_polygon.outer().at(i);
    const auto & next_p = input_polygon.outer().at(i + 1);
    const auto & prev_p =
      i == 0 ? input_polygon.outer().at(num_points - 1) : input_polygon.outer().at(i - 1);

    Eigen::Vector2d current_to_next(next_p.x() - curr_p.x(), next_p.y() - curr_p.y());
    Eigen::Vector2d current_to_prev(prev_p.x() - curr_p.x(), prev_p.y() - curr_p.y());
    current_to_next.normalize();
    current_to_prev.normalize();

    const Eigen::Vector2d offset_vector = (-current_to_next - current_to_prev).normalized();
    const double theta = std::acos(offset_vector.dot(current_to_next));
    const double scaled_offset = offset / std::sin(theta);
    const Eigen::Vector2d offset_point =
      Eigen::Vector2d(curr_p.x(), curr_p.y()) + offset_vector * scaled_offset;

    expanded_polygon.outer().push_back(Point2d(offset_point.x(), offset_point.y()));
  }

  boost::geometry::correct(expanded_polygon);
  return expanded_polygon;
}
}  // namespace autoware_utils_geometry
