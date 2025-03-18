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

#ifndef AUTOWARE_UTILS_GEOMETRY__POSE_DEVIATION_HPP_
#define AUTOWARE_UTILS_GEOMETRY__POSE_DEVIATION_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace autoware_utils_geometry
{
struct PoseDeviation
{
  double lateral{0.0};
  double longitudinal{0.0};
  double yaw{0.0};
};

double calc_lateral_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point);

double calc_longitudinal_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point);

double calc_yaw_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose);

PoseDeviation calc_pose_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose);

}  // namespace autoware_utils_geometry

#endif  // AUTOWARE_UTILS_GEOMETRY__POSE_DEVIATION_HPP_
