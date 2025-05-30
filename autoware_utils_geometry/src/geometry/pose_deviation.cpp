// Copyright 2023 TIER IV, Inc.
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

#include "autoware_utils_geometry/pose_deviation.hpp"

#include "autoware_utils_math/normalization.hpp"

#include <tf2/utils.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware_utils_geometry
{

double calc_lateral_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Eigen::Vector3d base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Eigen::Vector3d diff_vec{dx, dy, 0};

  const Eigen::Vector3d cross_vec = base_unit_vec.cross(diff_vec);

  return cross_vec.z();
}

double calc_longitudinal_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Eigen::Vector3d base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Eigen::Vector3d diff_vec{dx, dy, 0};

  return base_unit_vec.dot(diff_vec);
}

double calc_yaw_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  const auto base_yaw = tf2::getYaw(base_pose.orientation);
  const auto target_yaw = tf2::getYaw(target_pose.orientation);
  return autoware_utils_math::normalize_radian(target_yaw - base_yaw);
}

PoseDeviation calc_pose_deviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  PoseDeviation deviation{};

  deviation.lateral = calc_lateral_deviation(base_pose, target_pose.position);
  deviation.longitudinal = calc_longitudinal_deviation(base_pose, target_pose.position);
  deviation.yaw = calc_yaw_deviation(base_pose, target_pose);

  return deviation;
}
}  // namespace autoware_utils_geometry
