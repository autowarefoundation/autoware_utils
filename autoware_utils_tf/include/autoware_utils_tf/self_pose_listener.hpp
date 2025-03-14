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

#ifndef AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_
#define AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

namespace autoware_utils_tf
{
class SelfPoseListener
{
public:
  explicit SelfPoseListener(rclcpp::Node * node) : transform_listener_(node) {}

  void wait_for_first_pose()
  {
    while (rclcpp::ok()) {
      if (get_current_pose()) {
        return;
      }
      RCLCPP_INFO(transform_listener_.get_logger(), "waiting for self pose...");
      rclcpp::Rate(0.2).sleep();
    }
  }

  geometry_msgs::msg::PoseStamped::ConstSharedPtr get_current_pose()
  {
    const auto tf = transform_listener_.get_latest_transform("map", "base_link");
    if (!tf) {
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::PoseStamped>(
      autoware_utils_geometry::transform2pose(*tf));
  }

private:
  TransformListener transform_listener_;
};
}  // namespace autoware_utils_tf

#endif  // AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_
