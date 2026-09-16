// Copyright 2025 The Autoware Contributors
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

#include <functional>
#include <memory>

namespace autoware_utils_tf
{
template <class NodeT, class BufferT, class ListenerT>
class SelfPoseListenerT
{
public:
  explicit SelfPoseListenerT(NodeT * node) : transform_listener_(node) {}

  /// @param is_ok Decides whether to keep waiting. The default asks rclcpp, which is the wrong
  ///        context for a node spun by an AgnocastOnly executor: there rclcpp::ok() is false from
  ///        the start and the wait returns without a pose. Such a node passes its own liveness
  ///        check instead.
  void wait_for_first_pose(const std::function<bool()> & is_ok = []() { return rclcpp::ok(); })
  {
    while (is_ok()) {
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
  TransformListenerT<NodeT, BufferT, ListenerT> transform_listener_;
};

using SelfPoseListener =
  SelfPoseListenerT<rclcpp::Node, tf2_ros::Buffer, tf2_ros::TransformListener>;
}  // namespace autoware_utils_tf

#endif  // AUTOWARE_UTILS_TF__SELF_POSE_LISTENER_HPP_
