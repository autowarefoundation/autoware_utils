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

#ifndef AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_
#define AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>
#include <type_traits>

namespace autoware_utils_tf
{
template <class NodeT, class BufferT, class ListenerT>
class TransformListenerT
{
public:
  explicit TransformListenerT(NodeT * node) : clock_(node->get_clock()), logger_(node->get_logger())
  {
    tf_buffer_ = std::make_shared<BufferT>(clock_);
    // The two buffer backends take different TransformListener constructors.
    if constexpr (std::is_same_v<BufferT, tf2_ros::Buffer>) {
      tf_listener_ = std::make_shared<ListenerT>(*tf_buffer_);
    } else {
      tf_listener_ = std::make_shared<ListenerT>(*tf_buffer_, *node);
    }
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr get_latest_transform(
    const std::string & from, const std::string & to)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(from, to, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "failed to get transform from %s to %s: %s", from.c_str(),
        to.c_str(), ex.what());
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr get_transform(
    const std::string & from, const std::string & to, const rclcpp::Time & time,
    const rclcpp::Duration & duration)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(from, to, time, duration);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "failed to get transform from %s to %s: %s", from.c_str(),
        to.c_str(), ex.what());
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  }

  rclcpp::Logger get_logger() { return logger_; }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::shared_ptr<BufferT> tf_buffer_;
  std::shared_ptr<ListenerT> tf_listener_;
};

using TransformListener =
  TransformListenerT<rclcpp::Node, tf2_ros::Buffer, tf2_ros::TransformListener>;
}  // namespace autoware_utils_tf

#endif  // AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_
