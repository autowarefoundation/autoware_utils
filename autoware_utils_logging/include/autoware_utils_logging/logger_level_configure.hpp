// Copyright 2023 Tier IV, Inc.
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

// =============== Note ===============
// This is a util class implementation of the logger_config_component provided by ROS 2
// https://github.com/ros2/demos/blob/humble/logging_demo/src/logger_config_component.cpp
//
// When ROS 2 officially supports the set_logger_level option in release version, this class can be
// removed.
// https://github.com/ros2/ros2/issues/1355

// =============== How to use ===============
// ___In your_node.hpp___
// #include "autoware/universe_utils/ros/logger_level_configure.hpp"
// class YourNode : public rclcpp::Node {
//   ...
//
//   // Define logger_configure as a node class member variable
//   std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;
// }
//
// ___In your_node.cpp___
// YourNode::YourNode() {
//   ...
//
//   // Set up logger_configure
//   logger_configure_ = std::make_unique<LoggerLevelConfigure>(this);
// }

#ifndef AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_
#define AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_

#include <logging_demo/srv/config_logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcutils/logging.h>

#include <functional>
#include <string>
#include <utility>

namespace autoware_utils_logging
{

template <typename NodeT = rclcpp::Node>
class BasicLoggerLevelConfigure
{
private:
  using ConfigLogger = logging_demo::srv::ConfigLogger;
  using CallbackT = std::function<void(
    ConfigLogger::Request::SharedPtr, ConfigLogger::Response::SharedPtr)>;
  using ServicePtr = decltype(std::declval<NodeT *>()->template create_service<ConfigLogger>(
    std::declval<std::string>(), std::declval<CallbackT>()));

public:
  explicit BasicLoggerLevelConfigure(NodeT * node) : ros_logger_(node->get_logger())
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    srv_config_logger_ = node->template create_service<ConfigLogger>(
      "~/config_logger",
      std::bind(&BasicLoggerLevelConfigure::on_logger_config_service, this, _1, _2));
  }

private:
  rclcpp::Logger ros_logger_;
  ServicePtr srv_config_logger_;

  void on_logger_config_service(
    const ConfigLogger::Request::SharedPtr request,
    const ConfigLogger::Response::SharedPtr response)
  {
    int logging_severity;
    const auto ret_level = rcutils_logging_severity_level_from_string(
      request->level.c_str(), rcl_get_default_allocator(), &logging_severity);

    if (ret_level != RCUTILS_RET_OK) {
      response->success = false;
      RCLCPP_WARN_STREAM(
        ros_logger_, "Failed to change logger level for "
                       << request->logger_name
                       << " due to an invalid logging severity: " << request->level);
      return;
    }

    const auto ret_set =
      rcutils_logging_set_logger_level(request->logger_name.c_str(), logging_severity);

    if (ret_set != RCUTILS_RET_OK) {
      response->success = false;
      RCLCPP_WARN_STREAM(ros_logger_, "Failed to set logger level for " << request->logger_name);
      return;
    }

    response->success = true;
    RCLCPP_INFO_STREAM(
      ros_logger_, "Logger level [" << request->level << "] is set for " << request->logger_name);
  }
};

using LoggerLevelConfigure = BasicLoggerLevelConfigure<rclcpp::Node>;

}  // namespace autoware_utils_logging

#endif  // AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_
