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

#include "autoware_utils_debug/debug_publisher.hpp"

#include <autoware_utils_testing/mock_node.hpp>

#include <gtest/gtest.h>

#include <memory>

TEST(TestDebugPublisher, Main)
{
  const auto mock = std::make_shared<autoware_utils_testing::MockNode>("mock_node");
  const auto node = std::make_shared<rclcpp::Node>("test_node");

  auto pub = autoware_utils_debug::DebugPublisher(node.get(), "foo/bar");
  pub.publish<autoware_internal_debug_msgs::msg::Float64Stamped>("aaa", 123.456);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mock);
  executor.add_node(node);

  std::thread thread([&executor] { executor.spin(); });
  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  executor.cancel();
  thread.join();
}
