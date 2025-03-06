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
#include <autoware_utils_testing/spin_thread.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <thread>

void sleep(double seconds)
{
  const auto ms = static_cast<int64_t>(seconds * 1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

TEST(TestDebugPublisher, Main)
{
  namespace DebugMsgs = autoware_internal_debug_msgs::msg;

  const auto mock = std::make_shared<autoware_utils_testing::MockNode>("mock_node");
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  auto pub = autoware_utils_debug::DebugPublisher(node.get(), "debug");
  auto sub1 = mock->sub<DebugMsgs::Float64Stamped>("/debug/foo", rclcpp::QoS(5));
  auto sub2 = mock->sub<DebugMsgs::StringStamped>("/debug/bar", rclcpp::QoS(5));

  autoware_utils_testing::SpinThread thread;
  thread.add_node(mock);
  thread.add_node(node);
  thread.start();
  pub.publish<DebugMsgs::Float64Stamped>("foo", 12.34);
  pub.publish<DebugMsgs::Float64Stamped>("foo", 56.78);
  pub.publish<DebugMsgs::StringStamped>("bar", "hello");
  pub.publish<DebugMsgs::StringStamped>("bar", "world");
  sleep(0.01);
  thread.stop();

  EXPECT_EQ(sub1->data.size(), 2);
  EXPECT_EQ(sub1->data[0].data, 12.34);
  EXPECT_EQ(sub1->data[1].data, 56.78);

  EXPECT_EQ(sub2->data.size(), 2);
  EXPECT_EQ(sub2->data[0].data, "hello");
  EXPECT_EQ(sub2->data[1].data, "world");
}
