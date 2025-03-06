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

#include <autoware_utils_testing/mock_node.hpp>
#include <autoware_utils_testing/spin_thread.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <thread>

TEST(TestDebugPublisher, Main)
{
  const auto mock = std::make_shared<autoware_utils_testing::MockNode>("mock_node");
  const auto node = std::make_shared<rclcpp::Node>("test_node");

  auto pub1 = node->create_publisher<std_msgs::msg::Int32>("/test/foo", rclcpp::QoS(1));
  auto pub2 = node->create_publisher<std_msgs::msg::String>("/test/bar", rclcpp::QoS(1));
  auto sub1 = mock->sub<std_msgs::msg::Int32>("/test/foo", rclcpp::QoS(1));
  auto sub2 = mock->sub<std_msgs::msg::String>("/test/bar", rclcpp::QoS(1));

  autoware_utils_testing::SpinThread thread;
  thread.add_node(mock);
  thread.add_node(node);
  thread.start();
  pub1->publish(std_msgs::build<std_msgs::msg::Int32>().data(123));
  pub2->publish(std_msgs::build<std_msgs::msg::String>().data("hello"));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  pub1->publish(std_msgs::build<std_msgs::msg::Int32>().data(456));
  pub2->publish(std_msgs::build<std_msgs::msg::String>().data("world"));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  thread.stop();

  EXPECT_EQ(sub1->data.size(), 2);
  EXPECT_EQ(sub1->data[0].data, 123);
  EXPECT_EQ(sub1->data[1].data, 456);

  EXPECT_EQ(sub2->data.size(), 2);
  EXPECT_EQ(sub2->data[0].data, "hello");
  EXPECT_EQ(sub2->data[1].data, "world");
}
