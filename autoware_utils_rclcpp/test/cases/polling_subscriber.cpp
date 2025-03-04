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

#include "autoware_utils_rclcpp/polling_subscriber.hpp"

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

TEST(TestPollingSubscriber, PubSub)
{
  const auto pub_node = std::make_shared<rclcpp::Node>("pub_node");
  const auto sub_node = std::make_shared<rclcpp::Node>("sub_node");

  const auto pub = pub_node->create_publisher<std_msgs::msg::String>("/test/text", 1);
  const auto sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String>::create_subscription(sub_node.get(), "/test/text", 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread thread([&executor] { executor.spin(); });
  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std_msgs::msg::String pub_msg;
  pub_msg.data = "foo-bar";
  pub->publish(pub_msg);

  const auto sub_msg = sub->take_data();
  EXPECT_NE(sub_msg, nullptr);
  EXPECT_EQ(sub_msg->data, pub_msg.data);

  executor.cancel();
  thread.join();
}
