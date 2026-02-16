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

TEST(TestPollingSubscriber, InitialValues)
{
  const auto node = std::make_shared<rclcpp::Node>("test_initial_values");

  const auto latest_sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String, autoware_utils_rclcpp::polling_policy::Latest>::
    create_subscription(node.get(), "/test/initial_latest", 1);
  EXPECT_EQ(latest_sub->latest_timestamp(), std::nullopt);
  EXPECT_EQ(latest_sub->take_data(), nullptr);

  const auto newest_sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String, autoware_utils_rclcpp::polling_policy::Newest>::
    create_subscription(node.get(), "/test/initial_newest", 1);
  EXPECT_EQ(newest_sub->latest_timestamp(), std::nullopt);
  EXPECT_EQ(newest_sub->take_data(), nullptr);

  const auto all_sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String, autoware_utils_rclcpp::polling_policy::All>::
    create_subscription(node.get(), "/test/initial_all", 1);
  EXPECT_EQ(all_sub->latest_timestamp(), std::nullopt);
  EXPECT_TRUE(all_sub->take_data().empty());
}

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

  const auto timestamp = sub->latest_timestamp();
  EXPECT_TRUE(timestamp.has_value());

  rclcpp::Duration duration = sub_node->now() - timestamp.value();
  EXPECT_LE(duration.seconds(), 1.0);

  executor.cancel();
  thread.join();
}
