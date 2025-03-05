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

#include "autoware_utils_rclcpp/parameter.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

TEST(TestParameter, GetOrDeclare)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("foo", 12345);
  options.append_parameter_override("bar", "str");
  const auto node = std::make_shared<rclcpp::Node>("param_get_or_declare", options);
  const auto p1 = autoware_utils_rclcpp::get_or_declare_parameter<int>(*node, "foo");
  const auto p2 = autoware_utils_rclcpp::get_or_declare_parameter<int>(*node, "foo");
  const auto p3 = autoware_utils_rclcpp::get_or_declare_parameter<std::string>(*node, "bar");
  const auto p4 = autoware_utils_rclcpp::get_or_declare_parameter<std::string>(*node, "bar");
  EXPECT_EQ(p1, 12345);
  EXPECT_EQ(p2, 12345);
  EXPECT_EQ(p3, "str");
  EXPECT_EQ(p4, "str");
}

TEST(TestParameter, Update)
{
  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("foo", 12345),
    rclcpp::Parameter("bar", "str"),
  };
  int foo = 0;
  std::string bar = "default";
  std::string baz = "default";

  EXPECT_TRUE(autoware_utils_rclcpp::update_param(params, "foo", foo));
  EXPECT_TRUE(autoware_utils_rclcpp::update_param(params, "bar", bar));
  EXPECT_EQ(foo, 12345);
  EXPECT_EQ(bar, "str");

  EXPECT_FALSE(autoware_utils_rclcpp::update_param(params, "baz", baz));
  EXPECT_EQ(baz, "default");
}
