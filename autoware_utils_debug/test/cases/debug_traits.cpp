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

#include "autoware_utils_debug/debug_traits.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(TestDebugTraits, TrueCases)
{
  using autoware_utils_debug::debug_traits::is_debug_message;

  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::BoolStamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Float32Stamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Float64Stamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Int32MultiArrayStamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Int32Stamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Int64MultiArrayStamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::Int64Stamped>::value);
  EXPECT_TRUE(is_debug_message<autoware_internal_debug_msgs::msg::StringStamped>::value);
}

TEST(TestDebugTraits, FalseCases)
{
  using autoware_utils_debug::debug_traits::is_debug_message;

  EXPECT_FALSE(is_debug_message<std::string>::value);
  EXPECT_FALSE(is_debug_message<bool>::value);
  EXPECT_FALSE(is_debug_message<int32_t>::value);
  EXPECT_FALSE(is_debug_message<int64_t>::value);
  EXPECT_FALSE(is_debug_message<float>::value);
  EXPECT_FALSE(is_debug_message<double>::value);
}
