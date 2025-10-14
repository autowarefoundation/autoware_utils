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

#include "autoware_utils_diagnostics/timeout_diagnostics.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

TEST(TimeoutDiagTest, Instantiation)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  autoware_utils_diagnostics::TimeoutDiag::Params params{1.0, 2.0};
  autoware_utils_diagnostics::TimeoutDiag timeout_diag(params, *node->get_clock(), "test_diag");
}
