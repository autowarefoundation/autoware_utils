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

#include "autoware_utils_logging/logger_level_configure.hpp"

#include <gtest/gtest.h>

#include <memory>

class LoggerLevelConfigureNode : public rclcpp::Node
{
public:
  using LoggerLevelConfigure = autoware_utils_logging::LoggerLevelConfigure;

  LoggerLevelConfigureNode() : rclcpp::Node("__auto__")
  {
    logger_configure_ = std::make_unique<LoggerLevelConfigure>(this);
  }

private:
  std::unique_ptr<LoggerLevelConfigure> logger_configure_;
};

TEST(TestLoggerLevelConfigure, Instantiation)
{
  const auto node = std::make_shared<LoggerLevelConfigureNode>();
}
