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

#ifndef AUTOWARE_UTILS_TESTING__GTEST__ROS_ENV_HPP_
#define AUTOWARE_UTILS_TESTING__GTEST__ROS_ENV_HPP_

#include <gtest/gtest.h>

namespace autoware_utils_testing::gtest
{

// Create an environment pointor that executes rclcpp init and shutdown.
// Pass it to the AddGlobalTestEnvironment in your gtest main function.
testing::Environment * ros_env(int argc, char ** argv);

}  // namespace autoware_utils_testing::gtest

#endif  // AUTOWARE_UTILS_TESTING__GTEST__ROS_ENV_HPP_
