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

#ifndef AUTOWARE_UTILS_TESTING__SPIN_THREAD_HPP_
#define AUTOWARE_UTILS_TESTING__SPIN_THREAD_HPP_

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <optional>
#include <thread>

namespace autoware_utils_testing
{

template <class Executor = rclcpp::executors::SingleThreadedExecutor>
class SpinThread
{
public:
  void start()
  {
    thread_ = std::thread([this] { executor_.spin(); });
    while (rclcpp::ok() && !executor_.is_spinning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void stop()
  {
    executor_.cancel();
    thread_.join();
  }

  void add_node(std::shared_ptr<rclcpp::Node> node) { executor_.add_node(node); }

private:
  Executor executor_;
  std::thread thread_;
};

}  // namespace autoware_utils_testing

#endif  // AUTOWARE_UTILS_TESTING__SPIN_THREAD_HPP_
