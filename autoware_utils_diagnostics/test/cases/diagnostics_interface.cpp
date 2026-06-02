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

#include "autoware_utils_diagnostics/diagnostics_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

TEST(TestDiagnosticsInterface, Instantiation)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  autoware_utils_diagnostics::DiagnosticsInterface(node.get(), "diag_name");
}

// Characterization test that pins the observable behavior of DiagnosticsInterface across
// clear() cycles. It proves that publishing produces output-identical messages whether or not
// clear() releases the underlying buffer (i.e. the per-cycle shrink_to_fit removal is
// behavior-preserving). It mirrors the executor-in-thread pattern from
// autoware_utils_rclcpp/test/cases/polling_subscriber.cpp.
TEST(TestDiagnosticsInterface, PubSubAcrossClearCycles)
{
  const auto node = std::make_shared<rclcpp::Node>("test_diag_pubsub");
  autoware_utils_diagnostics::DiagnosticsInterface diag(node.get(), "diag_name");

  std::mutex mutex;
  std::optional<diagnostic_msgs::msg::DiagnosticArray> last_received;
  [[maybe_unused]] const auto sub =
    node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [&mutex, &last_received](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        last_received = *msg;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread thread([&executor] { executor.spin(); });

  // Ensure the executor is always stopped and the thread joined, even when a gtest
  // ASSERT_* early-returns; otherwise a joinable std::thread destructor would call
  // std::terminate() and mask the real failure.
  struct ExecutorStopGuard
  {
    rclcpp::executors::SingleThreadedExecutor & exec;
    std::thread & th;
    ~ExecutorStopGuard()
    {
      exec.cancel();
      if (th.joinable()) {
        th.join();
      }
    }
  } stop_guard{executor, thread};

  while (rclcpp::ok() && !executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Helper that waits until a message is received after a publish (or fails on timeout).
  const auto wait_for_message =
    [&mutex, &last_received]() -> std::optional<diagnostic_msgs::msg::DiagnosticArray> {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (last_received.has_value()) {
          return last_received;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::nullopt;
  };

  const auto reset_received = [&mutex, &last_received]() {
    std::lock_guard<std::mutex> lock(mutex);
    last_received.reset();
  };

  // Cycle 1: populate values, raise level to WARN with a message, then publish.
  {
    reset_received();
    diag.add_key_value("k1", std::string("v1"));
    diag.add_key_value("k2", 3);
    diag.update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::WARN, "warn-msg");
    diag.publish(node->now());

    const auto received = wait_for_message();
    ASSERT_TRUE(received.has_value()) << "Cycle 1 message was not received within the timeout";

    const auto & status = received->status;
    ASSERT_EQ(status.size(), 1u);
    ASSERT_EQ(status[0].values.size(), 2u);
    EXPECT_EQ(status[0].values[0].key, "k1");
    EXPECT_EQ(status[0].values[0].value, "v1");
    EXPECT_EQ(status[0].values[1].key, "k2");
    EXPECT_EQ(status[0].values[1].value, "3");
    EXPECT_EQ(status[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_EQ(status[0].message, "warn-msg");
  }

  // Cycle 2: clear() must empty the values (regardless of shrink_to_fit), reset level to OK and
  // let create_diagnostics_array rewrite the message to "OK".
  {
    reset_received();
    diag.clear();
    diag.add_key_value("k3", std::string("v3"));
    diag.publish(node->now());

    const auto received = wait_for_message();
    ASSERT_TRUE(received.has_value()) << "Cycle 2 message was not received within the timeout";

    const auto & status = received->status;
    ASSERT_EQ(status.size(), 1u);
    ASSERT_EQ(status[0].values.size(), 1u);
    EXPECT_EQ(status[0].values[0].key, "k3");
    EXPECT_EQ(status[0].values[0].value, "v3");
    EXPECT_EQ(status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(status[0].message, "OK");
  }

  // Duplicate key: re-adding an existing key overwrites the value instead of appending a new entry.
  {
    reset_received();
    diag.clear();
    diag.add_key_value("k1", std::string("a"));
    diag.add_key_value("k1", std::string("b"));
    diag.publish(node->now());

    const auto received = wait_for_message();
    ASSERT_TRUE(received.has_value()) << "duplicate-key message not received within the timeout";

    const auto & status = received->status;
    ASSERT_EQ(status.size(), 1u);
    ASSERT_EQ(status[0].values.size(), 1u);
    EXPECT_EQ(status[0].values[0].key, "k1");
    EXPECT_EQ(status[0].values[0].value, "b");
  }
}
