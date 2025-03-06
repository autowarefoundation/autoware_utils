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

#ifndef AUTOWARE_UTILS_TESTING__MOCK_NODE_HPP_
#define AUTOWARE_UTILS_TESTING__MOCK_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware_utils_testing
{

template <class T>
struct Topic
{
  int count;
  std::vector<T> data;
};

class MockNode : public rclcpp::Node
{
public:
  explicit MockNode(const std::string & name) : Node(name) {}

  template <class T>
  std::shared_ptr<Topic<T>> sub(const std::string & name, const rclcpp::QoS & qos);

private:
  std::unordered_map<std::string, std::shared_ptr<void>> topics_;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
};

template <class T>
std::shared_ptr<Topic<T>> MockNode::sub(const std::string & name, const rclcpp::QoS & qos)
{
  const auto topic = std::make_shared<Topic<T>>();
  const auto sub = create_subscription<T>(name, qos, [this, topic](const T & msg) {
    topic->count += 1;
    topic->data.push_back(msg);
  });
  subs_[name] = sub;
  topics_[name] = topic;
  return topic;
}

}  // namespace autoware_utils_testing

#endif  // AUTOWARE_UTILS_TESTING__MOCK_NODE_HPP_
