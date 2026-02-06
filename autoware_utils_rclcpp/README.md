# autoware_utils_rclcpp

## Overview

The **autoware_utils** library is a comprehensive toolkit designed to facilitate the development of autonomous driving applications.
This package provides essential utilities for rclcpp.
It is extensively used in the Autoware project to handle common tasks such as handling parameters, topics and services.

## Design

- **`parameter.hpp`**: Simplifies parameter declaration, retrieval, updating, and waiting.
- **`polling_subscriber.hpp`**: A subscriber class with different polling policies (latest, newest, all).

## Example Code Snippets

### Update Parameters Dynamically with update_param.hpp

```cpp
#include <autoware_utils_rclcpp/update_param.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("param_node");

  double param_value = 0.0;
  std::vector<rclcpp::Parameter> params = node->get_parameters({"my_param"});

  if (autoware_utils::update_param(params, "my_param", param_value)) {
    RCLCPP_INFO(node->get_logger(), "Updated parameter value: %f", param_value);
  } else {
    RCLCPP_WARN(node->get_logger(), "Parameter 'my_param' not found.");
  }

  rclcpp::shutdown();
  return 0;
}
```

### Subscribe to Topics with polling_subscriber.hpp

```cpp
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("polling_node");

  // Latest policy (default): returns the last received message, or the previous one if no new data.
  auto latest_sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String>::create_subscription(node.get(), "/topic", 1);

  // Newest policy: returns the new message only, or nullptr if no new data.
  auto newest_sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String, autoware_utils_rclcpp::polling_policy::Newest>::
    create_subscription(node.get(), "/topic", 1);

  // All policy: returns all received messages as a vector.
  auto all_sub = autoware_utils_rclcpp::InterProcessPollingSubscriber<
    std_msgs::msg::String, autoware_utils_rclcpp::polling_policy::All>::
    create_subscription(node.get(), "/topic", rclcpp::QoS{10});

  // Retrieve data and timestamp.
  auto msg = latest_sub->take_data();          // std_msgs::msg::String::ConstSharedPtr
  auto stamp = latest_sub->latest_timestamp(); // rclcpp::Time

  rclcpp::shutdown();
  return 0;
}
```

The `latest_timestamp()` method returns the source timestamp of the last received message. With the default `Latest` policy, if `latest_timestamp()` equals `rclcpp::Time{0, 0}`, the return value of `take_data()` is `nullptr`.
