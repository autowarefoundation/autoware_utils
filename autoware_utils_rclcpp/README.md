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
