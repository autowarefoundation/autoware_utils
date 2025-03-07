# autoware_utils_debug

## Overview

The **autoware_utils** library is a comprehensive toolkit designed to facilitate the development of autonomous driving applications.
This package provides essential utilities for debug.
It is extensively used in the Autoware project to handle common tasks such as publishing debug data and measuring time.

## Design

- **`debug_publisher.hpp`**: A helper class for publishing debug messages with timestamps.
- **`debug_traits.hpp`**: Traits for identifying debug message types.
- **`processing_time_publisher.hpp`**: Publishes processing times as diagnostic messages.
- **`published_time_publisher.hpp`**: Tracks and publishes the time when messages are published.
- **`time_keeper.hpp`**: Tracks and reports the processing time of various functions.

### Example Code Snippets

### Handling Debug Message Types with debug_traits.hpp

```cpp
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/debug_traits.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("debug_node");

  // Initialize DebugPublisher
  autoware_utils_debug::DebugPublisher debug_pub(node, "/debug");

  // Publish a debug message with custom type
  float debug_data = 42.0;
  debug_pub.publish<autoware_internal_debug_msgs::msg::Float32Stamped>("example", debug_data);

  rclcpp::shutdown();
  return 0;
}
```

### Logging Processing Times with ProcessingTimePublisher

```cpp
#include <autoware_utils_debug/processing_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("processing_time_node");

  // Initialize ProcessingTimePublisher
  autoware_utils_debug::ProcessingTimePublisher processing_time_pub(node.get(), "~/debug/processing_time_ms");

  // Simulate some processing times
  std::map<std::string, double> processing_times = {
    {"node1", 0.1}, {"node2", 0.2}, {"node3", 0.3}
  };

  // Publish processing times
  processing_time_pub.publish(processing_times);

  rclcpp::shutdown();
  return 0;
}
```
