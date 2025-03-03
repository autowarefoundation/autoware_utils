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

#include "autoware_utils_visualization/marker_helper.hpp"

#include <gtest/gtest.h>

TEST(TestMarkerHelper, CreatePosition)
{
  const auto r = autoware_utils_visualization::create_marker_position(0.1, 0.2, 0.3);
  EXPECT_DOUBLE_EQ(r.x, 0.1);
  EXPECT_DOUBLE_EQ(r.y, 0.2);
  EXPECT_DOUBLE_EQ(r.z, 0.3);
}

TEST(TestMarkerHelper, CreateOrientation)
{
  const auto r = autoware_utils_visualization::create_marker_orientation(0.1, 0.2, 0.3, 0.4);
  EXPECT_DOUBLE_EQ(r.x, 0.1);
  EXPECT_DOUBLE_EQ(r.y, 0.2);
  EXPECT_DOUBLE_EQ(r.z, 0.3);
  EXPECT_DOUBLE_EQ(r.w, 0.4);
}

TEST(TestMarkerHelper, CreateScale)
{
  const auto r = autoware_utils_visualization::create_marker_scale(0.1, 0.2, 0.3);
  EXPECT_DOUBLE_EQ(r.x, 0.1);
  EXPECT_DOUBLE_EQ(r.y, 0.2);
  EXPECT_DOUBLE_EQ(r.z, 0.3);
}

TEST(TestMarkerHelper, CreateColor)
{
  const auto r = autoware_utils_visualization::create_marker_color(0.1, 0.2, 0.3, 0.4);
  EXPECT_FLOAT_EQ(r.r, 0.1);
  EXPECT_FLOAT_EQ(r.g, 0.2);
  EXPECT_FLOAT_EQ(r.b, 0.3);
  EXPECT_FLOAT_EQ(r.a, 0.4);
}

TEST(TestMarkerHelper, CreateDefaultMarker)
{
  using visualization_msgs::msg::Marker;
  const auto stamp = rclcpp::Time(12345, 67890);
  const auto scale = autoware_utils_visualization::create_marker_scale(0.1, 0.2, 0.3);
  const auto color = autoware_utils_visualization::create_marker_color(0.1, 0.2, 0.3, 0.4);

  const auto m = autoware_utils_visualization::create_default_marker(
    "frame", stamp, "ns", 99, Marker::CUBE, scale, color);

  EXPECT_EQ(m.header.stamp.sec, 12345);
  EXPECT_EQ(m.header.stamp.nanosec, 67890);
  EXPECT_EQ(m.header.frame_id, "frame");
  EXPECT_EQ(m.ns, "ns");
  EXPECT_EQ(m.id, 99);
  EXPECT_EQ(m.action, Marker::ADD);
  EXPECT_EQ(m.type, Marker::CUBE);
  EXPECT_DOUBLE_EQ(m.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(m.scale.x, 0.1);
  EXPECT_DOUBLE_EQ(m.scale.y, 0.2);
  EXPECT_DOUBLE_EQ(m.scale.z, 0.3);
  EXPECT_FLOAT_EQ(m.color.r, 0.1);
  EXPECT_FLOAT_EQ(m.color.g, 0.2);
  EXPECT_FLOAT_EQ(m.color.b, 0.3);
  EXPECT_FLOAT_EQ(m.color.a, 0.4);
}

TEST(TestMarkerHelper, CreateDeleteMarker)
{
  using visualization_msgs::msg::Marker;
  const auto stamp = rclcpp::Time(12345, 67890);

  const auto m = autoware_utils_visualization::create_deleted_default_marker(stamp, "ns", 99);
  EXPECT_EQ(m.header.stamp.sec, 12345);
  EXPECT_EQ(m.header.stamp.nanosec, 67890);
  EXPECT_EQ(m.ns, "ns");
  EXPECT_EQ(m.id, 99);
  EXPECT_EQ(m.action, Marker::DELETE);
}

TEST(TestMarkerHelper, CreateAppendMarkerArray)
{
  visualization_msgs::msg::MarkerArray array1;
  visualization_msgs::msg::MarkerArray array2;
  array1.markers.resize(2);
  array1.markers[0].id = 10;
  array1.markers[1].id = 11;
  array2.markers.resize(3);
  array2.markers[0].id = 20;
  array2.markers[1].id = 21;
  array2.markers[2].id = 22;

  autoware_utils_visualization::append_marker_array(array2, &array1, std::nullopt);
  EXPECT_EQ(array1.markers.size(), 5);
  EXPECT_EQ(array1.markers[0].id, 10);
  EXPECT_EQ(array1.markers[1].id, 11);
  EXPECT_EQ(array1.markers[2].id, 20);
  EXPECT_EQ(array1.markers[3].id, 21);
  EXPECT_EQ(array1.markers[4].id, 22);
}
