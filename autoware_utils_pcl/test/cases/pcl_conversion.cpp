// Copyright 2025 Tier IV, Inc.
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

#include "autoware_utils_pcl/pcl_conversion.hpp"

#include <Eigen/Core>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{
// Build a tightly-packed PointCloud2 with three FLOAT32 fields x, y, z at offsets 0, 4, 8.
// The implementation reads x, y, z as the first three contiguous floats of each point, so a
// 12-byte point_step (no padding) is a valid input layout.
sensor_msgs::msg::PointCloud2 make_point_cloud2(
  const std::vector<std::array<float, 3>> & points, std::uint32_t width, std::uint32_t height,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  if (points.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
    ADD_FAILURE() << "make_point_cloud2: points.size() must equal width * height";
    return cloud;
  }
  cloud.header.frame_id = frame_id;
  cloud.height = height;
  cloud.width = width;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  const char * names[3] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = i * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }

  cloud.point_step = 3 * sizeof(float);  // 12 bytes, tightly packed
  cloud.row_step = cloud.point_step * width;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * height);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t byte_offset = i * cloud.point_step;
    std::memcpy(&cloud.data[byte_offset + 0 * sizeof(float)], &points[i][0], sizeof(float));
    std::memcpy(&cloud.data[byte_offset + 1 * sizeof(float)], &points[i][1], sizeof(float));
    std::memcpy(&cloud.data[byte_offset + 2 * sizeof(float)], &points[i][2], sizeof(float));
  }

  return cloud;
}
}  // namespace

// Sanity-check the assumption baked into the implementation: pcl::PointXYZ has a 16-byte stride
// (x, y, z floats plus 4 bytes of padding) and writes x, y, z as the first three floats.
TEST(pcl_conversion, point_xyz_stride)
{
  EXPECT_EQ(sizeof(pcl::PointXYZ), 16u);
}

TEST(pcl_conversion, identity_unorganized)
{
  const std::vector<std::array<float, 3>> input = {
    {1.0f, 2.0f, 3.0f}, {-4.5f, 0.25f, 7.75f}, {10.0f, -20.0f, 30.0f}};
  const std::uint32_t n = static_cast<std::uint32_t>(input.size());

  const auto cloud = make_point_cloud2(input, n, 1, "identity_frame");

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  autoware_utils_pcl::transform_point_cloud_from_ros_msg(cloud, pcl_cloud, transform);

  EXPECT_EQ(pcl_cloud.size(), static_cast<std::size_t>(n));
  EXPECT_EQ(pcl_cloud.width, n);
  EXPECT_EQ(pcl_cloud.height, 1u);
  EXPECT_TRUE(pcl_cloud.is_dense);
  EXPECT_EQ(pcl_cloud.header.frame_id, "identity_frame");

  for (std::uint32_t i = 0; i < n; ++i) {
    EXPECT_FLOAT_EQ(pcl_cloud.points[i].x, input[i][0]);
    EXPECT_FLOAT_EQ(pcl_cloud.points[i].y, input[i][1]);
    EXPECT_FLOAT_EQ(pcl_cloud.points[i].z, input[i][2]);
  }
}

TEST(pcl_conversion, known_affine_unorganized)
{
  // Two input points.
  const std::vector<std::array<float, 3>> input = {{1.0f, 0.0f, 5.0f}, {2.0f, 3.0f, -1.0f}};
  const std::uint32_t n = static_cast<std::uint32_t>(input.size());

  const auto cloud = make_point_cloud2(input, n, 1, "affine_frame");

  // 90 degree rotation about Z: (x, y, z) -> (-y, x, z), plus translation (10, 20, 30).
  //   col-major Eigen, but we fill via comma-initializer which is row-major.
  Eigen::Matrix4f transform;
  transform << 0.0f, -1.0f, 0.0f, 10.0f,  //
    1.0f, 0.0f, 0.0f, 20.0f,              //
    0.0f, 0.0f, 1.0f, 30.0f,              //
    0.0f, 0.0f, 0.0f, 1.0f;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  autoware_utils_pcl::transform_point_cloud_from_ros_msg(cloud, pcl_cloud, transform);

  ASSERT_EQ(pcl_cloud.size(), static_cast<std::size_t>(n));
  EXPECT_EQ(pcl_cloud.header.frame_id, "affine_frame");

  // Hand-computed expected outputs:
  // point 0: x' = -0 + 10 = 10, y' = 1 + 20 = 21, z' = 5 + 30 = 35
  EXPECT_NEAR(pcl_cloud.points[0].x, 10.0f, 1e-4);
  EXPECT_NEAR(pcl_cloud.points[0].y, 21.0f, 1e-4);
  EXPECT_NEAR(pcl_cloud.points[0].z, 35.0f, 1e-4);

  // point 1: x' = -3 + 10 = 7, y' = 2 + 20 = 22, z' = -1 + 30 = 29
  EXPECT_NEAR(pcl_cloud.points[1].x, 7.0f, 1e-4);
  EXPECT_NEAR(pcl_cloud.points[1].y, 22.0f, 1e-4);
  EXPECT_NEAR(pcl_cloud.points[1].z, 29.0f, 1e-4);
}

TEST(pcl_conversion, organized_height_greater_than_one)
{
  // 2x2 organized cloud (width = 2, height = 2). Row-major order in data.
  const std::vector<std::array<float, 3>> input = {
    {1.0f, 1.0f, 1.0f},   // row 0, col 0
    {2.0f, 0.0f, -1.0f},  // row 0, col 1
    {-3.0f, 4.0f, 0.5f},  // row 1, col 0
    {0.0f, 0.0f, 9.0f}};  // row 1, col 1
  const std::uint32_t width = 2;
  const std::uint32_t height = 2;

  const auto cloud = make_point_cloud2(input, width, height, "organized_frame");

  // 90 degree rotation about Z plus translation (10, 20, 30): (x, y, z) -> (-y + 10, x + 20, z +
  // 30).
  Eigen::Matrix4f transform;
  transform << 0.0f, -1.0f, 0.0f, 10.0f,  //
    1.0f, 0.0f, 0.0f, 20.0f,              //
    0.0f, 0.0f, 1.0f, 30.0f,              //
    0.0f, 0.0f, 0.0f, 1.0f;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  autoware_utils_pcl::transform_point_cloud_from_ros_msg(cloud, pcl_cloud, transform);

  EXPECT_EQ(pcl_cloud.width, width);
  EXPECT_EQ(pcl_cloud.height, height);
  EXPECT_EQ(pcl_cloud.size(), static_cast<std::size_t>(width) * height);
  EXPECT_EQ(pcl_cloud.header.frame_id, "organized_frame");

  for (std::size_t i = 0; i < input.size(); ++i) {
    const float expected_x = -input[i][1] + 10.0f;
    const float expected_y = input[i][0] + 20.0f;
    const float expected_z = input[i][2] + 30.0f;
    EXPECT_NEAR(pcl_cloud.points[i].x, expected_x, 1e-4) << "point " << i;
    EXPECT_NEAR(pcl_cloud.points[i].y, expected_y, 1e-4) << "point " << i;
    EXPECT_NEAR(pcl_cloud.points[i].z, expected_z, 1e-4) << "point " << i;
  }
}

TEST(pcl_conversion, empty_cloud)
{
  const auto cloud = make_point_cloud2({}, 0, 0, "empty");
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  const Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  EXPECT_NO_FATAL_FAILURE(
    autoware_utils_pcl::transform_point_cloud_from_ros_msg(cloud, pcl_cloud, transform));
  EXPECT_EQ(pcl_cloud.size(), 0u);
  EXPECT_EQ(pcl_cloud.width, 0u);
  EXPECT_EQ(pcl_cloud.height, 0u);
}
