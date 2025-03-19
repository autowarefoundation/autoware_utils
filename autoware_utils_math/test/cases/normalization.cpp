// Copyright 2020 TIER IV, Inc.
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

#include "autoware_utils_math/normalization.hpp"

#include <gtest/gtest.h>

#include <algorithm>

// The arguments a and b must be normalized.
double diff_radian(const double a, const double b)
{
  const auto diff = std::abs(a - b);
  return std::min(diff, 2 * autoware_utils_math::pi - diff);
}

TEST(TestNormalization, Degree)
{
  const double range_min = -180.0;
  const double range_max = +180.0;
  const double step = 30.0;
  for (double a = range_min; a < range_max; a += step) {
    for (int i = -3; i <= 3; ++i) {
      const auto r = autoware_utils_math::normalize_degree(i * 360.0 + a);
      EXPECT_DOUBLE_EQ(r, a);
    }
  }
}

TEST(TestNormalization, Radian)
{
  using autoware_utils_math::pi;
  const double range_min = -pi;
  const double range_max = +pi;
  const double step = pi / 6.0;
  for (double a = range_min; a < range_max; a += step) {
    for (int c = -3; c <= 3; ++c) {
      const auto x = a + 2 * pi * c;
      const auto r = autoware_utils_math::normalize_radian(x);
      EXPECT_NEAR(0.0, diff_radian(r, a), 2e-15);
    }
  }
}

TEST(normalization, normalize_degree)  // NOLINT for gtest
{
  using autoware_utils_math::normalize_degree;

  // -180 <= deg < 180
  {
    constexpr double eps = 0.1;
    constexpr double v_min = -180;
    constexpr double v_mid = 0;
    constexpr double v_max = 180;

    EXPECT_DOUBLE_EQ(normalize_degree(v_min - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_degree(v_min), v_min);
    EXPECT_DOUBLE_EQ(normalize_degree(v_mid), v_mid);
    EXPECT_DOUBLE_EQ(normalize_degree(v_max - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_degree(v_max), v_min);
  }

  // 0 <= deg < 360
  {
    constexpr double eps = 0.1;
    constexpr double v_min = 0;
    constexpr double v_mid = 180;
    constexpr double v_max = 360;

    EXPECT_DOUBLE_EQ(normalize_degree(v_min - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_degree(v_min, 0), v_min);
    EXPECT_DOUBLE_EQ(normalize_degree(v_mid, 0), v_mid);
    EXPECT_DOUBLE_EQ(normalize_degree(v_max - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_degree(v_max, 0), v_min);
  }
}

TEST(normalization, normalize_radian)  // NOLINT for gtest
{
  using autoware_utils_math::normalize_radian;

  // -M_PI <= deg < M_PI
  {
    constexpr double eps = 0.1;
    constexpr double v_min = -M_PI;
    constexpr double v_mid = 0;
    constexpr double v_max = M_PI;

    EXPECT_DOUBLE_EQ(normalize_radian(v_min - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_radian(v_min), v_min);
    EXPECT_DOUBLE_EQ(normalize_radian(v_mid), v_mid);
    EXPECT_DOUBLE_EQ(normalize_radian(v_max - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_radian(v_max), v_min);
  }

  // 0 <= deg < 2 * M_PI
  {
    constexpr double eps = 0.1;
    constexpr double v_min = 0;
    constexpr double v_mid = M_PI;
    constexpr double v_max = 2 * M_PI;

    EXPECT_DOUBLE_EQ(normalize_radian(v_min - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_radian(v_min, 0), v_min);
    EXPECT_DOUBLE_EQ(normalize_radian(v_mid, 0), v_mid);
    EXPECT_DOUBLE_EQ(normalize_radian(v_max - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalize_radian(v_max, 0), v_min);
  }
}
