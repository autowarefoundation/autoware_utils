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
