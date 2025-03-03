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

#include "autoware_utils_math/trigonometry.hpp"

#include "autoware_utils_math/constants.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

struct ParamSinCos
{
  float radian;
  float sin;
  float cos;
};

std::vector<ParamSinCos> make_test_cases(double min, double max, size_t size)
{
  const double step = (max - min) / size;
  std::vector<ParamSinCos> params;
  for (size_t i = 0; i < size; ++i) {
    ParamSinCos p;
    p.radian = min + i * step;
    p.sin = std::sin(p.radian);
    p.cos = std::cos(p.radian);
    params.push_back(p);
  }
  return params;
}

std::vector<ParamSinCos> make_normalized_cases()
{
  using autoware_utils_math::pi;
  return make_test_cases(-pi * 1, +pi * 1, 1000);
}

std::vector<ParamSinCos> make_periodic_cases()
{
  using autoware_utils_math::pi;
  return make_test_cases(-pi * 3, +pi * 3, 100);
}

TEST(TestTrigonometry, SinCos)
{
  std::vector<ParamSinCos> params;
  {
    const auto cases1 = make_normalized_cases();
    const auto cases2 = make_periodic_cases();
    params.insert(params.end(), cases1.begin(), cases1.end());
    params.insert(params.end(), cases2.begin(), cases2.end());
  }

  for (const auto & p : params) {
    const auto sin1 = autoware_utils_math::sin(p.radian);
    const auto cos1 = autoware_utils_math::cos(p.radian);
    const auto [sin2, cos2] = autoware_utils_math::sin_and_cos(p.radian);
    constexpr double eps = 3e-5;
    EXPECT_NEAR(sin1, p.sin, eps);
    EXPECT_NEAR(cos1, p.cos, eps);
    EXPECT_NEAR(sin2, p.sin, eps);
    EXPECT_NEAR(cos2, p.cos, eps);
  }
}

TEST(TestTrigonometry, Atan2)
{
  for (const auto & p : make_normalized_cases()) {
    const auto r0 = 0 <= p.radian ? p.radian : p.radian + 2 * autoware_utils_math::pi;
    const auto r1 = autoware_utils_math::opencv_fast_atan2(p.sin * 0.5, p.cos * 0.5);
    const auto r2 = autoware_utils_math::opencv_fast_atan2(p.sin * 1.0, p.cos * 1.0);
    const auto r3 = autoware_utils_math::opencv_fast_atan2(p.sin * 1.5, p.cos * 1.5);
    constexpr double eps = 2e-4;
    EXPECT_NEAR(r1, r0, eps);
    EXPECT_NEAR(r2, r0, eps);
    EXPECT_NEAR(r3, r0, eps);
  }
}
