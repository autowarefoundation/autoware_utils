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

#include "autoware_utils_math/unit_conversion.hpp"

#include <gtest/gtest.h>

#include <vector>

struct ParamAngle
{
  double radian;
  double degree;
};

struct ParamSpeed
{
  double mps;
  double kmph;
};

std::vector<ParamAngle> make_angle_cases()
{
  using autoware_utils_math::pi;
  const double range_min = -360 * 3;
  const double range_max = +360 * 3;
  const double step = 10;

  std::vector<ParamAngle> params;
  for (double i = range_min; i <= range_max; i += step) {
    params.push_back({i * pi / 180.0, i});
  }
  return params;
}

std::vector<ParamSpeed> make_speed_cases()
{
  const double range_min = -70;
  const double range_max = +70;
  const double step = 7;

  std::vector<ParamSpeed> params;
  for (double i = range_min; i <= range_max; i += step) {
    params.push_back({i / 3.6, i});
  }
  return params;
}

TEST(TestUnitConversion, DegToRad)
{
  for (const auto & p : make_angle_cases()) {
    EXPECT_DOUBLE_EQ(p.radian, autoware_utils_math::deg2rad(p.degree));
  }
}

TEST(TestUnitConversion, RadToDeg)
{
  for (const auto & p : make_angle_cases()) {
    EXPECT_DOUBLE_EQ(p.degree, autoware_utils_math::rad2deg(p.radian));
  }
}

TEST(TestUnitConversion, KmphToMps)
{
  for (const auto & p : make_speed_cases()) {
    EXPECT_DOUBLE_EQ(p.mps, autoware_utils_math::kmph2mps(p.kmph));
  }
}

TEST(TestUnitConversion, MpsToKmph)
{
  for (const auto & p : make_speed_cases()) {
    EXPECT_DOUBLE_EQ(p.kmph, autoware_utils_math::mps2kmph(p.mps));
  }
}
