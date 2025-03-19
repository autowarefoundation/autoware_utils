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

TEST(unit_conversion, deg2rad)  // NOLINT for gtest
{
  using autoware_utils_math::deg2rad;
  using autoware_utils_math::pi;

  EXPECT_DOUBLE_EQ(deg2rad(-720), -4 * pi);
  EXPECT_DOUBLE_EQ(deg2rad(0), 0);
  EXPECT_DOUBLE_EQ(deg2rad(30), pi / 6);
  EXPECT_DOUBLE_EQ(deg2rad(60), pi / 3);
  EXPECT_DOUBLE_EQ(deg2rad(90), pi / 2);
  EXPECT_DOUBLE_EQ(deg2rad(180), pi);
  EXPECT_DOUBLE_EQ(deg2rad(360), 2 * pi);
}

TEST(unit_conversion, rad2deg)  // NOLINT for gtest
{
  using autoware_utils_math::pi;
  using autoware_utils_math::rad2deg;

  EXPECT_DOUBLE_EQ(rad2deg(-4 * pi), -720);
  EXPECT_DOUBLE_EQ(rad2deg(0), 0);
  EXPECT_DOUBLE_EQ(rad2deg(pi / 6), 30);
  EXPECT_DOUBLE_EQ(rad2deg(pi / 3), 60);
  EXPECT_DOUBLE_EQ(rad2deg(pi / 2), 90);
  EXPECT_DOUBLE_EQ(rad2deg(pi), 180);
  EXPECT_DOUBLE_EQ(rad2deg(2 * pi), 360);
}

TEST(unit_conversion, kmph2mps)  // NOLINT for gtest
{
  using autoware_utils_math::kmph2mps;

  EXPECT_DOUBLE_EQ(kmph2mps(0), 0);
  EXPECT_DOUBLE_EQ(kmph2mps(36), 10);
  EXPECT_DOUBLE_EQ(kmph2mps(72), 20);
  EXPECT_DOUBLE_EQ(kmph2mps(180), 50);
}

TEST(unit_conversion, mps2kmph)  // NOLINT for gtest
{
  using autoware_utils_math::mps2kmph;

  EXPECT_DOUBLE_EQ(mps2kmph(0), 0);
  EXPECT_DOUBLE_EQ(mps2kmph(10), 36);
  EXPECT_DOUBLE_EQ(mps2kmph(20), 72);
  EXPECT_DOUBLE_EQ(mps2kmph(50), 180);
}
