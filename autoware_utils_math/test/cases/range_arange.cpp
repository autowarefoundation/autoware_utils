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

#include "autoware_utils_math/range.hpp"

#include <gtest/gtest.h>

#include <vector>

struct ParamArange
{
  double start;
  double stop;
  double step;
  std::vector<double> result;
};

class TestArange : public testing::TestWithParam<ParamArange>
{
};

TEST_P(TestArange, Normal)
{
  const auto p = GetParam();
  const auto v = autoware_utils_math::arange(p.start, p.stop, p.step);
  EXPECT_EQ(v.size(), p.result.size());
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_DOUBLE_EQ(v.at(i), p.result.at(i));
  }
}

TEST_P(TestArange, StepZero)
{
  const auto p = GetParam();
  EXPECT_THROW(autoware_utils_math::arange(p.start, p.stop, 0.0), std::invalid_argument);
}

TEST_P(TestArange, StepInverse)
{
  const auto p = GetParam();
  EXPECT_THROW(autoware_utils_math::arange(p.start, p.stop, -p.step), std::invalid_argument);
}

INSTANTIATE_TEST_CASE_P(
  TestRange, TestArange,
  testing::Values(
    ParamArange{0.0, 1.0, +0.1, {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}},
    ParamArange{0.0, 1.0, +0.2, {0.0, 0.2, 0.4, 0.6, 0.8}},
    ParamArange{1.0, 0.0, -0.1, {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1}},
    ParamArange{1.0, 0.0, -0.2, {1.0, 0.8, 0.6, 0.4, 0.2}}));
