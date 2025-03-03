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

struct ParamLinspace
{
  double start;
  double stop;
  size_t num;
  std::vector<double> result;
};

class TestLinspace : public testing::TestWithParam<ParamLinspace>
{
};

TEST_P(TestLinspace, Normal)
{
  const auto p = GetParam();
  const auto v = autoware_utils_math::linspace(p.start, p.stop, p.num);
  EXPECT_EQ(v.size(), p.result.size());
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_DOUBLE_EQ(v.at(i), p.result.at(i));
  }
}

INSTANTIATE_TEST_CASE_P(
  TestRange, TestLinspace,
  testing::Values(
    ParamLinspace{0.0, 1.0, 11, {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}},
    ParamLinspace{1.0, 0.0, 11, {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0}},
    ParamLinspace{0.0, 1.0, 6, {0.0, 0.2, 0.4, 0.6, 0.8, 1.0}},
    ParamLinspace{1.0, 0.0, 6, {1.0, 0.8, 0.6, 0.4, 0.2, 0.0}}));
