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

#include "autoware_utils_math/accumulator.hpp"

#include <gtest/gtest.h>

TEST(TestAccumulator, Case1)
{
  autoware_utils_math::Accumulator a;
  for (int i = 0; i < 10; ++i) {
    a.add(i);
  }
  EXPECT_DOUBLE_EQ(a.count(), 10);
  EXPECT_DOUBLE_EQ(a.min(), 0.0);
  EXPECT_DOUBLE_EQ(a.max(), 9.0);
  EXPECT_DOUBLE_EQ(a.mean(), 4.5);
}

TEST(TestAccumulator, Case2)
{
  autoware_utils_math::Accumulator a;
  for (int i = 0; i < 10; ++i) {
    a.add(i + 1);
  }
  EXPECT_DOUBLE_EQ(a.count(), 10);
  EXPECT_DOUBLE_EQ(a.min(), 1.0);
  EXPECT_DOUBLE_EQ(a.max(), 10.0);
  EXPECT_DOUBLE_EQ(a.mean(), 5.5);
}
