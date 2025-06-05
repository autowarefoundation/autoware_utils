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

#include "autoware_utils_math/tdigest.hpp"

#include <gtest/gtest.h>

TEST(tdigest, count)
{
  autoware_utils_math::tdigest<double> digest(100);
  for (double i = 1; i <= 1000; i += 1.0) {
    digest.insert(i);
  }
  digest.merge();

  EXPECT_EQ(digest.size(), 1000);
}

TEST(tdigest, insert)
{
  autoware_utils_math::tdigest<double> digest(100);

  // insert normally
  for (double i = 1; i <= 1000; i += 1.0) {
    digest.insert(i);
  }
  digest.merge();

  EXPECT_EQ(digest.size(), 1000);
  EXPECT_NEAR(digest.quantile(50.0), 500.0, 20.0);

  // insert with weight
  for (double i = 1; i <= 1000; i += 10.0) {
    digest.insert(i, 10);
  }
  digest.merge();

  EXPECT_EQ(digest.size(), 2000);
  EXPECT_NEAR(digest.quantile(50.0), 500.0, 20.0);

  // insert with another digest
  autoware_utils_math::tdigest<double> digest2(100);
  digest2.insert(500, 1000);
  digest2.merge();

  digest.insert(digest2);
  digest.merge();

  EXPECT_EQ(digest.size(), 3000);
  EXPECT_NEAR(digest.quantile(99.0), 990.0, 20.0);
}

TEST(tdigest, statistics)
{
  static constexpr double max_value = 1000.0;
  static constexpr double epsilon = 0.02;  // 2% error margin

  autoware_utils_math::tdigest<double> digest(100);
  for (double i = 1; i <= 1000; i += 1.0) {
    digest.insert(i);
  }
  digest.merge();

  EXPECT_NEAR(digest.quantile(0.0), 1.0, epsilon * max_value);
  EXPECT_NEAR(digest.quantile(50.0), max_value * 0.5, epsilon * max_value);
  EXPECT_NEAR(digest.quantile(100.0), max_value, epsilon * max_value);
  EXPECT_NEAR(digest.quantile(99.0), max_value * 0.99, epsilon * max_value);
  EXPECT_NEAR(digest.quantile(90.0), max_value * 0.9, epsilon * max_value);
  EXPECT_NEAR(digest.cumulative_distribution(max_value * 0.6), 0.6, epsilon);
  EXPECT_EQ(digest.min(), 1);
  EXPECT_EQ(digest.size(), 1000);
}
