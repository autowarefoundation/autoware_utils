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

#include "autoware_utils_system/lru_cache.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

struct ParamLruCache
{
  int size;
  std::vector<int> input;
  std::vector<int> cache;
};

class TestLruCache : public testing::TestWithParam<ParamLruCache>
{
};

TEST_P(TestLruCache, Main)
{
  const auto p = GetParam();
  autoware_utils_system::LRUCache<int, std::string> cache(p.size);
  for (const auto & v : p.input) {
    cache.put(v, std::to_string(v));
  }
  for (const auto & v : p.cache) {
    EXPECT_TRUE(cache.contains(v));
  }
}

INSTANTIATE_TEST_SUITE_P(
  TestLruCache, TestLruCache,
  testing::Values(
    ParamLruCache{3, {1, 2, 3, 4, 5, 6}, {4, 5, 6}},
    ParamLruCache{3, {1, 2, 3, 1, 4, 5}, {1, 4, 5}},
    ParamLruCache{3, {1, 2, 3, 2, 4, 5}, {2, 4, 5}},
    ParamLruCache{4, {1, 2, 3, 4, 5, 6}, {3, 4, 5, 6}},
    ParamLruCache{4, {1, 2, 3, 1, 4, 5}, {1, 3, 4, 5}},
    ParamLruCache{4, {1, 2, 3, 2, 4, 5}, {2, 3, 4, 5}}));
