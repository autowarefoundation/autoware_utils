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

// get() on a present key returns the stored value.
TEST(LruCache, GetHitReturnsValue)
{
  autoware_utils_system::LRUCache<int, std::string> cache(3);
  cache.put(1, "one");
  const auto value = cache.get(1);
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, "one");
}

// get() on a missing key returns std::nullopt.
TEST(LruCache, GetMissReturnsNullopt)
{
  autoware_utils_system::LRUCache<int, std::string> cache(3);
  cache.put(1, "one");
  EXPECT_FALSE(cache.get(99).has_value());
}

// get() refreshes recency: the accessed key must survive eviction (splice-to-front).
TEST(LruCache, GetRefreshesRecency)
{
  autoware_utils_system::LRUCache<int, std::string> cache(3);
  cache.put(1, "one");
  cache.put(2, "two");
  cache.put(3, "three");
  // Access key 1 so it becomes most-recently-used; the least-recently-used is now 2.
  ASSERT_TRUE(cache.get(1).has_value());
  // Exceeding capacity must evict 2, not 1.
  cache.put(4, "four");
  EXPECT_TRUE(cache.contains(1));
  EXPECT_FALSE(cache.contains(2));
  EXPECT_TRUE(cache.contains(3));
  EXPECT_TRUE(cache.contains(4));
}

// put() on an existing key overwrites the value and refreshes recency.
TEST(LruCache, PutExistingKeyOverwritesAndRefreshes)
{
  autoware_utils_system::LRUCache<int, std::string> cache(2);
  cache.put(1, "a");
  cache.put(2, "b");
  // Re-put key 1: value becomes "c" and 1 becomes most-recently-used, so 2 is now LRU.
  cache.put(1, "c");
  cache.put(3, "d");  // exceeds capacity -> evicts 2
  const auto value = cache.get(1);
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, "c");
  EXPECT_FALSE(cache.contains(2));
  EXPECT_TRUE(cache.contains(3));
}

// clear() empties the cache.
TEST(LruCache, ClearEmptiesCache)
{
  autoware_utils_system::LRUCache<int, std::string> cache(3);
  cache.put(1, "one");
  cache.put(2, "two");
  cache.clear();
  EXPECT_EQ(cache.size(), 0u);
  EXPECT_TRUE(cache.empty());
  EXPECT_FALSE(cache.contains(1));
  EXPECT_FALSE(cache.get(1).has_value());
}

// size()/empty()/capacity() report consistent values and size never exceeds capacity.
TEST(LruCache, SizeEmptyCapacity)
{
  autoware_utils_system::LRUCache<int, std::string> cache(3);
  EXPECT_EQ(cache.capacity(), 3u);
  EXPECT_TRUE(cache.empty());
  EXPECT_EQ(cache.size(), 0u);

  cache.put(1, "one");
  cache.put(2, "two");
  EXPECT_EQ(cache.size(), 2u);
  EXPECT_FALSE(cache.empty());

  // Overflow the capacity; size must cap at the configured capacity.
  cache.put(3, "three");
  cache.put(4, "four");
  EXPECT_EQ(cache.size(), 3u);
  EXPECT_EQ(cache.capacity(), 3u);
}
