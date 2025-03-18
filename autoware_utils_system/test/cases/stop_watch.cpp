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

#include "autoware_utils_system/stop_watch.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

TEST(TestStopWatch, Default)
{
  autoware_utils_system::StopWatch sw;
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_NEAR(sw.toc(), 0.01, 0.001);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_NEAR(sw.toc(), 0.02, 0.001);
}

TEST(TestStopWatch, Named)
{
  autoware_utils_system::StopWatch sw;
  sw.tic("foo");
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_NEAR(sw.toc("foo"), 0.01, 0.001);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_NEAR(sw.toc("foo"), 0.02, 0.001);
}

TEST(TestStopWatch, Mixed)
{
  autoware_utils_system::StopWatch sw;
  sw.tic("foo");
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  sw.tic("bar");
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_NEAR(sw.toc("foo"), 0.02, 0.001);
  EXPECT_NEAR(sw.toc("bar"), 0.01, 0.001);
}

TEST(TestStopWatch, Abort)
{
  autoware_utils_system::StopWatch sw;
  EXPECT_THROW(sw.toc("baz"), std::out_of_range);
}
