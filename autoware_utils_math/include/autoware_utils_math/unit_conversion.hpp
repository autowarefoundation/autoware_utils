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

#ifndef AUTOWARE_UTILS_MATH__UNIT_CONVERSION_HPP_
#define AUTOWARE_UTILS_MATH__UNIT_CONVERSION_HPP_

#include "autoware_utils_math/constants.hpp"

namespace autoware_utils_math
{
constexpr double deg2rad(const double deg)
{
  return deg * pi / 180.0;
}

constexpr double rad2deg(const double rad)
{
  return rad * 180.0 / pi;
}

constexpr double kmph2mps(const double kmph)
{
  return kmph * 1000.0 / 3600.0;
}

constexpr double mps2kmph(const double mps)
{
  return mps * 3600.0 / 1000.0;
}
}  // namespace autoware_utils_math

#endif  // AUTOWARE_UTILS_MATH__UNIT_CONVERSION_HPP_
