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

#ifndef AUTOWARE_UTILS_MATH__NORMALIZATION_HPP_
#define AUTOWARE_UTILS_MATH__NORMALIZATION_HPP_

#include "autoware_utils_math/constants.hpp"

#include <cmath>

namespace autoware_utils_math
{

namespace detail
{
constexpr double kDoublePi = 2.0 * pi;
}  // namespace detail

/*
  * @brief Wraps an angle to the range [-pi, pi].
  *
  * This function takes an angle in radians and wraps it to the range [-pi, pi]. It uses the
  * remainder function for efficient wrapping and handles both positive and negative angles.
  *
  * @tparam T A floating-point type (e.g., float, double).
  * @param angle The input angle in radians to be wrapped.
  * @return The wrapped angle in the range [-pi, pi].
*/
template <typename T>
constexpr T wrap_angle(T angle) noexcept
{
  static_assert(std::is_floating_point<T>::value, "wrap_angle requires floating point type");
  
  // Use remainder for efficient wrapping to [-pi, pi]
  T wrapped = std::fmod(angle + T(pi), T(detail::kDoublePi));
  if (wrapped < T{}) {
    wrapped += T(detail::kDoublePi);
  }
  return wrapped - T(pi);
}

inline double normalize_degree(const double deg, const double min_deg = -180)
{
  const auto max_deg = min_deg + 360.0;

  const auto value = std::fmod(deg, 360.0);
  if (min_deg <= value && value < max_deg) {
    return value;
  }

  return value - std::copysign(360.0, value);
}

inline double normalize_radian(const double rad, const double min_rad = -pi)
{
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}

}  // namespace autoware_utils_math

#endif  // AUTOWARE_UTILS_MATH__NORMALIZATION_HPP_
