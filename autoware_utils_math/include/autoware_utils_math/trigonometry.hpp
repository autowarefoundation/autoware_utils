// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE_UTILS_MATH__TRIGONOMETRY_HPP_
#define AUTOWARE_UTILS_MATH__TRIGONOMETRY_HPP_

#include <utility>

namespace autoware_utils_math
{

float sin(float radian);

float cos(float radian);

std::pair<float, float> sin_and_cos(float radian);

float opencv_fast_atan2(float dy, float dx);

}  // namespace autoware_utils_math

#endif  // AUTOWARE_UTILS_MATH__TRIGONOMETRY_HPP_
