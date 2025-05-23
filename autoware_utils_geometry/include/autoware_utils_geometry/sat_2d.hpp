// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE_UTILS_GEOMETRY__SAT_2D_HPP_
#define AUTOWARE_UTILS_GEOMETRY__SAT_2D_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

namespace autoware_utils_geometry::sat
{
/**
 * @brief Check if 2 convex polygons intersect using the SAT algorithm
 * @details faster than boost::geometry::overlap() but speed decline sharply as vertices increase
 */
bool intersects(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2);

}  // namespace autoware_utils_geometry::sat

#endif  // AUTOWARE_UTILS_GEOMETRY__SAT_2D_HPP_
