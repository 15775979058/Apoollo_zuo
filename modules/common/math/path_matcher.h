/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#ifndef MODULES_COMMON_MATH_PATH_MATCHER_H_
#define MODULES_COMMON_MATH_PATH_MATCHER_H_

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class PathMatcher {
 public:
  PathMatcher() = delete;

  static common::PathPoint MatchToPath(
      const std::vector<common::PathPoint>& reference_line, const double x,
      const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<common::PathPoint>& reference_line, const double x,
      const double y);

  static common::PathPoint MatchToPath(
      const std::vector<common::PathPoint>& reference_line, const double s);

 private:
  static common::PathPoint FindProjectionPoint(const common::PathPoint& p0,
                                               const common::PathPoint& p1,
                                               const double x, const double y);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_COMMON_MATH_PATH_MATCHER_H_
