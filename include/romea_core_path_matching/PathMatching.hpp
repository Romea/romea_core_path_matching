// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef ROMEA_CORE_PATH_MATCHING__PATHMATCHING_HPP_
#define ROMEA_CORE_PATH_MATCHING__PATHMATCHING_HPP_

// std
#include <optional>
#include <string>
#include <vector>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_path/PathMatching2D.hpp"
#include "romea_core_path_matching/PathMatchingDiagnostic.hpp"

namespace romea
{
namespace core
{

class PathMatching
{
public:
  PathMatching(
    const std::string & pathFilename,
    const double & maximalResearchRadius,
    const double & interpolationWindowLength);

  const Path2D & getPath() const;

  std::vector<PathMatchedPoint2D> match(
    const Duration & stamp,
    const Pose2D & vehiclePose,
    const Twist2D & vehicleTwist,
    const double & predictionTimeHorizon = 0.0);

  DiagnosticReport getReport(const Duration & stamp);

  void reset();

protected:
  double maximalResearchRadius_;

  Path2D path_;
  std::vector<PathMatchedPoint2D> matchedPoints_;

  PathMatchingDiagnostic diagnostics_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_MATCHING__PATHMATCHING_HPP_
