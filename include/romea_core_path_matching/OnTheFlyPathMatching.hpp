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

#ifndef ROMEA_CORE_PATH_MATCHING__ONTHEFLYPATHMATCHING_HPP_
#define ROMEA_CORE_PATH_MATCHING__ONTHEFLYPATHMATCHING_HPP_

// std
#include <optional>
#include <string>

// romea
#include "romea_core_path/PathMatching2D.hpp"
#include "romea_core_path_matching/OnTheFlyPathMatchingDiagnostic.hpp"

namespace romea
{
namespace core
{

class OnTheFlyPathMatching
{
public:
  OnTheFlyPathMatching(
    const double & maximalResearchRadius,
    const double & interpolationWindowLength,
    const double & minimalDistanceBetweenTwoPoints,
    const double & minimalVehicleSpeedToInsertPoint);

  const PathSection2D & getSection() const;

  const PathCurve2D & getCurve(const size_t & curve_index) const;

  bool updatePath(
    const Duration & stamp,
    const Pose2D & leaderVehiclePose,
    const Twist2D & leaderVehicleTwist);

  std::optional<PathMatchedPoint2D> match(
    const Duration & stamp,
    const Pose2D & followerVehiclePose,
    const Twist2D & followerVehicleTwist,
    const double & predictionTimeHorizon = 0.0);

  DiagnosticReport getReport(const Duration & stamp);

  void reset();

private:
  void tryMatchOnFullPath_(
    const Pose2D & followerVehiclePose,
    const Twist2D & followerVehicleTwist,
    const double & predictionTimeHorizon);

  void tryMatchOnFirstPoint_(
    const Pose2D & followerVehiclePose,
    const Twist2D & followerVehicleTwist,
    const double & predictionTimeHorizon);

  double travelledDistance_(const Pose2D & leaderVehiclePose);

  double leaderVehicleSpeed_(const Twist2D & leaderVehicleTwist);

protected:
  double maximalResearchRadius_;
  double interpolationWindowLength_;
  double minimalDistanceBetweenTwoPoints_;
  double minimalVehicleSpeedToInsertPoint_;

  PathSection2D pathSection_;
  std::optional<PathMatchedPoint2D> matchedPoint_;
  OnTheFlyPathMatchingDiagnostic diagnostics_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_MATCHING__ONTHEFLYPATHMATCHING_HPP_
