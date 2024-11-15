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

// std
#include <optional>

// romea
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_path/PathSectionMatching2D.hpp"
#include "romea_core_path_matching/OnTheFlyPathMatching.hpp"

namespace
{
romea::core::PathMatchedPoint2D fakeMatchedPoint(
  const romea::core::Pose2D & vehiclePose,
  const Eigen::Vector2d & directionToReach)
{
  romea::core::PathMatchedPoint2D fakeMatchedPoint;

  fakeMatchedPoint.pathPosture.course = std::atan2(
    directionToReach.y(), directionToReach.x());
  fakeMatchedPoint.pathPosture.position = vehiclePose.position;
  fakeMatchedPoint.pathPosture.curvature = 0;
  fakeMatchedPoint.pathPosture.dotCurvature = 0;

  fakeMatchedPoint.frenetPose.lateralDeviation = 0;
  fakeMatchedPoint.frenetPose.courseDeviation = romea::core::betweenMinusPiAndPi(
    vehiclePose.yaw - fakeMatchedPoint.pathPosture.course);
  fakeMatchedPoint.frenetPose.curvilinearAbscissa = -directionToReach.norm();

  return fakeMatchedPoint;
}
}  // namespace


namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
OnTheFlyPathMatching::OnTheFlyPathMatching(
  const double & maximalResearchRadius,
  const double & interpolationWindowLength,
  const double & minimalDistanceBetweenTwoPoints,
  const double & minimalVehicleSpeedToInsertPoint)
: maximalResearchRadius_(maximalResearchRadius),
  interpolationWindowLength_(interpolationWindowLength),
  minimalDistanceBetweenTwoPoints_(minimalDistanceBetweenTwoPoints),
  minimalVehicleSpeedToInsertPoint_(minimalVehicleSpeedToInsertPoint),
  pathSection_(interpolationWindowLength),
  matchedPoint_()
{
}

//-----------------------------------------------------------------------------
const PathCurve2D & OnTheFlyPathMatching::getCurve(
  const size_t & curve_index) const
{
  return pathSection_.getCurve(curve_index);
}

//-----------------------------------------------------------------------------
bool OnTheFlyPathMatching::updatePath(
  const Duration & stamp,
  const Pose2D & leaderVehiclePose,
  const Twist2D & leaderVehicleTwist)
{
  diagnostics_.updateLeaderLocalisationRate(stamp);
  if (travelledDistance_(leaderVehiclePose) > minimalDistanceBetweenTwoPoints_ &&
    leaderVehicleSpeed_(leaderVehicleTwist) > minimalVehicleSpeedToInsertPoint_)
  {
    pathSection_.addWayPoint(PathWayPoint2D(leaderVehiclePose.position));
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
std::optional<PathMatchedPoint2D> OnTheFlyPathMatching::match(
  const Duration & stamp,
  const core::Pose2D & vehiclePose,
  const core::Twist2D & vehicleTwist,
  const double & predictionTimeHorizon)
{
  diagnostics_.updateFollowerLocalisationRate(stamp);

  if (pathSection_.getLength() > 2) {
    tryMatchOnFullPath_(vehiclePose, vehicleTwist, predictionTimeHorizon);

    if (!matchedPoint_.has_value()) {
      tryMatchOnFirstPoint_(vehiclePose, vehicleTwist, predictionTimeHorizon);
    }
  }
  diagnostics_.updatePathMatchingStatus(matchedPoint_.has_value());
  return matchedPoint_;
}

//-----------------------------------------------------------------------------
DiagnosticReport OnTheFlyPathMatching::getReport(const Duration & stamp)
{
  return diagnostics_.makeReport(stamp);
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::reset()
{
  matchedPoint_.reset();
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::tryMatchOnFullPath_(
  const core::Pose2D & followerVehiclePose,
  const core::Twist2D & followerVehicleTwist,
  const double & predictionTimeHorizon)
{
  double followerVehicleSpeed = followerVehicleTwist.linearSpeeds.x();

  if (matchedPoint_.has_value()) {
    matchedPoint_ = romea::core::match(
      pathSection_,
      followerVehiclePose,
      followerVehicleSpeed,
      *matchedPoint_,
      10,
      predictionTimeHorizon,
      maximalResearchRadius_);

  } else {
    matchedPoint_ = romea::core::match(
      pathSection_,
      followerVehiclePose,
      followerVehicleSpeed,
      predictionTimeHorizon,
      maximalResearchRadius_);
  }
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::tryMatchOnFirstPoint_(
  const core::Pose2D & followerVehiclePose,
  const core::Twist2D & /*followerVehicleTwist*/,
  const double & /*predictionTimeHorizon*/)
{
  Eigen::Vector2d firstPathPosition(pathSection_.getX()[0], pathSection_.getX()[1]);
  Eigen::Vector2d directionToReach = followerVehiclePose.position - firstPathPosition;
  if ((directionToReach).norm() < maximalResearchRadius_) {
    matchedPoint_ = fakeMatchedPoint(followerVehiclePose, directionToReach);
  }
}

//-----------------------------------------------------------------------------
double OnTheFlyPathMatching::travelledDistance_(const core::Pose2D & leaderVehiclePose)
{
  const Eigen::Vector2d & leaderPosition = leaderVehiclePose.position;
  static Eigen::Vector2d previousLeaderPosition = leaderPosition;
  double distance = (leaderPosition - previousLeaderPosition).norm();
  previousLeaderPosition = leaderPosition;
  return distance;
}

//-----------------------------------------------------------------------------
double OnTheFlyPathMatching::leaderVehicleSpeed_(const core::Twist2D & leaderVehicleTwist)
{
  return leaderVehicleTwist.linearSpeeds.norm();
}

}  // namespace core
}  // namespace romea
