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
#include <string>

// romea
#include "romea_core_path/PathFile.hpp"
#include "romea_core_path/PathMatching2D.hpp"
#include "romea_core_path_matching/PathMatching.hpp"

namespace
{
romea::core::Path2D create_path(
  const std::string & pathFilename,
  const double & interpolationWindowLength)
{
  romea::core::PathFile pathFile(pathFilename);
  return romea::core::Path2D(
    pathFile.getWayPoints(),
    interpolationWindowLength,
    pathFile.getAnnotations());
}
}  // namespace

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathMatching::PathMatching(
  const std::string & pathFilename,
  const double & predictionTimeHorizon,
  const double & maximalResearchRadius,
  const double & interpolationWindowLength)
: predictionTimeHorizon_(predictionTimeHorizon),
  maximalResearchRadius_(maximalResearchRadius),
  interpolationWindowLength_(interpolationWindowLength),
  path_(create_path(pathFilename, interpolationWindowLength)),
  matchedPoints_(),
  trackedMatchedPointIndex_(0),
  diagnostics_(pathFilename)
{
}

//-----------------------------------------------------------------------------
const Path2D PathMatching::getPath() const
{
  return path_;
}

//-----------------------------------------------------------------------------
std::optional<PathMatchedPoint2D> PathMatching::match(
  const Duration & stamp,
  const Pose2D & vehiclePose,
  const Twist2D & vehicleTwist)
{
  diagnostics_.updateLocalisationRate(stamp);
  double vehicleSpeed = vehicleTwist.linearSpeeds.x();

  if (matchedPoints_.empty()) {
    matchedPoints_ = romea::core::match(
      path_,
      vehiclePose,
      vehicleSpeed,
      predictionTimeHorizon_,
      maximalResearchRadius_);
  } else {
    matchedPoints_ = romea::core::match(
      path_,
      vehiclePose,
      vehicleSpeed,
      matchedPoints_[trackedMatchedPointIndex_], 2,
      predictionTimeHorizon_,
      maximalResearchRadius_);
  }

  if (!matchedPoints_.empty()) {
    diagnostics_.updatePathMatchingStatus(true);
    trackedMatchedPointIndex_ = bestMatchedPointIndex(matchedPoints_, vehicleSpeed);
    return matchedPoints_[trackedMatchedPointIndex_];
  } else {
    diagnostics_.updatePathMatchingStatus(false);
    return {};
  }
}

//-----------------------------------------------------------------------------
DiagnosticReport PathMatching::getReport(const Duration & stamp)
{
  return diagnostics_.makeReport(stamp);
}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  matchedPoints_.clear();
}

}  // namespace core
}  // namespace romea
