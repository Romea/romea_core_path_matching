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
#include <limits>
#include <string>


// romea
#include "romea_core_path_matching/OnTheFlyPathMatchingDiagnostic.hpp"

namespace
{
std::string booleanToString(const bool & flag)
{
  return flag ? "true" : "false";
}
}

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
OnTheFlyPathMatchingDiagnostic::OnTheFlyPathMatchingDiagnostic()
: leaderLocalisationRateDiagnostic_("leader_localisation", 0,
    std::numeric_limits<double>::epsilon()),
  followerLocalisationRateDiagnostic_("follower_localisation", 0,
    std::numeric_limits<double>::epsilon()),
  pathMatchingStatus_()
{
  setReportInfo(pathMatchingStatus_, "path_matching", "");
}


//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingDiagnostic::updateLeaderLocalisationRate(const Duration & duration)
{
  leaderLocalisationRateDiagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingDiagnostic::updateFollowerLocalisationRate(const Duration & duration)
{
  followerLocalisationRateDiagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingDiagnostic::updatePathMatchingStatus(const bool & status)
{
  if (status) {
    pathMatchingStatus_.diagnostics.push_back(
      Diagnostic(DiagnosticStatus::OK, "path matching succeeded."));
  } else {
    pathMatchingStatus_.diagnostics.push_back(
      Diagnostic(DiagnosticStatus::ERROR, "path matching failed."));
  }
  setReportInfo(pathMatchingStatus_, "path_matching", booleanToString(status));
}


//-----------------------------------------------------------------------------
DiagnosticReport OnTheFlyPathMatchingDiagnostic::makeReport(const core::Duration & duration)
{
  leaderLocalisationRateDiagnostic_.heartBeatCallback(duration);
  if (!followerLocalisationRateDiagnostic_.heartBeatCallback(duration)) {
    pathMatchingStatus_.diagnostics.clear();
    setReportInfo(pathMatchingStatus_, "path_matching", "");
  }

  DiagnosticReport report;
  report += leaderLocalisationRateDiagnostic_.getReport();
  report += followerLocalisationRateDiagnostic_.getReport();
  report += pathMatchingStatus_;
  return report;
}

}  // namespace core
}  // namespace romea
