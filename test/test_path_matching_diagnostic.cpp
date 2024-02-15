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


// gtest
#include <gtest/gtest.h>

// std
#include <random>
#include <string>

// romea
#include "romea_core_path_matching/PathMatchingDiagnostic.hpp"

bool boolean(const romea::core::DiagnosticStatus & status)
{
  return status == romea::core::DiagnosticStatus::OK;
}


class TestPathMatchingDiagnostic : public ::testing::Test
{
public:
  TestPathMatchingDiagnostic()
  : diagnostic("/foo/bar.json")
  {
  }

  romea::core::DiagnosticReport getReport(
    const bool & localisationStatus,
    const bool & pathMatchingStatus,
    const double & stamp)
  {
    if (localisationStatus) {
      for (size_t n = 0; n <= 10; ++n) {
        diagnostic.updateLocalisationRate(romea::core::durationFromSecond(n * 0.1));
      }
      diagnostic.updatePathMatchingStatus(pathMatchingStatus);
    }
    return diagnostic.makeReport(romea::core::durationFromSecond(stamp));
  }

  romea::core::PathMatchingDiagnostic diagnostic;
};

//-----------------------------------------------------------------------------
TEST_F(TestPathMatchingDiagnostic, testInitialValue)
{
  auto report = getReport(false, false, 1.0);
  EXPECT_EQ(report.diagnostics.size(), 1);
  EXPECT_EQ(boolean(report.diagnostics.front().status), false);
  EXPECT_STREQ(report.diagnostics.front().message.c_str(), "no data received from localisation");
  EXPECT_EQ(report.info.size(), 4);
  EXPECT_STREQ(report.info["localisation_rate"].c_str(), "");
  EXPECT_STREQ(report.info["path_matching"].c_str(), "");
  EXPECT_STREQ(report.info["path_file_directory"].c_str(), "/foo");
  EXPECT_STREQ(report.info["path_file_name"].c_str(), "bar.json");
  std::cout << report << std::endl;
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatchingDiagnostic, testPathMatchingFailed)
{
  auto report = getReport(true, false, 1.0);
  EXPECT_EQ(report.diagnostics.size(), 2);
  EXPECT_EQ(boolean(report.diagnostics.front().status), true);
  EXPECT_STREQ(report.diagnostics.front().message.c_str(), "localisation_rate is OK.");
  EXPECT_EQ(boolean(report.diagnostics.back().status), false);
  EXPECT_STREQ(report.diagnostics.back().message.c_str(), "path matching failed.");
  EXPECT_EQ(report.info.size(), 4);
  EXPECT_STREQ(report.info["localisation_rate"].c_str(), "10");
  EXPECT_STREQ(report.info["path_matching"].c_str(), "false");
  EXPECT_STREQ(report.info["path_file_directory"].c_str(), "/foo");
  EXPECT_STREQ(report.info["path_file_name"].c_str(), "bar.json");

  std::cout << report << std::endl;
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatchingDiagnostic, testPathMatchingSucceded)
{
  auto report = getReport(true, true, 1.0);
  EXPECT_EQ(report.diagnostics.size(), 2);
  EXPECT_EQ(boolean(report.diagnostics.front().status), true);
  EXPECT_STREQ(report.diagnostics.front().message.c_str(), "localisation_rate is OK.");
  EXPECT_EQ(boolean(report.diagnostics.back().status), true);
  EXPECT_STREQ(report.diagnostics.back().message.c_str(), "path matching succeeded.");
  EXPECT_EQ(report.info.size(), 4);
  EXPECT_STREQ(report.info["localisation_rate"].c_str(), "10");
  EXPECT_STREQ(report.info["path_matching"].c_str(), "true");
  EXPECT_STREQ(report.info["path_file_directory"].c_str(), "/foo");
  EXPECT_STREQ(report.info["path_file_name"].c_str(), "bar.json");

  std::cout << report << std::endl;
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatchingDiagnostic, testLocalisatinTimeout)
{
  auto report = getReport(true, true, 10.0);
  EXPECT_EQ(report.diagnostics.size(), 1);
  EXPECT_EQ(boolean(report.diagnostics.front().status), false);
  EXPECT_STREQ(report.diagnostics.front().message.c_str(), "localisation_rate timeout.");
  EXPECT_EQ(report.info.size(), 4);
  EXPECT_STREQ(report.info["localisation_rate"].c_str(), "");
  EXPECT_STREQ(report.info["path_matching"].c_str(), "");
  EXPECT_STREQ(report.info["path_file_directory"].c_str(), "/foo");
  EXPECT_STREQ(report.info["path_file_name"].c_str(), "bar.json");

  std::cout << report << std::endl;

}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
