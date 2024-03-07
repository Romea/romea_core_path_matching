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
#include "../test/test_helper.h"
#include "romea_core_path_matching/PathMatching.hpp"

class TestPathMatching : public ::testing::Test
{
public:
  TestPathMatching()
  : pathMatching(std::string(TEST_DIR) + "/test_path_matching.cvs", 10.0, 3.0)
  {
  }

  romea::core::PathMatching pathMatching;
};

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testPathMatchingFailed)
{
  romea::core::Twist2D follower_twist;
  follower_twist.linearSpeeds.x() = 2.0;

  romea::core::Pose2D follower_pose;
  follower_pose.position.x() = 10;
  follower_pose.position.y() = 20;

  auto pathMatchingPoints = pathMatching.match(
    romea::core::durationFromSecond(10), follower_pose, follower_twist);

  // EXPECT_FALSE(pathMatchingPoint.has_value());
  EXPECT_TRUE(pathMatchingPoints.empty());
}

//-----------------------------------------------------------------------------
TEST_F(TestPathMatching, testPathMatchingOK)
{
  romea::core::Twist2D follower_twist;
  follower_twist.linearSpeeds.x() = 2.0;

  romea::core::Pose2D follower_pose;
  follower_pose.position.x() = 10;
  follower_pose.position.y() = 1;

  auto pathMatchingPoints = pathMatching.match(
    romea::core::durationFromSecond(10), follower_pose, follower_twist);

  // EXPECT_TRUE(pathMatchingPoint.has_value());
  EXPECT_FALSE(pathMatchingPoints.empty());
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
