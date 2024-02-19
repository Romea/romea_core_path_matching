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
#include "romea_core_path_matching/OnTheFlyPathMatching.hpp"

// bool boolean(const romea::core::DiagnosticStatus & status)
// {
//   return status == romea::core::DiagnosticStatus::OK;
// }


class TestOnTheFlyPathMatching : public ::testing::Test
{
public:
  TestOnTheFlyPathMatching()
  : pathMatching_(1.0, 10.0, 3.0, 0.1, 0.1)
  {
  }

  void create_path()
  {
    double dt = 0.1;
    romea::core::Twist2D leader_twist;
    leader_twist.linearSpeeds.x() = 2.0;

    romea::core::Pose2D leader_pose;
    for (size_t i = 0; i < 100; ++i) {
      pathMatching_.updatePath(romea::core::durationFromSecond(dt), leader_pose, leader_twist);
      leader_pose.position.x() += leader_twist.linearSpeeds.x() * dt;
    }
  }

  romea::core::OnTheFlyPathMatching pathMatching_;
};

//-----------------------------------------------------------------------------
TEST_F(TestOnTheFlyPathMatching, testPathMatchingFailed) {
  romea::core::Twist2D follower_twist;
  follower_twist.linearSpeeds.x() = 2.0;

  romea::core::Pose2D follower_pose;
  follower_pose.position.x() = 10;
  follower_pose.position.y() = 1;

  auto pathMatchingPoint = pathMatching_.match(
    romea::core::durationFromSecond(10), follower_pose, follower_twist);

  EXPECT_FALSE(pathMatchingPoint.has_value());
}

//-----------------------------------------------------------------------------
TEST_F(TestOnTheFlyPathMatching, testPathMatchingOK) {
  create_path();

  romea::core::Twist2D follower_twist;
  follower_twist.linearSpeeds.x() = 2.0;

  romea::core::Pose2D follower_pose;
  follower_pose.position.x() = 10;
  follower_pose.position.y() = 1;

  auto pathMatchingPoint = pathMatching_.match(
    romea::core::durationFromSecond(10), follower_pose, follower_twist);

  EXPECT_TRUE(pathMatchingPoint.has_value());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
