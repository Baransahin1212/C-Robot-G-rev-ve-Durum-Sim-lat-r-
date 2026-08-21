#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/HomeNavigator.hpp"

namespace
{

using robot::visual::BasePlatform;
using robot::visual::HomeNavigationOutput;
using robot::visual::HomeNavigationState;
using robot::visual::HomeNavigator;
using robot::visual::RobotPose;
using robot::visual::Vec3;

constexpr float kEpsilon = 0.01F;

BasePlatform MakeBase(float x, float z)
{
    return BasePlatform{Vec3{x, 0.025F, z}, Vec3{1.5F, 0.05F, 1.5F}};
}

RobotPose MakePose(float x, float z, float headingDegrees)
{
    return RobotPose{Vec3{x, 0.125F, z}, headingDegrees};
}

} // namespace

// 1: DefaultsInactive
TEST(HomeNavigatorTest, DefaultsInactive)
{
    HomeNavigator navigator;

    EXPECT_EQ(navigator.state(), HomeNavigationState::Inactive);
}

// 2: EnableStartsNavigation
TEST(HomeNavigatorTest, EnableStartsNavigation)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 180.0F); // facing away from base
    const BasePlatform base = MakeBase(0.0F, 10.0F);

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    EXPECT_EQ(output.state, HomeNavigationState::Aligning);
    EXPECT_EQ(navigator.state(), HomeNavigationState::Aligning);
}

// 3: AlreadyAtHomeReportsArrived
TEST(HomeNavigatorTest, AlreadyAtHomeReportsArrived)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(4.0F, 4.0F, 0.0F);
    const BasePlatform base = MakeBase(4.0F, 4.0F); // exactly on the robot

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    // Base start condition: no rotation/driving commanded on arrival.
    EXPECT_EQ(output.state, HomeNavigationState::Arrived);
    EXPECT_TRUE(output.arrived);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

// 4: TargetDirectlyAheadDrivesForward
TEST(HomeNavigatorTest, TargetDirectlyAheadDrivesForward)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F); // facing +Z
    const BasePlatform base = MakeBase(0.0F, 5.0F);    // directly ahead

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    ASSERT_EQ(output.state, HomeNavigationState::Driving);
    EXPECT_GT(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, output.wheelSpeeds.right);
}

// 5: TargetDirectlyBehindAlignsFirst
TEST(HomeNavigatorTest, TargetDirectlyBehindAlignsFirst)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F); // facing +Z
    const BasePlatform base = MakeBase(0.0F, -5.0F);   // directly behind

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    EXPECT_EQ(output.state, HomeNavigationState::Aligning);
    EXPECT_NE(output.wheelSpeeds.left, output.wheelSpeeds.right);
}

// 6: PositiveHeadingErrorTurnsCorrectDirection
TEST(HomeNavigatorTest, PositiveHeadingErrorTurnsCorrectDirection)
{
    HomeNavigator navigator;
    // Base to the +X side of straight-ahead -> positive heading error.
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F);
    const BasePlatform base = MakeBase(5.0F, 5.0F);

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    ASSERT_GT(output.headingErrorDegrees, 0.0F);
    EXPECT_LT(output.wheelSpeeds.left, 0.0F);
    EXPECT_GT(output.wheelSpeeds.right, 0.0F);
}

// 7: NegativeHeadingErrorTurnsCorrectDirection
TEST(HomeNavigatorTest, NegativeHeadingErrorTurnsCorrectDirection)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F);
    const BasePlatform base = MakeBase(-5.0F, 5.0F);

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    ASSERT_LT(output.headingErrorDegrees, 0.0F);
    EXPECT_GT(output.wheelSpeeds.left, 0.0F);
    EXPECT_LT(output.wheelSpeeds.right, 0.0F);
}

// 8: HeadingZeroConvention
TEST(HomeNavigatorTest, HeadingZeroConvention)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 90.0F); // arbitrary current heading
    const BasePlatform base = MakeBase(0.0F, 5.0F);     // target directly on +Z

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    EXPECT_NEAR(output.targetHeadingDegrees, 0.0F, kEpsilon);
}

// 9: HeadingNinetyConvention
TEST(HomeNavigatorTest, HeadingNinetyConvention)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F);
    const BasePlatform base = MakeBase(5.0F, 0.0F); // target directly on +X

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    EXPECT_NEAR(output.targetHeadingDegrees, 90.0F, kEpsilon);
}

// 10: AlignmentWithinStartToleranceBeginsDriving
TEST(HomeNavigatorTest, AlignmentWithinStartToleranceBeginsDriving)
{
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 5.0F); // target heading 0

    // Establish Aligning with a large error.
    ASSERT_EQ(navigator.update(MakePose(0.0F, 0.0F, 90.0F), base, true).state, HomeNavigationState::Aligning);

    // Heading now within the 8-degree start-driving tolerance.
    const HomeNavigationOutput output = navigator.update(MakePose(0.0F, 0.0F, 5.0F), base, true);

    EXPECT_EQ(output.state, HomeNavigationState::Driving);
}

// 11: DrivingContinuesInsideStopTolerance
TEST(HomeNavigatorTest, DrivingContinuesInsideStopTolerance)
{
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 5.0F);

    navigator.update(MakePose(0.0F, 0.0F, 90.0F), base, true);              // Aligning
    ASSERT_EQ(navigator.update(MakePose(0.0F, 0.0F, 5.0F), base, true).state, HomeNavigationState::Driving);

    // Still comfortably inside the 15-degree stop tolerance.
    const HomeNavigationOutput output = navigator.update(MakePose(0.0F, 0.0F, 10.0F), base, true);

    EXPECT_EQ(output.state, HomeNavigationState::Driving);
}

// 12: DrivingReturnsToAligningOutsideStopTolerance
TEST(HomeNavigatorTest, DrivingReturnsToAligningOutsideStopTolerance)
{
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 5.0F);

    navigator.update(MakePose(0.0F, 0.0F, 90.0F), base, true);
    ASSERT_EQ(navigator.update(MakePose(0.0F, 0.0F, 5.0F), base, true).state, HomeNavigationState::Driving);

    // Heading drifted past the 15-degree stop tolerance.
    const HomeNavigationOutput output = navigator.update(MakePose(0.0F, 0.0F, 20.0F), base, true);

    EXPECT_EQ(output.state, HomeNavigationState::Aligning);
}

// 13: ArrivalStopsWheels
TEST(HomeNavigatorTest, ArrivalStopsWheels)
{
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 5.0F);

    navigator.update(MakePose(0.0F, 0.0F, 90.0F), base, true);
    navigator.update(MakePose(0.0F, 0.0F, 5.0F), base, true); // Driving

    // Now within the arrival radius.
    const HomeNavigationOutput output = navigator.update(MakePose(0.0F, 4.8F, 0.0F), base, true);

    ASSERT_EQ(output.state, HomeNavigationState::Arrived);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

// 14: DistanceToHomeCorrect
TEST(HomeNavigatorTest, DistanceToHomeCorrect)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F);
    const BasePlatform base = MakeBase(3.0F, 4.0F); // 3-4-5 triangle

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    EXPECT_NEAR(output.distanceToHome, 5.0F, kEpsilon);
}

// 15: TargetHeadingCorrect
TEST(HomeNavigatorTest, TargetHeadingCorrect)
{
    HomeNavigator navigator;
    const RobotPose pose = MakePose(0.0F, 0.0F, 0.0F);
    const BasePlatform base = MakeBase(-5.0F, 0.0F); // due -X -> heading -90 (== 270)

    const HomeNavigationOutput output = navigator.update(pose, base, true);

    EXPECT_NEAR(output.targetHeadingDegrees, 270.0F, kEpsilon);
}

// 16: ResetReturnsInactive
TEST(HomeNavigatorTest, ResetReturnsInactive)
{
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 5.0F);
    navigator.update(MakePose(0.0F, 0.0F, 90.0F), base, true);
    ASSERT_NE(navigator.state(), HomeNavigationState::Inactive);

    navigator.reset();

    EXPECT_EQ(navigator.state(), HomeNavigationState::Inactive);
}

// 17: ReEnableAfterResetWorks
TEST(HomeNavigatorTest, ReEnableAfterResetWorks)
{
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 5.0F);
    navigator.update(MakePose(0.0F, 0.0F, 5.0F), base, true);
    navigator.reset();
    ASSERT_EQ(navigator.state(), HomeNavigationState::Inactive);

    // A future Return Home request must work again.
    const HomeNavigationOutput output = navigator.update(MakePose(0.0F, 0.0F, 5.0F), base, true);

    EXPECT_EQ(output.state, HomeNavigationState::Driving);
}
