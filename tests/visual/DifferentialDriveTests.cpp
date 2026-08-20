#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::DifferentialDrive;
using robot::visual::RobotPose;
using robot::visual::WheelSpeeds;

constexpr float kEpsilon = 0.001F;

} // namespace

// 1: DefaultsToStopped
TEST(DifferentialDriveTest, DefaultsToStopped)
{
    // Arrange / Act
    DifferentialDrive drive;

    // Assert
    const WheelSpeeds speeds = drive.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 2: StopSetsBothWheelSpeedsToZero
TEST(DifferentialDriveTest, StopSetsBothWheelSpeedsToZero)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(1.0F, 1.0F);

    // Act
    drive.stop();

    // Assert
    const WheelSpeeds speeds = drive.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 3: EqualPositiveWheelSpeedsMoveStraight
TEST(DifferentialDriveTest, EqualPositiveWheelSpeedsMoveStraight)
{
    // Arrange: heading 0 faces +Z.
    DifferentialDrive drive;
    drive.setWheelSpeeds(1.0F, 1.0F);
    RobotPose pose{};

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, 0.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, 1.0F, kEpsilon);
    EXPECT_NEAR(pose.headingDegrees, 0.0F, kEpsilon);
}

// 4: EqualNegativeWheelSpeedsMoveBackward
TEST(DifferentialDriveTest, EqualNegativeWheelSpeedsMoveBackward)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(-1.0F, -1.0F);
    RobotPose pose{};

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, 0.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, -1.0F, kEpsilon);
}

// 5: ZeroWheelSpeedsDoNotMove
TEST(DifferentialDriveTest, ZeroWheelSpeedsDoNotMove)
{
    // Arrange: wheel speeds default to zero.
    DifferentialDrive drive;
    RobotPose pose{};
    pose.position = robot::visual::Vec3{2.0F, 0.125F, 3.0F};
    pose.headingDegrees = 45.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, 2.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, 3.0F, kEpsilon);
    EXPECT_NEAR(pose.headingDegrees, 45.0F, kEpsilon);
}

// 6: LeftNegativeRightPositiveRotatesInPlace
TEST(DifferentialDriveTest, LeftNegativeRightPositiveRotatesInPlace)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(-0.5F, 0.5F);
    RobotPose pose{};

    // Act
    drive.update(pose, 1.0F);

    // Assert: v = 0, so position stays put; heading changes.
    EXPECT_NEAR(pose.position.x, 0.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, 0.0F, kEpsilon);
    EXPECT_GT(std::fabs(pose.headingDegrees), 1.0F);
}

// 7: LeftPositiveRightNegativeRotatesOppositeDirection
TEST(DifferentialDriveTest, LeftPositiveRightNegativeRotatesOppositeDirection)
{
    // Arrange: mirror image of test 6's wheel speeds.
    DifferentialDrive driveA;
    driveA.setWheelSpeeds(-0.5F, 0.5F);
    RobotPose poseA{};
    driveA.update(poseA, 1.0F);

    DifferentialDrive driveB;
    driveB.setWheelSpeeds(0.5F, -0.5F);
    RobotPose poseB{};

    // Act
    driveB.update(poseB, 1.0F);

    // Assert: same magnitude, opposite sign of heading change; position
    // stays put in both cases. Headings are normalized into [0, 360), so
    // the opposite-direction heading is 360 - poseA's, not its raw
    // negation.
    EXPECT_NEAR(poseB.position.x, 0.0F, kEpsilon);
    EXPECT_NEAR(poseB.position.z, 0.0F, kEpsilon);
    EXPECT_NEAR(poseB.headingDegrees, 360.0F - poseA.headingDegrees, kEpsilon);
}

// 8: UnequalPositiveSpeedsProduceArc
TEST(DifferentialDriveTest, UnequalPositiveSpeedsProduceArc)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(0.5F, 1.0F);
    RobotPose pose{};

    // Act
    drive.update(pose, 1.0F);

    // Assert: both moved forward-ish and turned - neither position nor
    // heading stayed at their starting values.
    EXPECT_GT(pose.position.z, 0.0F);
    EXPECT_GT(std::fabs(pose.headingDegrees), 0.1F);
}

// 9: HeadingZeroStraightMovesPositiveZ
TEST(DifferentialDriveTest, HeadingZeroStraightMovesPositiveZ)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(1.0F, 1.0F);
    RobotPose pose{};
    pose.headingDegrees = 0.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, 0.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, 1.0F, kEpsilon);
}

// 10: HeadingNinetyStraightMovesPositiveX
TEST(DifferentialDriveTest, HeadingNinetyStraightMovesPositiveX)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(1.0F, 1.0F);
    RobotPose pose{};
    pose.headingDegrees = 90.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, 1.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, 0.0F, kEpsilon);
}

// 11: HeadingOneEightyStraightMovesNegativeZ
TEST(DifferentialDriveTest, HeadingOneEightyStraightMovesNegativeZ)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(1.0F, 1.0F);
    RobotPose pose{};
    pose.headingDegrees = 180.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, 0.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, -1.0F, kEpsilon);
}

// 12: HeadingTwoSeventyStraightMovesNegativeX
TEST(DifferentialDriveTest, HeadingTwoSeventyStraightMovesNegativeX)
{
    // Arrange
    DifferentialDrive drive;
    drive.setWheelSpeeds(1.0F, 1.0F);
    RobotPose pose{};
    pose.headingDegrees = 270.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert
    EXPECT_NEAR(pose.position.x, -1.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, 0.0F, kEpsilon);
}

// 13: DeltaTimeScalesDistance
TEST(DifferentialDriveTest, DeltaTimeScalesDistance)
{
    // Arrange: two independent drives/poses.
    DifferentialDrive driveHalf;
    driveHalf.setWheelSpeeds(1.0F, 1.0F);
    RobotPose poseHalf{};

    DifferentialDrive driveFull;
    driveFull.setWheelSpeeds(1.0F, 1.0F);
    RobotPose poseFull{};

    // Act
    driveHalf.update(poseHalf, 0.5F);
    driveFull.update(poseFull, 1.0F);

    // Assert: double the delta time produces double the distance moved.
    EXPECT_NEAR(poseFull.position.z, poseHalf.position.z * 2.0F, kEpsilon);
}

// 14: HeadingWrapsAbove360
TEST(DifferentialDriveTest, HeadingWrapsAbove360)
{
    // Arrange: heading close to 360, spinning in place with enough omega*dt
    // to cross the wrap boundary.
    DifferentialDrive drive;
    drive.setWheelSpeeds(-5.0F, 5.0F);
    RobotPose pose{};
    pose.headingDegrees = 350.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert: normalized into [0, 360).
    EXPECT_GE(pose.headingDegrees, 0.0F);
    EXPECT_LT(pose.headingDegrees, 360.0F);
}

// 15: HeadingWrapsBelowZero
TEST(DifferentialDriveTest, HeadingWrapsBelowZero)
{
    // Arrange: heading near 0, spinning the opposite direction enough to
    // cross below zero.
    DifferentialDrive drive;
    drive.setWheelSpeeds(5.0F, -5.0F);
    RobotPose pose{};
    pose.headingDegrees = 10.0F;

    // Act
    drive.update(pose, 1.0F);

    // Assert: normalized into [0, 360), not left negative.
    EXPECT_GE(pose.headingDegrees, 0.0F);
    EXPECT_LT(pose.headingDegrees, 360.0F);
}

// 16: SymmetricInPlaceRotationKeepsPosition
TEST(DifferentialDriveTest, SymmetricInPlaceRotationKeepsPosition)
{
    // Arrange: repeated small in-place-rotation steps must never drift the
    // position away from where it started, however many steps accumulate.
    DifferentialDrive drive;
    drive.setWheelSpeeds(-0.7F, 0.7F);
    RobotPose pose{};
    pose.position = robot::visual::Vec3{1.0F, 0.125F, -2.0F};

    // Act
    for (int i = 0; i < 20; ++i)
    {
        drive.update(pose, 0.1F);
    }

    // Assert
    EXPECT_NEAR(pose.position.x, 1.0F, kEpsilon);
    EXPECT_NEAR(pose.position.z, -2.0F, kEpsilon);
}

// 17: SmallerWheelTrackProducesFasterAngularRate
TEST(DifferentialDriveTest, SmallerWheelTrackProducesFasterAngularRate)
{
    // Arrange: identical wheel-speed differential, different tracks.
    DifferentialDrive narrowDrive(0.3F);
    narrowDrive.setWheelSpeeds(-1.0F, 1.0F);
    RobotPose narrowPose{};

    DifferentialDrive wideDrive(0.6F);
    wideDrive.setWheelSpeeds(-1.0F, 1.0F);
    RobotPose widePose{};

    // Act
    narrowDrive.update(narrowPose, 0.1F);
    wideDrive.update(widePose, 0.1F);

    // Assert: a narrower track turns faster for the same wheel speeds.
    EXPECT_GT(std::fabs(narrowPose.headingDegrees), std::fabs(widePose.headingDegrees));
}

// 18: TurningDirectionMatchesConvention
TEST(DifferentialDriveTest, TurningDirectionMatchesConvention)
{
    // Arrange: omega = (vRight - vLeft) / wheelTrack - right wheel faster
    // than left must increase heading (rotating the front marker from +Z
    // toward +X, this project's convention for increasing heading), and
    // left wheel faster than right must decrease it.
    DifferentialDrive rightFasterDrive;
    rightFasterDrive.setWheelSpeeds(0.0F, 1.0F);
    RobotPose rightFasterPose{};

    DifferentialDrive leftFasterDrive;
    leftFasterDrive.setWheelSpeeds(1.0F, 0.0F);
    RobotPose leftFasterPose{};

    // Act
    rightFasterDrive.update(rightFasterPose, 0.1F);
    leftFasterDrive.update(leftFasterPose, 0.1F);

    // Assert
    EXPECT_GT(rightFasterPose.headingDegrees, 0.0F);
    EXPECT_LT(leftFasterPose.headingDegrees, 360.0F);
    EXPECT_NEAR(leftFasterPose.headingDegrees, 360.0F - rightFasterPose.headingDegrees, kEpsilon);
}
