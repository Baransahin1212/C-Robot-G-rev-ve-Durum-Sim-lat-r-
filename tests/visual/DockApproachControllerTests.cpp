#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/DockApproachArrivalEventSource.hpp"
#include "robot/visual/DockApproachController.hpp"
#include "robot/visual/DockCaptureRegion.hpp"
#include "robot/visual/DockChargingContacts.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualMath.hpp"

namespace
{

using robot::visual::BasePlatform;
using robot::visual::chargingContactsAligned;
using robot::visual::computeChargingContactErrors;
using robot::visual::computeDockChargingContacts;
using robot::visual::computeDockReverseHeadingDegrees;
using robot::visual::computeDockStagingPoint;
using robot::visual::computeRobotRearChargingContacts;
using robot::visual::DockApproachArrivalEventSource;
using robot::visual::DockApproachController;
using robot::visual::DockApproachOutput;
using robot::visual::DockApproachState;
using robot::visual::DockChargingContactErrors;
using robot::visual::DockChargingContactPair;
using robot::visual::dockInwardDirection;
using robot::visual::forwardDirection;
using robot::visual::isDockCaptureEligible;
using robot::visual::isDockStagingGeometryValid;
using robot::visual::isInsideDockStagingCaptureRegion;
using robot::visual::isNearestHazardAttributableToDockGeometry;
using robot::visual::kContactAlignmentToleranceWorldUnits;
using robot::visual::kContactHeightWorldUnits;
using robot::visual::kContactPairSpacingWorldUnits;
using robot::visual::kContactRadiusWorldUnits;
using robot::visual::kRobotCollisionRadius;
using robot::visual::rightDirection;
using robot::visual::BoxObstacle;
using robot::visual::RobotPose;
using robot::visual::TableSurface;
using robot::visual::Vec3;
using robot::visual::VirtualWorld;

float distanceWorld(const Vec3& a, const Vec3& b)
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

RobotPose poseAt(const Vec3& position, float headingDegrees = 0.0F)
{
    return RobotPose{position, headingDegrees};
}

// The pose whose rear contacts are exactly centered on the dock's own
// contacts, facing the reverse-docking heading - i.e. "fully docked."
// Derived from real robot geometry (never a guessed literal): the robot's
// REAR (not its center) touches the dock, so its center sits
// kBodyLength/2 further toward the desk interior than `base.position`.
RobotPose dockedPose(const BasePlatform& base, const TableSurface& tableSurface)
{
    const Vec3 inward = dockInwardDirection(base, tableSurface);
    const float halfLength = robot::visual::RobotDimensions::kBodyLength / 2.0F;
    const Vec3 position{base.position.x + (inward.x * halfLength), base.position.y,
                         base.position.z + (inward.z * halfLength)};
    return RobotPose{position, computeDockReverseHeadingDegrees(base, tableSurface)};
}

// ============================================================
// Dock/robot charging contact geometry
// ============================================================

// --- 1: DockHasTwoChargingContacts ---
TEST(DockChargingContactsTest, DockHasTwoChargingContacts)
{
    VirtualWorld world;
    const DockChargingContactPair contacts = computeDockChargingContacts(world.basePlatform(), world.tableSurface());

    EXPECT_GT(contacts.left.radius, 0.0F);
    EXPECT_GT(contacts.right.radius, 0.0F);
    EXPECT_GT(distanceWorld(contacts.left.position, contacts.right.position), 0.0F);
}

// --- 2: ChargingContactsAreSymmetric ---
TEST(DockChargingContactsTest, ChargingContactsAreSymmetric)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const DockChargingContactPair contacts = computeDockChargingContacts(base, world.tableSurface());

    const float leftDistanceFromBase = distanceWorld(contacts.left.position, base.position);
    const float rightDistanceFromBase = distanceWorld(contacts.right.position, base.position);
    EXPECT_NEAR(leftDistanceFromBase, rightDistanceFromBase, 1.0e-4F);
    EXPECT_NEAR(leftDistanceFromBase, kContactPairSpacingWorldUnits / 2.0F, 1.0e-4F);
}

// --- 3: ChargingContactSpacingFitsRobotWidth ---
TEST(DockChargingContactsTest, ChargingContactSpacingFitsRobotWidth)
{
    VirtualWorld world;
    const DockChargingContactPair contacts = computeDockChargingContacts(world.basePlatform(), world.tableSurface());
    const float spacing = distanceWorld(contacts.left.position, contacts.right.position);

    EXPECT_NEAR(spacing, kContactPairSpacingWorldUnits, 1.0e-4F);
    EXPECT_LT(spacing, robot::visual::RobotDimensions::kBodyWidth);
    // Comfortably inside, not merely narrower - see class docs.
    EXPECT_GT(robot::visual::RobotDimensions::kBodyWidth - spacing, 0.05F);
}

// --- 4: RobotHasTwoRearChargingContacts ---
TEST(DockChargingContactsTest, RobotHasTwoRearChargingContacts)
{
    const RobotPose pose = poseAt(Vec3{0.0F, 0.0F, 0.0F}, 0.0F);
    const DockChargingContactPair contacts = computeRobotRearChargingContacts(pose);

    EXPECT_GT(contacts.left.radius, 0.0F);
    EXPECT_GT(contacts.right.radius, 0.0F);
    EXPECT_GT(distanceWorld(contacts.left.position, contacts.right.position), 0.0F);
}

// --- 5: RobotContactsMatchDockSpacing ---
TEST(DockChargingContactsTest, RobotContactsMatchDockSpacing)
{
    const RobotPose pose = poseAt(Vec3{0.0F, 0.0F, 0.0F}, 0.0F);
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(pose);
    const float spacing = distanceWorld(robotContacts.left.position, robotContacts.right.position);

    EXPECT_NEAR(spacing, kContactPairSpacingWorldUnits, 1.0e-4F);
}

// --- 6: RobotContactsAreOnRearFace ---
TEST(DockChargingContactsTest, RobotContactsAreOnRearFace)
{
    const RobotPose pose = poseAt(Vec3{0.0F, 0.0F, 0.0F}, 0.0F);
    const Vec3 forward = forwardDirection(pose);
    const DockChargingContactPair contacts = computeRobotRearChargingContacts(pose);

    // Both contacts must sit BEHIND the robot's own position along its
    // forward axis (negative projection), never in front.
    const float leftProjection =
        ((contacts.left.position.x - pose.position.x) * forward.x) + ((contacts.left.position.z - pose.position.z) * forward.z);
    const float rightProjection = ((contacts.right.position.x - pose.position.x) * forward.x) +
                                   ((contacts.right.position.z - pose.position.z) * forward.z);
    EXPECT_LT(leftProjection, 0.0F);
    EXPECT_LT(rightProjection, 0.0F);
}

// --- 7: ContactHeightsMatch ---
TEST(DockChargingContactsTest, ContactHeightsMatch)
{
    VirtualWorld world;
    const DockChargingContactPair dockContacts =
        computeDockChargingContacts(world.basePlatform(), world.tableSurface());
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(poseAt(Vec3{0.0F, 0.0F, 0.0F}));

    EXPECT_FLOAT_EQ(dockContacts.left.position.y, kContactHeightWorldUnits);
    EXPECT_FLOAT_EQ(dockContacts.right.position.y, kContactHeightWorldUnits);
    EXPECT_FLOAT_EQ(robotContacts.left.position.y, kContactHeightWorldUnits);
    EXPECT_FLOAT_EQ(robotContacts.right.position.y, kContactHeightWorldUnits);
}

// --- 8: DockedRequiresBothContactPairs ---
TEST(DockChargingContactsTest, DockedRequiresBothContactPairs)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, table);

    // Shift laterally so both sides move by the SAME amount, well past
    // kContactAlignmentToleranceWorldUnits - proves computeChargingContactErrors()
    // reports each side's REAL distance (never collapsing early to a
    // single pass/fail the way chargingContactsAligned() does), so a
    // caller can require BOTH leftError and rightError within tolerance,
    // never accept on the smaller of the two (or the center distance)
    // alone.
    const Vec3 shifted{docked.position.x + (kContactAlignmentToleranceWorldUnits * 4.0F), docked.position.y,
                        docked.position.z};
    const DockChargingContactPair robotContacts =
        computeRobotRearChargingContacts(poseAt(shifted, docked.headingDegrees));

    const DockChargingContactErrors errors = computeChargingContactErrors(dockContacts, robotContacts);
    EXPECT_GT(errors.leftError, kContactAlignmentToleranceWorldUnits);
    EXPECT_GT(errors.rightError, kContactAlignmentToleranceWorldUnits);
    EXPECT_FLOAT_EQ(errors.maxError, std::max(errors.leftError, errors.rightError));
}

// --- 9: RenderAndLogicUseSameDockContactGeometry ---
TEST(DockChargingContactsTest, RenderAndLogicUseSameDockContactGeometry)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();

    // Renderer3D.cpp's own dock-pin draw call (DrawSphere at
    // dockContacts.left/right.position, radius dockContacts.left/right.radius)
    // invokes this SAME function with these SAME BasePlatform/TableSurface
    // arguments - never a separately hardcoded/duplicated position. Two
    // independent calls with identical inputs producing an identical
    // result is the determinism proof that nothing (a stray static, a
    // per-call random offset) could ever separate what gets rendered from
    // what DockApproachController's own Docked decision (also this exact
    // function) checks against - there is only ever ONE source of this
    // geometry.
    const DockChargingContactPair first = computeDockChargingContacts(base, table);
    const DockChargingContactPair second = computeDockChargingContacts(base, table);

    EXPECT_FLOAT_EQ(first.left.position.x, second.left.position.x);
    EXPECT_FLOAT_EQ(first.left.position.y, second.left.position.y);
    EXPECT_FLOAT_EQ(first.left.position.z, second.left.position.z);
    EXPECT_FLOAT_EQ(first.right.position.x, second.right.position.x);
    EXPECT_FLOAT_EQ(first.right.position.y, second.right.position.y);
    EXPECT_FLOAT_EQ(first.right.position.z, second.right.position.z);
    EXPECT_FLOAT_EQ(first.left.position.y, kContactHeightWorldUnits);
    EXPECT_FLOAT_EQ(first.left.radius, kContactRadiusWorldUnits);
    EXPECT_FLOAT_EQ(first.right.radius, kContactRadiusWorldUnits);
}

// --- 10: RenderAndLogicUseSameRobotRearContactGeometry ---
TEST(DockChargingContactsTest, RenderAndLogicUseSameRobotRearContactGeometry)
{
    // An arbitrary, non-axis-aligned pose - the whole point of this audit
    // is to prove the match holds for a general heading, not merely the
    // convenient 0-degree case every other test in this file already uses.
    const RobotPose pose = poseAt(Vec3{1.25F, 0.0F, -0.6F}, 37.0F);
    const DockChargingContactPair logicContacts = computeRobotRearChargingContacts(pose);

    // Independently reproduces VisualRobot.cpp's drawVisualRobot() own
    // local-space contact placement (local X = rightDirection(), local Z =
    // forwardDirection(), rearZ = -(kBodyLength/2), local contact Y already
    // cancels pose.position.y under rlTranslatef - see that function's own
    // docs for the full derivation) and transforms it into world space via
    // this project's own forward/right basis vectors - the SAME basis
    // rlRotatef(pose.headingDegrees, 0,1,0) itself rotates local Z/X into.
    // A mismatch here would mean the two spheres the user actually SEES
    // are not where DockApproachController's own contact-alignment math
    // (computeRobotRearChargingContacts(), used by both this controller
    // and this exact assertion) thinks they are - precisely the class of
    // real, audited discrepancy Phase 13Y's fix closed for the rear-
    // contact sphere's now-removed extra `-radius*0.3F` offset (see
    // VisualRobot.cpp's own docs) - this test pins it so it can never
    // silently regress.
    const Vec3 right = rightDirection(pose);
    const Vec3 forward = forwardDirection(pose);
    const float halfSpacing = kContactPairSpacingWorldUnits / 2.0F;
    const float rearZ = -(robot::visual::RobotDimensions::kBodyLength / 2.0F);

    const auto localToWorld = [&](float lateralX) {
        return Vec3{pose.position.x + (right.x * lateralX) + (forward.x * rearZ), kContactHeightWorldUnits,
                    pose.position.z + (right.z * lateralX) + (forward.z * rearZ)};
    };
    const Vec3 renderedA = localToWorld(-halfSpacing);
    const Vec3 renderedB = localToWorld(halfSpacing);

    // Match as a SET - never assume a fixed left/right ordering convention
    // agreement (see DockChargingContacts.hpp's own cross-order matching
    // docs elsewhere in this codebase for why that assumption is never
    // made anywhere else either).
    const float distToLeftA = distanceWorld(renderedA, logicContacts.left.position);
    const float distToRightA = distanceWorld(renderedA, logicContacts.right.position);
    const bool aMatchesLeft = distToLeftA < distToRightA;
    const Vec3& expectedForA = aMatchesLeft ? logicContacts.left.position : logicContacts.right.position;
    const Vec3& expectedForB = aMatchesLeft ? logicContacts.right.position : logicContacts.left.position;

    EXPECT_NEAR(renderedA.x, expectedForA.x, 1.0e-4F);
    EXPECT_NEAR(renderedA.y, expectedForA.y, 1.0e-4F);
    EXPECT_NEAR(renderedA.z, expectedForA.z, 1.0e-4F);
    EXPECT_NEAR(renderedB.x, expectedForB.x, 1.0e-4F);
    EXPECT_NEAR(renderedB.y, expectedForB.y, 1.0e-4F);
    EXPECT_NEAR(renderedB.z, expectedForB.z, 1.0e-4F);
}

// ============================================================
// Docking staging point
// ============================================================

// --- 8: StagingPointIsOnDockCenterline ---
TEST(DockStagingPointTest, StagingPointIsOnDockCenterline)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const Vec3 staging = computeDockStagingPoint(base, world.tableSurface());

    // Offset only along the dock's own axis - the real desk's dock is
    // Z-offset from a table edge, so the staging point's X must match the
    // platform's exactly (no lateral drift).
    EXPECT_NEAR(staging.x, base.position.x, 1.0e-4F);
    EXPECT_GT(staging.z, base.position.z);
}

// --- 9: StagingPointIsInsideTable ---
TEST(DockStagingPointTest, StagingPointIsInsideTable)
{
    VirtualWorld world;
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(world.basePlatform(), table);

    EXPECT_GT(staging.x, table.minX);
    EXPECT_LT(staging.x, table.maxX);
    EXPECT_GT(staging.z, table.minZ);
    EXPECT_LT(staging.z, table.maxZ);
}

// --- 10: StagingPointHasPlanningClearance ---
TEST(DockStagingPointTest, StagingPointHasPlanningClearance)
{
    VirtualWorld world;
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(world.basePlatform(), table);

    EXPECT_GT(staging.x - table.minX, kRobotCollisionRadius);
    EXPECT_GT(table.maxX - staging.x, kRobotCollisionRadius);
    EXPECT_GT(staging.z - table.minZ, kRobotCollisionRadius);
    EXPECT_GT(table.maxZ - staging.z, kRobotCollisionRadius);
    EXPECT_TRUE(isDockStagingGeometryValid(world.basePlatform(), table));
}

// ============================================================
// DockApproachController - reverse alignment
// ============================================================

// --- 1: WrongHeadingDoesNotReverse ---
TEST(DockApproachControllerTest, WrongHeadingDoesNotReverse)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading + 90.0F), base, table, true, true);
    for (int i = 0; i < 10; ++i)
    {
        const DockApproachOutput output = controller.update(poseAt(staging, reverseHeading + 90.0F), base, table, true, true);
        EXPECT_EQ(output.state, DockApproachState::AlignForReverse);
        EXPECT_NE(output.wheelSpeeds.left, output.wheelSpeeds.right);
    }
}

// --- 2: CorrectReverseHeadingBeginsReverseMotion ---
TEST(DockApproachControllerTest, CorrectReverseHeadingBeginsReverseMotion)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    // First call: NavigateToStagingPoint -> AlignForReverse (arrival just
    // detected). Second call: already-correct heading evaluated ->
    // ReverseApproach.
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(staging, reverseHeading), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::ReverseApproach);
    EXPECT_TRUE(output.driving);
}

// --- 3: ReverseMotionUsesNegativeEqualWheelSpeeds ---
TEST(DockApproachControllerTest, ReverseMotionUsesNegativeEqualWheelSpeeds)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(staging, reverseHeading), base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::ReverseApproach);
    EXPECT_EQ(output.wheelSpeeds.left, output.wheelSpeeds.right);
    EXPECT_LT(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, -DockApproachController::kReverseDockSpeed);
}

// --- 4: ReverseSpeedIsLowerThanNormalNavigation ---
TEST(DockApproachControllerTest, ReverseSpeedIsLowerThanNormalNavigation)
{
    EXPECT_LT(DockApproachController::kReverseDockSpeed, 0.8F);
    EXPECT_GT(DockApproachController::kReverseDockSpeed, 0.0F);
}

// --- 5: ExcessiveLateralErrorStopsReverse ---
TEST(DockApproachControllerTest, ExcessiveLateralErrorStopsReverse)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    ASSERT_EQ(controller.state(), DockApproachState::ReverseApproach);

    // Displace laterally (perpendicular to the dock axis, which is Z for
    // the real desk - X is the lateral axis) well beyond tolerance, still
    // correctly headed.
    const Vec3 displaced{staging.x + (DockApproachController::kReverseLateralErrorToleranceWorldUnits * 3.0F),
                          staging.y, staging.z};
    const DockApproachOutput output = controller.update(poseAt(displaced, reverseHeading), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::AlignForReverse);
}

// --- 6: HeadingDriftStopsReverse ---
TEST(DockApproachControllerTest, HeadingDriftStopsReverse)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    ASSERT_EQ(controller.state(), DockApproachState::ReverseApproach);

    const float driftedHeading = reverseHeading + DockApproachController::kReverseDockHeadingReleaseToleranceDegrees + 1.0F;
    const DockApproachOutput output = controller.update(poseAt(staging, driftedHeading), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::AlignForReverse);
}

// --- 7: AlignmentUsesShortestTurn ---
TEST(DockApproachControllerTest, AlignmentUsesShortestTurn)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    // Heading just PAST the wrap-around from the reverse heading (e.g.
    // reverseHeading - 10, wrapped into [0,360)) - the shortest turn must
    // still be a small, single-direction correction, never a near-360
    // sweep.
    const float nearWrapHeading = std::fmod(reverseHeading - 10.0F + 360.0F, 360.0F);

    DockApproachController controller;
    controller.update(poseAt(staging, nearWrapHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(staging, nearWrapHeading), base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::AlignForReverse);
    // A near-10-degree error should produce the SAME wheel magnitude as
    // any other in-tolerance-exceeding error - never a special-cased near-
    // wrap behavior - and should have already been fixed to within a
    // small residual by two frames of turning at kAligningTurnSpeed, not
    // still reporting a near-350-degree error.
    EXPECT_NEAR(std::fabs(output.wheelSpeeds.left), DockApproachController::kAligningTurnSpeed, 1.0e-4F);
}

TEST(DockApproachControllerTest, WaitsInactiveUntilStage1Arrives)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);

    DockApproachController controller;
    const DockApproachOutput output = controller.update(poseAt(staging, 0.0F), base, table, true, false);

    EXPECT_EQ(output.state, DockApproachState::NavigateToStagingPoint);
    EXPECT_FALSE(output.driving);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(DockApproachControllerTest, DisablingResetsToInactive)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(staging, reverseHeading), base, table, false, true);

    EXPECT_EQ(output.state, DockApproachState::Inactive);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(DockApproachControllerTest, DisplacedFarFromStagingPointFallsBackToNavigateToStagingPoint)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    controller.update(poseAt(staging, reverseHeading), base, table, true, true);
    ASSERT_EQ(controller.state(), DockApproachState::ReverseApproach);

    const Vec3 displaced{staging.x + 3.0F, staging.y, staging.z + 3.0F};
    const DockApproachOutput output = controller.update(poseAt(displaced, reverseHeading), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::NavigateToStagingPoint);
}

// ============================================================
// DockApproachController - contact detection / docking
// ============================================================

// --- 1: NoDockWhenOnlyLeftContactMatches ---
TEST(DockApproachControllerContactTest, NoDockWhenOnlyLeftContactMatches)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, table);

    // Shift the robot laterally so only ONE contact pairing lands within
    // tolerance.
    const Vec3 shifted{docked.position.x + 0.15F, docked.position.y, docked.position.z};
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(poseAt(shifted, docked.headingDegrees));

    EXPECT_FALSE(chargingContactsAligned(dockContacts, robotContacts, kContactAlignmentToleranceWorldUnits));
}

// --- 2: NoDockWhenOnlyRightContactMatches ---
TEST(DockApproachControllerContactTest, NoDockWhenOnlyRightContactMatches)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, table);

    const Vec3 shifted{docked.position.x - 0.15F, docked.position.y, docked.position.z};
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(poseAt(shifted, docked.headingDegrees));

    EXPECT_FALSE(chargingContactsAligned(dockContacts, robotContacts, kContactAlignmentToleranceWorldUnits));
}

// --- 3: NoDockWhenPositionMatchesButHeadingWrong ---
TEST(DockApproachControllerContactTest, NoDockWhenPositionMatchesButHeadingWrong)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);
    const Vec3 staging = computeDockStagingPoint(base, table);

    // At the staging point, position is nowhere near contact range, but
    // this proves the controller-level Docked check requires heading
    // too - even if contacts coincidentally happened to be in range, a
    // wrong heading alone must never report Docked.
    DockApproachController controller;
    controller.update(poseAt(staging, reverseHeading + 90.0F), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(staging, reverseHeading + 90.0F), base, table, true, true);

    EXPECT_NE(output.state, DockApproachState::Docked);
    EXPECT_FALSE(output.docked);
}

// --- 4: NoDockWhenRobotBesideDock ---
TEST(DockApproachControllerContactTest, NoDockWhenRobotBesideDock)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, table);

    // Well off to the side of the dock, facing an arbitrary direction.
    const Vec3 beside{base.position.x + 1.5F, base.position.y, base.position.z};
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(poseAt(beside, 90.0F));

    EXPECT_FALSE(chargingContactsAligned(dockContacts, robotContacts, kContactAlignmentToleranceWorldUnits));
}

// --- 5: BothContactsAlignedReportsDocked ---
TEST(DockApproachControllerContactTest, BothContactsAlignedReportsDocked)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, table);
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(docked);

    EXPECT_TRUE(chargingContactsAligned(dockContacts, robotContacts, kContactAlignmentToleranceWorldUnits));

    DockApproachController controller;
    controller.update(docked, base, table, true, true);
    const DockApproachOutput output = controller.update(docked, base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::Docked);
    EXPECT_TRUE(output.docked);
}

// --- 6: DockedEmitsHomeReachedExactlyOnce ---
TEST(DockApproachControllerContactTest, DockedEmitsHomeReachedExactlyOnce)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);

    DockApproachController controller;
    DockApproachArrivalEventSource eventSource(controller);

    int homeReachedCount = 0;
    for (int i = 0; i < 5; ++i)
    {
        controller.update(docked, base, table, true, true);
        if (eventSource.pollEvent().has_value())
        {
            ++homeReachedCount;
        }
    }

    EXPECT_EQ(homeReachedCount, 1);
}

// --- 7: DockedStopsWheels ---
TEST(DockApproachControllerContactTest, DockedStopsWheels)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);

    DockApproachController controller;
    controller.update(docked, base, table, true, true);
    const DockApproachOutput output = controller.update(docked, base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::Docked);
    EXPECT_FALSE(output.driving);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

// ============================================================
// Phase 13Y final docking visual-precision polish
// ============================================================

// --- 1: FinalDockPoseHeadingIsWithinPrecisionTolerance ---
TEST(DockApproachControllerContactTest, FinalDockPoseHeadingIsWithinPrecisionTolerance)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(docked, base, table, true, true);
    const DockApproachOutput output = controller.update(docked, base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::Docked);
    const float headingError = std::fmod(std::fabs(docked.headingDegrees - reverseHeading) + 540.0F, 360.0F) - 180.0F;
    EXPECT_LE(std::fabs(headingError), DockApproachController::kFinalContactHeadingToleranceDegrees);
}

// --- 2: FinalDockPoseLateralErrorIsWithinPrecisionTolerance ---
TEST(DockApproachControllerContactTest, FinalDockPoseLateralErrorIsWithinPrecisionTolerance)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const Vec3 lateralAxis = rightDirection(RobotPose{Vec3{}, computeDockReverseHeadingDegrees(base, table) + 180.0F});

    DockApproachController controller;
    controller.update(docked, base, table, true, true);
    const DockApproachOutput output = controller.update(docked, base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::Docked);
    const float lateralError = ((docked.position.x - base.position.x) * lateralAxis.x) +
                                ((docked.position.z - base.position.z) * lateralAxis.z);
    EXPECT_LE(std::fabs(lateralError), DockApproachController::kFinalLateralToleranceWorldUnits);
}

// --- 3: DockedRejectedWhenVisiblyYawed ---
TEST(DockApproachControllerContactTest, DockedRejectedWhenVisiblyYawed)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);

    // Heading error deliberately BETWEEN the tight final-contact tolerance
    // and the general (looser) reverse-entry tolerance - a pose the OLDER,
    // general-only check would have let through as "close enough," but
    // this fix's own brief explicitly forbids ("do not continue reversing/
    // dock while visibly diagonal").
    const float yawedHeading = docked.headingDegrees +
        ((DockApproachController::kFinalContactHeadingToleranceDegrees +
          DockApproachController::kReverseDockHeadingToleranceDegrees) / 2.0F);
    const RobotPose yawed{docked.position, yawedHeading};

    DockApproachController controller;
    controller.update(yawed, base, table, true, true);
    const DockApproachOutput output = controller.update(yawed, base, table, true, true);

    EXPECT_NE(output.state, DockApproachState::Docked);
    EXPECT_FALSE(output.docked);
}

// --- 4: DockedRejectedWhenLaterallyOffset ---
TEST(DockApproachControllerContactTest, DockedRejectedWhenLaterallyOffset)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);

    // Lateral offset deliberately BETWEEN the tight final-contact
    // tolerance and the general (looser) reverse-approach tolerance.
    const float lateralOffset = (DockApproachController::kFinalLateralToleranceWorldUnits +
                                  DockApproachController::kReverseLateralErrorToleranceWorldUnits) / 2.0F;
    const Vec3 shifted{docked.position.x + lateralOffset, docked.position.y, docked.position.z};
    const RobotPose offsetPose{shifted, docked.headingDegrees};

    DockApproachController controller;
    controller.update(offsetPose, base, table, true, true);
    const DockApproachOutput output = controller.update(offsetPose, base, table, true, true);

    EXPECT_NE(output.state, DockApproachState::Docked);
    EXPECT_FALSE(output.docked);
}

// --- 5: PrecisionZoneUsesLowerReverseSpeed ---
TEST(DockApproachControllerTest, PrecisionZoneUsesLowerReverseSpeed)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const Vec3 inward = dockInwardDirection(base, table);

    // Backed off in DEPTH only (heading/lateral exact), by an amount
    // between the contact tolerance and the precision-zone entry distance -
    // close enough to be inside the precision zone, not yet close enough
    // to satisfy the contact-pair tolerance itself.
    const float depthBackoff =
        (kContactAlignmentToleranceWorldUnits + DockApproachController::kPrecisionZoneEntryDistanceWorldUnits) / 2.0F;
    const Vec3 nearContactPosition{docked.position.x + (inward.x * depthBackoff), docked.position.y,
                                    docked.position.z + (inward.z * depthBackoff)};
    const RobotPose approaching{nearContactPosition, docked.headingDegrees};

    DockApproachController controller;
    controller.update(approaching, base, table, true, true);
    const DockApproachOutput output = controller.update(approaching, base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::ReverseApproach);
    EXPECT_NEAR(std::fabs(output.wheelSpeeds.left), DockApproachController::kPrecisionReverseDockSpeed, 1.0e-4F);
    EXPECT_NEAR(std::fabs(output.wheelSpeeds.right), DockApproachController::kPrecisionReverseDockSpeed, 1.0e-4F);
    EXPECT_LT(DockApproachController::kPrecisionReverseDockSpeed, DockApproachController::kReverseDockSpeed);
}

// --- 6: HeadingDriftStopsPrecisionReverse ---
TEST(DockApproachControllerTest, HeadingDriftStopsPrecisionReverse)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const Vec3 inward = dockInwardDirection(base, table);

    const float depthBackoff =
        (kContactAlignmentToleranceWorldUnits + DockApproachController::kPrecisionZoneEntryDistanceWorldUnits) / 2.0F;
    const Vec3 nearContactPosition{docked.position.x + (inward.x * depthBackoff), docked.position.y,
                                    docked.position.z + (inward.z * depthBackoff)};
    const RobotPose approaching{nearContactPosition, docked.headingDegrees};

    DockApproachController controller;
    controller.update(approaching, base, table, true, true);
    controller.update(approaching, base, table, true, true);
    ASSERT_EQ(controller.state(), DockApproachState::ReverseApproach);

    // Heading drift deliberately BETWEEN the tight final-contact tolerance
    // and the general release tolerance - old (general-only) logic would
    // have kept reversing right through this; the precision zone must stop
    // and correct instead ("do not continue reversing while visibly
    // diagonal").
    const float driftedHeading = docked.headingDegrees + DockApproachController::kFinalContactHeadingToleranceDegrees + 1.0F;
    const RobotPose drifted{nearContactPosition, driftedHeading};
    const DockApproachOutput output = controller.update(drifted, base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::AlignForReverse);
    // A correction turn, never a continued straight reverse - opposite-
    // sign wheel speeds, not equal negative ones.
    EXPECT_LT(output.wheelSpeeds.left * output.wheelSpeeds.right, 0.0F);
}

// --- 7: PrecisionCorrectionCanRecoverAndDock ---
TEST(DockApproachControllerTest, PrecisionCorrectionCanRecoverAndDock)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const RobotPose docked = dockedPose(base, table);
    const Vec3 inward = dockInwardDirection(base, table);

    const float depthBackoff =
        (kContactAlignmentToleranceWorldUnits + DockApproachController::kPrecisionZoneEntryDistanceWorldUnits) / 2.0F;
    const Vec3 nearContactPosition{docked.position.x + (inward.x * depthBackoff), docked.position.y,
                                    docked.position.z + (inward.z * depthBackoff)};
    const RobotPose approaching{nearContactPosition, docked.headingDegrees};
    const float driftedHeading = docked.headingDegrees + DockApproachController::kFinalContactHeadingToleranceDegrees + 1.0F;
    const RobotPose drifted{nearContactPosition, driftedHeading};

    DockApproachController controller;
    controller.update(approaching, base, table, true, true);
    controller.update(approaching, base, table, true, true);
    ASSERT_EQ(controller.state(), DockApproachState::ReverseApproach);

    // A few bounded precision corrections - well under
    // kMaxPrecisionDockRetries - simulating heading drift being caught and
    // corrected repeatedly, never exhausting the retry budget into Failed.
    for (int i = 0; i < 3; ++i)
    {
        const DockApproachOutput correcting = controller.update(drifted, base, table, true, true);
        ASSERT_NE(correcting.state, DockApproachState::Failed);
        controller.update(approaching, base, table, true, true);
    }
    ASSERT_NE(controller.state(), DockApproachState::Failed);

    // Once the (now-corrected) pose exactly matches the perfect docked
    // pose, docking must still complete normally - the bounded retry
    // counter must never itself prevent an eventual successful dock.
    controller.update(docked, base, table, true, true);
    const DockApproachOutput output = controller.update(docked, base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::Docked);
    EXPECT_TRUE(output.docked);
}

// ============================================================
// Phase 13Y dock-capture handoff fix (real-GUI-traced) -
// DockCaptureRegion.hpp
// ============================================================

// --- CaptureRegion 1: InsideRadiusIsInsideCaptureRegion ---
TEST(DockCaptureRegionTest, InsideRadiusIsInsideCaptureRegion)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);

    EXPECT_TRUE(isInsideDockStagingCaptureRegion(poseAt(staging), base, table));
}

// --- CaptureRegion 2: OutsideRadiusIsNotInsideCaptureRegion ---
TEST(DockCaptureRegionTest, OutsideRadiusIsNotInsideCaptureRegion)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const Vec3 farAway{staging.x, staging.y,
                        staging.z + (DockApproachController::kDockStagingCaptureRadius * 3.0F)};

    EXPECT_FALSE(isInsideDockStagingCaptureRegion(poseAt(farAway), base, table));
}

// --- CaptureRegion 3: NearestHazardAttributedToDockWhenClosestObstacleIsHousing ---
TEST(DockCaptureRegionTest, NearestHazardAttributedToDockWhenClosestObstacleIsHousing)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    // Deliberately the final DOCKED position (right next to the housing),
    // not the staging point itself - at the staging point, this project's
    // real desk layout puts the mouse desk object numerically closer by raw
    // center-to-center distance than the housing is (0.79 vs 1.08 world
    // units), even though the mouse is nowhere near the robot's actual
    // forward sensor cone during a real approach (see
    // RealFrame62PoseCompletesReverseDocking in
    // MapAwareNavigationIntegrationTests.cpp for the real, traced end-to-
    // end proof that avoidance never triggers along the ACTUAL docking
    // trajectory) - this test instead proves the unambiguous case, deep
    // inside the validated reverse-docking lane where only the housing is
    // anywhere close.
    const Vec3 nearHousing = dockedPose(base, table).position;

    // No obstacle is placed by this test - it exercises the REAL production
    // VirtualWorld obstacle list, exactly like every other geometry test in
    // this file.
    EXPECT_TRUE(isNearestHazardAttributableToDockGeometry(poseAt(nearHousing), base, world.obstacles()));
}

// --- CaptureRegion 4: NearestHazardNotAttributedWhenUnrelatedObstacleCloser ---
TEST(DockCaptureRegionTest, NearestHazardNotAttributedWhenUnrelatedObstacleCloser)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const Vec3 staging = computeDockStagingPoint(base, world.tableSurface());

    std::vector<BoxObstacle> obstacles = world.obstacles();
    // An "unrelated" obstacle placed right next to the robot's own staging-
    // point position - closer to the robot than the dock's own housing is,
    // so it (correctly) becomes the nearest-to-robot obstacle while the
    // dock housing remains the nearest-to-base one - a genuine mismatch.
    obstacles.push_back(BoxObstacle{Vec3{staging.x + 0.05F, staging.y, staging.z}, Vec3{0.1F, 0.1F, 0.1F}, true});

    EXPECT_FALSE(isNearestHazardAttributableToDockGeometry(poseAt(staging), base, obstacles));
}

// --- CaptureRegion 5: DisabledUnrelatedObstacleNeverCountsAsNearest ---
TEST(DockCaptureRegionTest, DisabledUnrelatedObstacleNeverCountsAsNearest)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 nearHousing = dockedPose(base, table).position;

    std::vector<BoxObstacle> obstacles = world.obstacles();
    // Same placement as the test above, but disabled - must be ignored
    // entirely (mirrors robotPositionCollidesWithObstacles()'s own
    // enabled-only convention), so attribution falls back to the real dock
    // housing again.
    obstacles.push_back(
        BoxObstacle{Vec3{nearHousing.x + 0.05F, nearHousing.y, nearHousing.z}, Vec3{0.1F, 0.1F, 0.1F}, false});

    EXPECT_TRUE(isNearestHazardAttributableToDockGeometry(poseAt(nearHousing), base, obstacles));
}

// --- CaptureRegion 6: CaptureEligibleNearDockWhenSupported ---
TEST(DockCaptureRegionTest, CaptureEligibleNearDockWhenSupported)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    // See NearestHazardAttributedToDockWhenClosestObstacleIsHousing's own
    // docs for why this uses the docked position rather than the literal
    // staging point.
    const Vec3 nearHousing = dockedPose(base, table).position;

    EXPECT_TRUE(isDockCaptureEligible(poseAt(nearHousing), base, table, world.obstacles(), true));
}

// --- CaptureRegion 7: CaptureNotEligibleWhenNotPhysicallySupported ---
TEST(DockCaptureRegionTest, CaptureNotEligibleWhenNotPhysicallySupported)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);

    EXPECT_FALSE(isDockCaptureEligible(poseAt(staging), base, table, world.obstacles(), false));
}

// --- CaptureRegion 8: CaptureNotEligibleOutsideRadius ---
TEST(DockCaptureRegionTest, CaptureNotEligibleOutsideRadius)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 staging = computeDockStagingPoint(base, table);
    const Vec3 farAway{staging.x, staging.y,
                        staging.z + (DockApproachController::kDockStagingCaptureRadius * 3.0F)};

    EXPECT_FALSE(isDockCaptureEligible(poseAt(farAway), base, table, world.obstacles(), true));
}

// --- CaptureRegion 9: CaptureNotEligibleWhenCurrentPoseCollides ---
TEST(DockCaptureRegionTest, CaptureNotEligibleWhenCurrentPoseCollides)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    // The staging margin (kDockStagingMarginWorldUnits) plus the capture
    // margin (kDockCaptureMarginWorldUnits) together already keep the WHOLE
    // capture region structurally clear of the real dock housing by
    // construction (proof: the closest any in-radius point can get to the
    // housing is still ~0.08 world units outside kRobotCollisionRadius) -
    // so this test injects its own synthetic obstacle exactly at the
    // staging point instead, to exercise the collision-safety check in
    // isolation from that (already-proven-safe) real housing geometry.
    const Vec3 collidingPoint = computeDockStagingPoint(base, table);
    std::vector<BoxObstacle> obstacles = world.obstacles();
    obstacles.push_back(BoxObstacle{collidingPoint, Vec3{0.1F, 0.1F, 0.1F}, true});

    EXPECT_FALSE(isDockCaptureEligible(poseAt(collidingPoint), base, table, obstacles, true));
}

// --- CaptureRegion 10: CaptureNotEligibleWhenUnrelatedObstacleNearby ---
TEST(DockCaptureRegionTest, CaptureNotEligibleWhenUnrelatedObstacleNearby)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    // See NearestHazardAttributedToDockWhenClosestObstacleIsHousing's own
    // docs for why this uses the docked position - isolates this test's
    // OWN injected obstacle as the reason capture is withheld, rather than
    // the staging point's own unrelated (mouse) ambiguity.
    const Vec3 nearHousing = dockedPose(base, table).position;

    std::vector<BoxObstacle> obstacles = world.obstacles();
    obstacles.push_back(
        BoxObstacle{Vec3{nearHousing.x + 0.05F, nearHousing.y, nearHousing.z}, Vec3{0.1F, 0.1F, 0.1F}, true});

    EXPECT_FALSE(isDockCaptureEligible(poseAt(nearHousing), base, table, obstacles, true));
}

} // namespace
