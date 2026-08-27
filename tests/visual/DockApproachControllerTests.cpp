#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/DockApproachController.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::BasePlatform;
using robot::visual::computeDockApproachPoint;
using robot::visual::computeDockEntranceHeadingDegrees;
using robot::visual::DockApproachController;
using robot::visual::DockApproachOutput;
using robot::visual::DockApproachState;
using robot::visual::isDockApproachGeometryValid;
using robot::visual::kRobotCollisionRadius;
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

// ============================================================
// computeDockApproachPoint() / computeDockEntranceHeadingDegrees()
// ============================================================

// --- 1: DockApproachPointIsCenteredOnDockEntrance ---
TEST(DockApproachPointTest, DockApproachPointIsCenteredOnDockEntrance)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const Vec3 approach = computeDockApproachPoint(base, world.tableSurface());

    // The dock's own placement (VirtualWorld.cpp) is nearest the table's
    // -Z edge, so the approach point must sit on the SAME X as the
    // platform (centered on its entrance), offset only along Z.
    EXPECT_NEAR(approach.x, base.position.x, 1.0e-4F);
    EXPECT_GT(approach.z, base.position.z);
}

// --- 2: DockApproachPointIsInsideTable ---
TEST(DockApproachPointTest, DockApproachPointIsInsideTable)
{
    VirtualWorld world;
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(world.basePlatform(), table);

    EXPECT_GT(approach.x, table.minX);
    EXPECT_LT(approach.x, table.maxX);
    EXPECT_GT(approach.z, table.minZ);
    EXPECT_LT(approach.z, table.maxZ);
    // Comfortably inside, never merely on the boundary.
    EXPECT_GT(approach.x - table.minX, kRobotCollisionRadius);
    EXPECT_GT(table.maxX - approach.x, kRobotCollisionRadius);
    EXPECT_GT(approach.z - table.minZ, kRobotCollisionRadius);
    EXPECT_GT(table.maxZ - approach.z, kRobotCollisionRadius);
    EXPECT_TRUE(isDockApproachGeometryValid(world.basePlatform(), table));
}

// --- 3: DockApproachPointIsOutsideDockHousing ---
TEST(DockApproachPointTest, DockApproachPointIsOutsideDockHousing)
{
    VirtualWorld world;
    const Vec3 approach = computeDockApproachPoint(world.basePlatform(), world.tableSurface());

    // The dock housing is the LAST obstacle registered by VirtualWorld's
    // constructor (VirtualWorld::kDockHousingIndex) - real housing
    // geometry, never a re-guessed literal.
    const auto& housing = world.obstacles()[VirtualWorld::kDockHousingIndex];
    const float halfX = housing.size.x / 2.0F;
    const float halfZ = housing.size.z / 2.0F;
    const float closestX = std::clamp(approach.x, housing.position.x - halfX, housing.position.x + halfX);
    const float closestZ = std::clamp(approach.z, housing.position.z - halfZ, housing.position.z + halfZ);
    const float clearance = distanceWorld(approach, Vec3{closestX, approach.y, closestZ});

    // The approach point sits on the OPPOSITE side of the platform from
    // the housing (entrance side vs. rear side) - clearance must be
    // comfortably larger than the robot's own collision radius.
    EXPECT_GT(clearance, kRobotCollisionRadius);
}

// --- 4: DockApproachPointHasRobotClearance ---
TEST(DockApproachPointTest, DockApproachPointHasRobotClearance)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const Vec3 approach = computeDockApproachPoint(base, world.tableSurface());

    // The approach point must sit clear of the platform's OWN footprint
    // too - the robot should not be standing half-on the dock platform
    // itself while still in NavigatingToApproach/Aligning.
    const float halfX = base.size.x / 2.0F;
    const float halfZ = base.size.z / 2.0F;
    const float closestX = std::clamp(approach.x, base.position.x - halfX, base.position.x + halfX);
    const float closestZ = std::clamp(approach.z, base.position.z - halfZ, base.position.z + halfZ);
    const float clearance = distanceWorld(approach, Vec3{closestX, approach.y, closestZ});

    EXPECT_GE(clearance, kRobotCollisionRadius);
}

TEST(DockApproachPointTest, EntranceHeadingPointsFromApproachTowardHome)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);
    const float heading = computeDockEntranceHeadingDegrees(base, table);

    // Approach sits north (+Z) of home in this project's real geometry,
    // so driving "forward" at this heading must decrease Z - heading 180
    // (this project's 0 = +Z convention) is exactly that.
    EXPECT_NEAR(heading, 180.0F, 1.0F);
    EXPECT_GT(approach.z, base.position.z);
}

// ============================================================
// DockApproachController state machine
// ============================================================

// --- 6: AtApproachPointRobotAlignsBeforeDrivingForward ---
TEST(DockApproachControllerTest, AtApproachPointRobotAlignsBeforeDrivingForward)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);

    DockApproachController controller;
    // At the approach point, facing AWAY from the dock entrance heading.
    const DockApproachOutput output = controller.update(poseAt(approach, 0.0F), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::Aligning);
    EXPECT_TRUE(output.driving);
    // Turning wheels (unequal, opposite sign), never a forward command.
    EXPECT_NE(output.wheelSpeeds.left, output.wheelSpeeds.right);
    EXPECT_NEAR(output.wheelSpeeds.left, -output.wheelSpeeds.right, 1.0e-4F);
}

// --- 7: IncorrectDockHeadingDoesNotAdvance ---
TEST(DockApproachControllerTest, IncorrectDockHeadingDoesNotAdvance)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);
    const float entranceHeading = computeDockEntranceHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(approach, entranceHeading + 90.0F), base, table, true, true);
    for (int i = 0; i < 10; ++i)
    {
        const DockApproachOutput output =
            controller.update(poseAt(approach, entranceHeading + 90.0F), base, table, true, true);
        EXPECT_EQ(output.state, DockApproachState::Aligning);
    }
}

// --- 8: CorrectDockHeadingBeginsSlowFinalApproach ---
TEST(DockApproachControllerTest, CorrectDockHeadingBeginsSlowFinalApproach)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);
    const float entranceHeading = computeDockEntranceHeadingDegrees(base, table);

    DockApproachController controller;
    // First call: NavigatingToApproach -> Aligning (arrival just detected).
    // Second call: already-correct heading is now evaluated -> FinalApproach.
    controller.update(poseAt(approach, entranceHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(approach, entranceHeading), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::FinalApproach);
    EXPECT_TRUE(output.driving);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, DockApproachController::kFinalApproachSpeed);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, DockApproachController::kFinalApproachSpeed);
}

// --- 9: FinalApproachUsesEqualWheelSpeeds ---
TEST(DockApproachControllerTest, FinalApproachUsesEqualWheelSpeeds)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);
    const float entranceHeading = computeDockEntranceHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(approach, entranceHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(approach, entranceHeading), base, table, true, true);

    ASSERT_EQ(output.state, DockApproachState::FinalApproach);
    EXPECT_EQ(output.wheelSpeeds.left, output.wheelSpeeds.right);
    // Deliberately slower than ordinary navigation.
    EXPECT_LT(DockApproachController::kFinalApproachSpeed, 0.8F);
}

TEST(DockApproachControllerTest, ArrivesWithinHomeArrivalRadius)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();

    DockApproachController controller;
    const DockApproachOutput output = controller.update(poseAt(base.position, 0.0F), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::Arrived);
    EXPECT_TRUE(output.arrived);
    EXPECT_FALSE(output.driving);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(DockApproachControllerTest, WaitsInactiveUntilStage1Arrives)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);

    DockApproachController controller;
    const DockApproachOutput output = controller.update(poseAt(approach, 0.0F), base, table, true, false);

    EXPECT_EQ(output.state, DockApproachState::NavigatingToApproach);
    EXPECT_FALSE(output.driving);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(DockApproachControllerTest, DisablingResetsToInactive)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);
    const float entranceHeading = computeDockEntranceHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(approach, entranceHeading), base, table, true, true);
    const DockApproachOutput output = controller.update(poseAt(approach, entranceHeading), base, table, false, true);

    EXPECT_EQ(output.state, DockApproachState::Inactive);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(DockApproachControllerTest, DisplacedFarFromApproachPointFallsBackToNavigatingToApproach)
{
    VirtualWorld world;
    const BasePlatform& base = world.basePlatform();
    const TableSurface& table = world.tableSurface();
    const Vec3 approach = computeDockApproachPoint(base, table);
    const float entranceHeading = computeDockEntranceHeadingDegrees(base, table);

    DockApproachController controller;
    controller.update(poseAt(approach, entranceHeading), base, table, true, true);
    controller.update(poseAt(approach, entranceHeading), base, table, true, true);
    ASSERT_EQ(controller.state(), DockApproachState::FinalApproach);

    // Simulate a large displacement (e.g. Safety recovery) far off the
    // known lane.
    const Vec3 displaced{approach.x + 3.0F, approach.y, approach.z + 3.0F};
    const DockApproachOutput output = controller.update(poseAt(displaced, entranceHeading), base, table, true, true);

    EXPECT_EQ(output.state, DockApproachState::NavigatingToApproach);
}

} // namespace
