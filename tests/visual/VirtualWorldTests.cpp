#include <algorithm>
#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::BoxObstacle;
using robot::visual::DeskObject;
using robot::visual::DeskObjectType;
using robot::visual::kRobotCollisionRadius;
using robot::visual::toString;
using robot::visual::VirtualWorld;

bool containsType(const VirtualWorld& world, DeskObjectType type)
{
    for (const DeskObject& object : world.deskObjects())
    {
        if (object.type == type)
        {
            return true;
        }
    }
    return false;
}

} // namespace

TEST(VirtualWorldTest, RobotStartsAtDeterministicPosition)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: Phase 13W final workspace redesign - robot starts parked at
    // the charging dock (same X as basePlatform()), 1.0F out toward the
    // open desk interior (+Z, since the dock now sits at the rear -Z edge
    // - see VirtualWorld.cpp's kRobotStart*).
    EXPECT_FLOAT_EQ(world.robotPose().position.x, world.basePlatform().position.x);
    EXPECT_FLOAT_EQ(world.robotPose().position.z, world.basePlatform().position.z + 1.0F);
}

TEST(VirtualWorldTest, RobotStartsAboveGroundByHalfBodyHeight)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: resting on the ground plane, not embedded in or floating
    // above it.
    EXPECT_GT(world.robotPose().position.y, 0.0F);
    EXPECT_LT(world.robotPose().position.y, 1.0F);
}

TEST(VirtualWorldTest, RobotStartsFacingAcrossTheDesk)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: Phase 13W human-visual-redesign v2 - heading 90 (per
    // VisualMath.hpp's forwardDirection() convention: 0 = +Z, 90 = +X)
    // faces across the open desk interior, only a quarter-turn away from
    // the dock's own bearing (0). A full 180-degree start (facing
    // directly away from home) forced HomeNavigator's Aligning phase to
    // sweep through every heading, including ones pointing at the Mouse
    // desk object a little over a body-length away - a false
    // VirtualRobotHardware::obstacleDetected() body-corridor hazard this
    // 90-degree start heading avoids entirely by never sweeping past it
    // (see VirtualRobotHardwareTests.cpp's own
    // ReturnHomeReachesChargingDockWithoutOscillating and
    // docs/technical-decisions.md, Phase 13W v2, "Return Home regression").
    EXPECT_FLOAT_EQ(world.robotPose().headingDegrees, 90.0F);
}

TEST(VirtualWorldTest, WorldContainsExpectedObstacleCount)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: Phase 13W final workspace redesign - a clean desk holds only
    // Monitor/Keyboard/Mouse (Mug/Notebook/LampBase removed, explicit
    // product requirement) plus the charging dock's 1 rear-housing
    // obstacle = 4.
    EXPECT_EQ(world.obstacles().size(), 4u);
}

TEST(VirtualWorldTest, ObstaclesHaveNonZeroSize)
{
    // Arrange
    VirtualWorld world;

    // Act / Assert
    for (const auto& obstacle : world.obstacles())
    {
        EXPECT_GT(obstacle.size.x, 0.0F);
        EXPECT_GT(obstacle.size.y, 0.0F);
        EXPECT_GT(obstacle.size.z, 0.0F);
    }
}

TEST(VirtualWorldTest, BasePlatformIsPositionedAwayFromRobotStart)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: Phase 13W human-visual-redesign v2 - the robot now starts
    // "at/immediately in front of" the dock (same X, deliberately - see
    // VirtualWorld.cpp), so only Z differs; the two must still not
    // coincide (robot does not start already sitting exactly on the dock's
    // own arrival point).
    EXPECT_NE(world.basePlatform().position.z, world.robotPose().position.z);
}

TEST(VirtualWorldTest, BasePlatformHasNonZeroFootprint)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert
    EXPECT_GT(world.basePlatform().size.x, 0.0F);
    EXPECT_GT(world.basePlatform().size.z, 0.0F);
}

TEST(VirtualWorldTest, ConstructingMultipleWorldsIsDeterministic)
{
    // Arrange / Act
    VirtualWorld first;
    VirtualWorld second;

    // Assert: no randomness, no shared mutable state between instances.
    EXPECT_FLOAT_EQ(first.robotPose().position.x, second.robotPose().position.x);
    EXPECT_FLOAT_EQ(first.robotPose().position.y, second.robotPose().position.y);
    EXPECT_FLOAT_EQ(first.robotPose().position.z, second.robotPose().position.z);
    EXPECT_EQ(first.obstacles().size(), second.obstacles().size());
}

// --- Phase 13W: desktop workspace world model ---

// 1: WorkspaceContainsMonitor
TEST(VirtualWorldTest, WorkspaceContainsMonitor)
{
    VirtualWorld world;
    EXPECT_TRUE(containsType(world, DeskObjectType::Monitor));
}

// 2: WorkspaceContainsKeyboard
TEST(VirtualWorldTest, WorkspaceContainsKeyboard)
{
    VirtualWorld world;
    EXPECT_TRUE(containsType(world, DeskObjectType::Keyboard));
}

// 3: WorkspaceContainsMouse
TEST(VirtualWorldTest, WorkspaceContainsMouse)
{
    VirtualWorld world;
    EXPECT_TRUE(containsType(world, DeskObjectType::Mouse));
}

// 4: WorkspaceDoesNotContainMug
TEST(VirtualWorldTest, WorkspaceDoesNotContainMug)
{
    // Phase 13W final workspace redesign: explicit product requirement -
    // a clean desk holds only Monitor/Keyboard/Mouse.
    VirtualWorld world;
    EXPECT_FALSE(containsType(world, DeskObjectType::Mug));
}

// 5: WorkspaceDoesNotContainNotebook
TEST(VirtualWorldTest, WorkspaceDoesNotContainNotebook)
{
    VirtualWorld world;
    EXPECT_FALSE(containsType(world, DeskObjectType::Notebook));
}

// 6: WorkspaceDoesNotContainLampBase
TEST(VirtualWorldTest, WorkspaceDoesNotContainLampBase)
{
    VirtualWorld world;
    EXPECT_FALSE(containsType(world, DeskObjectType::LampBase));
}

// 7: ObjectsHavePositiveDimensions
TEST(VirtualWorldTest, ObjectsHavePositiveDimensions)
{
    VirtualWorld world;
    ASSERT_EQ(world.deskObjects().size(), 3u);
    for (const DeskObject& object : world.deskObjects())
    {
        EXPECT_GT(object.size.x, 0.0F);
        EXPECT_GT(object.size.y, 0.0F);
        EXPECT_GT(object.size.z, 0.0F);
    }
}

// 8: ObjectsStayInsideTableBounds
TEST(VirtualWorldTest, ObjectsStayInsideTableBounds)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    for (const DeskObject& object : world.deskObjects())
    {
        EXPECT_GE(object.position.x - (object.size.x / 2.0F), table.minX);
        EXPECT_LE(object.position.x + (object.size.x / 2.0F), table.maxX);
        EXPECT_GE(object.position.z - (object.size.z / 2.0F), table.minZ);
        EXPECT_LE(object.position.z + (object.size.z / 2.0F), table.maxZ);
    }
}

// 9: ObjectsDoNotOverlapChargingDockParkingArea
//
// "Parking area" = the disk HomeNavigator actually targets/arrives
// within (base.position, HomeNavigator::kHomeArrivalRadius - duplicated
// here as a literal, since visual/world-model tests must not depend on
// robot_hardware) - no desk object's footprint may intersect it.
TEST(VirtualWorldTest, ObjectsDoNotOverlapChargingDockParkingArea)
{
    constexpr float kHomeArrivalRadius = 0.40F;
    VirtualWorld world;
    const auto& base = world.basePlatform();

    for (const DeskObject& object : world.deskObjects())
    {
        const float minX = object.position.x - (object.size.x / 2.0F);
        const float maxX = object.position.x + (object.size.x / 2.0F);
        const float minZ = object.position.z - (object.size.z / 2.0F);
        const float maxZ = object.position.z + (object.size.z / 2.0F);
        const float closestX = std::clamp(base.position.x, minX, maxX);
        const float closestZ = std::clamp(base.position.z, minZ, maxZ);
        const float dx = base.position.x - closestX;
        const float dz = base.position.z - closestZ;
        const float distance = std::sqrt((dx * dx) + (dz * dz));
        EXPECT_GE(distance, kHomeArrivalRadius);
    }
}

// 10: DemoLayoutLeavesNavigableClearanceForRobot
//
// The desk-object cluster, treated as one bounding region, must leave a
// robot-diameter's worth of clear space to at least one full table edge
// on both the X and Z axes - proof a route AROUND the whole cluster
// always exists, without asserting every individual item-to-item gap is
// itself robot-passable (real desk items legitimately sit close together
// - the robot is expected to go around the cluster, not thread between a
// keyboard and a mouse).
TEST(VirtualWorldTest, DemoLayoutLeavesNavigableClearanceForRobot)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const float robotDiameter = kRobotCollisionRadius * 2.0F;

    ASSERT_FALSE(world.deskObjects().empty());
    float minX = world.deskObjects().front().position.x;
    float maxX = minX;
    float minZ = world.deskObjects().front().position.z;
    float maxZ = minZ;
    for (const DeskObject& object : world.deskObjects())
    {
        minX = std::min(minX, object.position.x - (object.size.x / 2.0F));
        maxX = std::max(maxX, object.position.x + (object.size.x / 2.0F));
        minZ = std::min(minZ, object.position.z - (object.size.z / 2.0F));
        maxZ = std::max(maxZ, object.position.z + (object.size.z / 2.0F));
    }

    // Clearance to at least one edge along each axis.
    EXPECT_TRUE((minX - table.minX >= robotDiameter) || (table.maxX - maxX >= robotDiameter));
    EXPECT_TRUE((minZ - table.minZ >= robotDiameter) || (table.maxZ - maxZ >= robotDiameter));
}

// ============================================================
// Phase 13W final workspace redesign: clean-desk object set, layout, and
// navigable-corridor coverage
// ============================================================
//
// A modest named safety margin (on top of the robot's own collision
// diameter) used throughout the corridor checks below - the same
// "diameter plus a margin" concept the brief itself asks for, kept as one
// local named constant rather than a repeated magic number.
namespace
{
constexpr float kNavigationSafetyMargin = 0.10F;

std::optional<BoxObstacle> findDeskObjectBox(const VirtualWorld& world, DeskObjectType type)
{
    const std::size_t index = world.deskObjectObstacleIndex(type);
    if (index >= world.obstacles().size())
    {
        return std::nullopt;
    }
    return world.obstacles()[index];
}
} // namespace

// --- Clean-desk object set ---

// 11: NoLegacyGenericObstacleExists
//
// Every physical obstacle in the default workspace must be either a
// registered desk object or the dock's own rear housing - no standalone,
// unlabeled generic box left over from any earlier phase.
TEST(VirtualWorldTest, NoLegacyGenericObstacleExists)
{
    VirtualWorld world;
    EXPECT_EQ(world.obstacles().size(), world.deskObjects().size() + 1);
}

// 12: EveryPhysicalObstacleCorrespondsToVisibleWorkspaceObject
TEST(VirtualWorldTest, EveryPhysicalObstacleCorrespondsToVisibleWorkspaceObject)
{
    VirtualWorld world;
    for (std::size_t i = 0; i < world.obstacles().size(); ++i)
    {
        if (i == VirtualWorld::kDockHousingIndex)
        {
            continue; // the dock's own visible rear housing
        }
        bool matchesDeskObject = false;
        for (const DeskObject& object : world.deskObjects())
        {
            if (object.position.x == world.obstacles()[i].position.x &&
                object.position.z == world.obstacles()[i].position.z)
            {
                matchesDeskObject = true;
                break;
            }
        }
        EXPECT_TRUE(matchesDeskObject) << "obstacle " << i << " has no visible desk-object counterpart";
    }
}

// --- Layout ---

// 13: TableAspectIsApproximatelyTwoToOne
TEST(VirtualWorldTest, TableAspectIsApproximatelyTwoToOne)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const float width = table.maxX - table.minX;
    const float depth = table.maxZ - table.minZ;
    EXPECT_NEAR(width / depth, 2.0F, 0.1F);
}

// 14: MonitorNearRearCenter
TEST(VirtualWorldTest, MonitorNearRearCenter)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const std::optional<BoxObstacle> monitor = findDeskObjectBox(world, DeskObjectType::Monitor);
    ASSERT_TRUE(monitor.has_value());

    // Closer to the rear (-Z) edge than the front (+Z) edge.
    EXPECT_LT(std::fabs(monitor->position.z - table.minZ), std::fabs(monitor->position.z - table.maxZ));
    // Roughly within the desk's own central half along X.
    EXPECT_GT(monitor->position.x, table.minX / 2.0F);
    EXPECT_LT(monitor->position.x, table.maxX / 2.0F);
}

// 15: KeyboardInFrontOfMonitor
TEST(VirtualWorldTest, KeyboardInFrontOfMonitor)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> monitor = findDeskObjectBox(world, DeskObjectType::Monitor);
    const std::optional<BoxObstacle> keyboard = findDeskObjectBox(world, DeskObjectType::Keyboard);
    ASSERT_TRUE(monitor.has_value());
    ASSERT_TRUE(keyboard.has_value());
    EXPECT_GT(keyboard->position.z, monitor->position.z);
}

// 16: MouseRightOfKeyboard
TEST(VirtualWorldTest, MouseRightOfKeyboard)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> keyboard = findDeskObjectBox(world, DeskObjectType::Keyboard);
    const std::optional<BoxObstacle> mouse = findDeskObjectBox(world, DeskObjectType::Mouse);
    ASSERT_TRUE(keyboard.has_value());
    ASSERT_TRUE(mouse.has_value());
    const float keyboardRightEdge = keyboard->position.x + (keyboard->size.x / 2.0F);
    EXPECT_GT(mouse->position.x, keyboardRightEdge);
}

// 17: DockImmediatelyBesideMonitor
TEST(VirtualWorldTest, DockImmediatelyBesideMonitor)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const std::optional<BoxObstacle> monitor = findDeskObjectBox(world, DeskObjectType::Monitor);
    ASSERT_TRUE(monitor.has_value());
    const auto& dock = world.basePlatform();

    // To the monitor's right...
    EXPECT_GT(dock.position.x, monitor->position.x);
    // ...and close by, relative to the desk's own width (never off in a
    // separate, disconnected region of the desk).
    const float gap = dock.position.x - (monitor->position.x + (monitor->size.x / 2.0F)) - (dock.size.x / 2.0F);
    EXPECT_LT(gap, (table.maxX - table.minX) * 0.3F);
}

// 18: DockNearRearEdge
TEST(VirtualWorldTest, DockNearRearEdge)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const auto& dock = world.basePlatform();
    EXPECT_LT(std::fabs(dock.position.z - table.minZ), (table.maxZ - table.minZ) * 0.35F);
}

// 19: DockOpeningFacesDeskInterior
//
// The rear housing sits strictly behind the platform (further toward the
// -Z table edge), and the robot's own start point sits strictly in front
// of it (toward the desk interior) - the parking slot opens the same way
// the robot actually enters/leaves.
TEST(VirtualWorldTest, DockOpeningFacesDeskInterior)
{
    VirtualWorld world;
    const auto& dock = world.basePlatform();
    const BoxObstacle& housing = world.obstacles()[VirtualWorld::kDockHousingIndex];
    EXPECT_LT(housing.position.z, dock.position.z);
    EXPECT_LT(dock.position.z, world.robotPose().position.z);
}

// 20: RobotStartsAtDock
TEST(VirtualWorldTest, RobotStartsAtDock)
{
    VirtualWorld world;
    const auto& dock = world.basePlatform();
    const float dx = dock.position.x - world.robotPose().position.x;
    const float dz = dock.position.z - world.robotPose().position.z;
    const float distance = std::sqrt((dx * dx) + (dz * dz));
    // Close enough to read as "parked at the dock," never already
    // overlapping its own arrival point (see
    // RobotStartsAtDeterministicPosition/BasePlatformIsPositionedAwayFromRobotStart
    // above for the exact, deterministic offset).
    EXPECT_GT(distance, 0.0F);
    EXPECT_LT(distance, 2.0F);
}

// --- Navigable corridors ---

// 21: DockExitCorridorIsWiderThanRobot / DockExitCorridorIsNavigable
//
// No desk object may intrude into the straight lane between the dock and
// the robot's own start point - the corridor the robot actually departs
// through every time Return Home completes.
TEST(VirtualWorldTest, DockExitCorridorIsNavigable)
{
    VirtualWorld world;
    const auto& dock = world.basePlatform();
    const float robotRadius = kRobotCollisionRadius + kNavigationSafetyMargin;
    const float corridorMinX = dock.position.x - robotRadius;
    const float corridorMaxX = dock.position.x + robotRadius;
    const float corridorMinZ = dock.position.z;
    const float corridorMaxZ = world.robotPose().position.z;

    for (const DeskObject& object : world.deskObjects())
    {
        const float objMinX = object.position.x - (object.size.x / 2.0F);
        const float objMaxX = object.position.x + (object.size.x / 2.0F);
        const float objMinZ = object.position.z - (object.size.z / 2.0F);
        const float objMaxZ = object.position.z + (object.size.z / 2.0F);
        const bool overlapsX = objMaxX > corridorMinX && objMinX < corridorMaxX;
        const bool overlapsZ = objMaxZ > corridorMinZ && objMinZ < corridorMaxZ;
        EXPECT_FALSE(overlapsX && overlapsZ) << toString(object.type) << " blocks the dock exit corridor";
    }
}

// 22: MonitorDockGeometryDoesNotTrapRobot
//
// The gap between the monitor's own stand and the dock must comfortably
// fit the robot - the two must never form a narrow physical trap the
// robot could get wedged between while leaving the dock.
TEST(VirtualWorldTest, MonitorDockGeometryDoesNotTrapRobot)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> monitor = findDeskObjectBox(world, DeskObjectType::Monitor);
    ASSERT_TRUE(monitor.has_value());
    const auto& dock = world.basePlatform();

    const float monitorRightEdge = monitor->position.x + (monitor->size.x / 2.0F);
    const float dockLeftEdge = dock.position.x - (dock.size.x / 2.0F);
    const float gap = dockLeftEdge - monitorRightEdge;
    EXPECT_GT(gap, (kRobotCollisionRadius * 2.0F) + kNavigationSafetyMargin);
}

// 23: KeyboardDoesNotBlockDockExit
TEST(VirtualWorldTest, KeyboardDoesNotBlockDockExit)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> keyboard = findDeskObjectBox(world, DeskObjectType::Keyboard);
    ASSERT_TRUE(keyboard.has_value());
    const auto& dock = world.basePlatform();

    const float keyboardRightEdge = keyboard->position.x + (keyboard->size.x / 2.0F);
    const float dockCorridorLeftEdge = dock.position.x - kRobotCollisionRadius - kNavigationSafetyMargin;
    EXPECT_LT(keyboardRightEdge, dockCorridorLeftEdge);
}

// 24: MonitorBypassRouteExists
//
// At least one side of the monitor's own stand must have a clear lane to
// a table edge wide enough for the robot - so a route around the monitor
// specifically (not just the whole cluster, per
// DemoLayoutLeavesNavigableClearanceForRobot above) always exists.
TEST(VirtualWorldTest, MonitorBypassRouteExists)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const std::optional<BoxObstacle> monitor = findDeskObjectBox(world, DeskObjectType::Monitor);
    ASSERT_TRUE(monitor.has_value());

    const float monitorLeftEdge = monitor->position.x - (monitor->size.x / 2.0F);
    const float monitorRightEdge = monitor->position.x + (monitor->size.x / 2.0F);
    const float robotDiameter = kRobotCollisionRadius * 2.0F;

    EXPECT_TRUE(((monitorLeftEdge - table.minX) >= robotDiameter) ||
                ((table.maxX - monitorRightEdge) >= robotDiameter));
}

// 25: OpenFrontDeskAreaExists
//
// The front half of the desk (the +Z side, opposite the rear-mounted
// monitor/dock) is deliberately left clear - a visually clean desk, not a
// cluttered one, per this phase's own brief.
TEST(VirtualWorldTest, OpenFrontDeskAreaExists)
{
    VirtualWorld world;
    const auto& table = world.tableSurface();
    const float frontHalfBoundary = (table.minZ + table.maxZ) / 2.0F;

    for (const DeskObject& object : world.deskObjects())
    {
        const float objMaxZ = object.position.z + (object.size.z / 2.0F);
        EXPECT_LT(objMaxZ, frontHalfBoundary) << toString(object.type) << " reaches into the front half of the desk";
    }
}
