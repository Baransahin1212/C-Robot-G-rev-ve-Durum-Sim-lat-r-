#include <cmath>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::BoxObstacle;
using robot::visual::DeskObject;
using robot::visual::DeskObjectType;
using robot::visual::DifferentialDrive;
using robot::visual::kRobotCollisionRadius;
using robot::visual::ObstacleSensorArrayReadings;
using robot::visual::ReactiveObstacleAvoidance;
using robot::visual::robotPositionCollidesWithObstacles;
namespace RobotDimensions = robot::visual::RobotDimensions;
using robot::visual::Vec3;
using robot::visual::VirtualObstacleSensorArray;
using robot::visual::VirtualWorld;

// Finds the DeskObject of `type` in `world` and returns the BoxObstacle
// registered alongside it (same position, per
// VirtualWorld::addDeskObject()'s own contract - see VirtualWorld.cpp).
// Fails the test (via a null return the caller ASSERTs on) if either
// lookup comes up empty, rather than silently testing nothing.
std::optional<BoxObstacle> findDeskObjectObstacle(const VirtualWorld& world, DeskObjectType type)
{
    for (const BoxObstacle& obstacle : world.obstacles())
    {
        for (const DeskObject& object : world.deskObjects())
        {
            if (object.type == type && object.position.x == obstacle.position.x &&
                object.position.z == obstacle.position.z)
            {
                return obstacle;
            }
        }
    }
    return std::nullopt;
}

} // namespace

// 1: RobotFarFromObstacleDoesNotCollide
TEST(RobotCollisionTest, RobotFarFromObstacleDoesNotCollide)
{
    const std::vector<BoxObstacle> obstacles = {BoxObstacle{Vec3{10.0F, 0.4F, 10.0F}, Vec3{0.8F, 0.8F, 0.8F}, true}};

    EXPECT_FALSE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, 0.0F}, obstacles));
}

// 2: RobotApproachingObstacleCannotEnterIt
TEST(RobotCollisionTest, RobotApproachingObstacleCannotEnterIt)
{
    // Obstacle centered at Z 2.0, size 0.8 -> near face Z 1.6.
    const std::vector<BoxObstacle> obstacles = {BoxObstacle{Vec3{0.0F, 0.4F, 2.0F}, Vec3{0.8F, 0.8F, 0.8F}, true}};

    // Just inside the collision radius of the near face - collides.
    const float justInsideZ = 1.6F - (kRobotCollisionRadius * 0.5F);
    EXPECT_TRUE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, justInsideZ}, obstacles));

    // Clearly outside the collision radius of the near face - does not.
    const float clearlyOutsideZ = 1.6F - (kRobotCollisionRadius * 2.0F);
    EXPECT_FALSE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, clearlyOutsideZ}, obstacles));
}

// 3: DisabledObstacleDoesNotCollideEvenAtItsCenter
TEST(RobotCollisionTest, DisabledObstacleDoesNotCollideEvenAtItsCenter)
{
    const std::vector<BoxObstacle> obstacles = {BoxObstacle{Vec3{0.0F, 0.4F, 2.0F}, Vec3{0.8F, 0.8F, 0.8F}, false}};

    EXPECT_FALSE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, 2.0F}, obstacles));
}

// 4: ObstacleToSideDoesNotBlockStraightMotion
TEST(RobotCollisionTest, ObstacleToSideDoesNotBlockStraightMotion)
{
    // Obstacle well off to the side in X; robot travels straight along Z
    // at X 0 - never within range in X.
    const std::vector<BoxObstacle> obstacles = {BoxObstacle{Vec3{5.0F, 0.4F, 2.0F}, Vec3{0.8F, 0.8F, 0.8F}, true}};

    EXPECT_FALSE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, 2.0F}, obstacles));
}

// 5: ReEnablingObstacleMakesItCollideAgain
TEST(RobotCollisionTest, ReEnablingObstacleMakesItCollideAgain)
{
    std::vector<BoxObstacle> obstacles = {BoxObstacle{Vec3{0.0F, 0.4F, 2.0F}, Vec3{0.8F, 0.8F, 0.8F}, false}};
    const Vec3 centerPosition{0.0F, 0.125F, 2.0F};

    ASSERT_FALSE(robotPositionCollidesWithObstacles(centerPosition, obstacles));

    obstacles[0].enabled = true;

    EXPECT_TRUE(robotPositionCollidesWithObstacles(centerPosition, obstacles));
}

// 6: CollisionDetectedAgainstAnyEnabledObstacleInList
TEST(RobotCollisionTest, CollisionDetectedAgainstAnyEnabledObstacleInList)
{
    const std::vector<BoxObstacle> obstacles = {
        BoxObstacle{Vec3{10.0F, 0.4F, 10.0F}, Vec3{0.8F, 0.8F, 0.8F}, true}, // far - no hit
        BoxObstacle{Vec3{0.0F, 0.4F, 2.0F}, Vec3{0.8F, 0.8F, 0.8F}, true}    // near - hit
    };

    EXPECT_TRUE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, 2.0F}, obstacles));
}

// 7: EmptyObstacleListNeverCollides
TEST(RobotCollisionTest, EmptyObstacleListNeverCollides)
{
    const std::vector<BoxObstacle> obstacles;

    EXPECT_FALSE(robotPositionCollidesWithObstacles(Vec3{0.0F, 0.125F, 0.0F}, obstacles));
}

// --- Phase 13W: desk-object collision-proxy tests ---
//
// Each DeskObject (semantic type + visual dispatch, VirtualWorld.hpp) is
// registered together with a plain BoxObstacle of identical position/size
// (VirtualWorld::addDeskObject()) - RobotCollision/VirtualObstacleSensorArray
// never know DeskObject exists at all; these tests prove the collision-
// proxy handoff between the two actually holds for the real demo scene.

// 8: KeyboardProducesExpectedObstacleBounds
TEST(RobotCollisionTest, KeyboardProducesExpectedObstacleBounds)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> obstacle = findDeskObjectObstacle(world, DeskObjectType::Keyboard);
    ASSERT_TRUE(obstacle.has_value());
    EXPECT_GT(obstacle->size.x, 0.0F);
    EXPECT_GT(obstacle->size.z, 0.0F);
    EXPECT_TRUE(obstacle->enabled);
}

// 9: MouseProducesExpectedObstacleBounds
TEST(RobotCollisionTest, MouseProducesExpectedObstacleBounds)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> obstacle = findDeskObjectObstacle(world, DeskObjectType::Mouse);
    ASSERT_TRUE(obstacle.has_value());
    EXPECT_GT(obstacle->size.x, 0.0F);
    EXPECT_GT(obstacle->size.z, 0.0F);
    EXPECT_TRUE(obstacle->enabled);
}

// 10: MonitorProducesExpectedObstacleBounds
TEST(RobotCollisionTest, MonitorProducesExpectedObstacleBounds)
{
    // Phase 13W final workspace redesign: the production desk holds only
    // Monitor/Keyboard/Mouse - Mug is no longer instantiated (see
    // VirtualWorld.cpp's constructor), so this test now covers the third
    // and last production desk object rather than a removed one.
    VirtualWorld world;
    const std::optional<BoxObstacle> obstacle = findDeskObjectObstacle(world, DeskObjectType::Monitor);
    ASSERT_TRUE(obstacle.has_value());
    EXPECT_GT(obstacle->size.x, 0.0F);
    EXPECT_GT(obstacle->size.z, 0.0F);
    EXPECT_TRUE(obstacle->enabled);
}

// 11: MonitorFootprintIsStandSizedNotFullScreenVolume
//
// Collision (like every check in this file) is purely an X/Z footprint
// test - Y/height never participates - so a monitor's registered
// obstacle can never physically block the robot at "screen height"
// regardless of how tall Renderer3D draws the screen+neck visually. This
// asserts the STAND footprint itself is also kept small and deliberate
// (not, say, accidentally sized to a whole desk), matching the brief's
// "physical footprint is not the entire screen visual volume" concern.
TEST(RobotCollisionTest, MonitorFootprintIsStandSizedNotFullScreenVolume)
{
    // Phase 13W human-visual-redesign v2: the monitor's screen is now the
    // visually DOMINANT desk object (~3.0 x 1.65 world units - see
    // Renderer3D::drawMonitor()'s own fixed screen constants), so a
    // simple absolute-area threshold would need constant re-tuning. This
    // instead compares against the Keyboard's own registered footprint
    // (2.3 x 0.75 = 1.725 sq units) - a stable, self-documenting proof
    // that the monitor's STAND footprint stays modest relative to another
    // real desk object, never anywhere close to the screen's own visual
    // area.
    VirtualWorld world;
    const std::optional<BoxObstacle> monitorObstacle = findDeskObjectObstacle(world, DeskObjectType::Monitor);
    const std::optional<BoxObstacle> keyboardObstacle = findDeskObjectObstacle(world, DeskObjectType::Keyboard);
    ASSERT_TRUE(monitorObstacle.has_value());
    ASSERT_TRUE(keyboardObstacle.has_value());
    const float monitorFootprintArea = monitorObstacle->size.x * monitorObstacle->size.z;
    const float keyboardFootprintArea = keyboardObstacle->size.x * keyboardObstacle->size.z;
    EXPECT_LT(monitorFootprintArea, keyboardFootprintArea);
}

// 12: DisabledDeskObjectObstacleIsIgnoredConsistently
TEST(RobotCollisionTest, DisabledDeskObjectObstacleIsIgnoredConsistently)
{
    // Phase 13W human-visual-redesign v2: named semantic lookup, not a
    // raw magic index - desk objects now occupy obstacles()[0..5]
    // directly (the old 4-generic-obstacle offset is gone entirely).
    VirtualWorld world;
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    ASSERT_TRUE(world.setObstacleEnabled(keyboardIndex, false));

    const Vec3& deskObjectPosition = world.obstacles()[keyboardIndex].position;
    EXPECT_FALSE(robotPositionCollidesWithObstacles(deskObjectPosition, world.obstacles()));
}

// 13: DeskObjectBoundsWorkWithVirtualObstacleSensorArray
TEST(RobotCollisionTest, DeskObjectBoundsWorkWithVirtualObstacleSensorArray)
{
    // Arrange: robot placed to face directly into the keyboard's footprint
    // (see VirtualWorld.cpp's Phase 13W desk-cluster placement).
    VirtualWorld world;
    const std::optional<BoxObstacle> keyboard = findDeskObjectObstacle(world, DeskObjectType::Keyboard);
    ASSERT_TRUE(keyboard.has_value());
    world.setRobotPosition(Vec3{keyboard->position.x, 0.125F, keyboard->position.z - 1.0F});
    world.setRobotHeading(0.0F); // faces +Z, straight at the keyboard

    // Act
    const VirtualObstacleSensorArray sensorArray(world);
    const ObstacleSensorArrayReadings readings = sensorArray.readings();

    // Assert: the real sensor array perceives the desk object exactly like
    // any other BoxObstacle - it has no notion of "desk object" at all.
    EXPECT_TRUE(readings.anyDetected() || readings.frontCenterDistance.has_value());
}

// 14: RobotCollisionRejectsPenetrationIntoDeskObject
TEST(RobotCollisionTest, RobotCollisionRejectsPenetrationIntoDeskObject)
{
    VirtualWorld world;
    const std::optional<BoxObstacle> monitor = findDeskObjectObstacle(world, DeskObjectType::Monitor);
    ASSERT_TRUE(monitor.has_value());

    // At the monitor stand's own center - well inside its collision
    // footprint.
    EXPECT_TRUE(robotPositionCollidesWithObstacles(monitor->position, world.obstacles()));
}

// --- Phase 13W final workspace redesign: robot scale ---
//
// Ratio/range assertions, not brittle exact numbers (this phase's own
// brief) - each of these documents an INTENT ("miniature, ~8-10cm robot")
// that stays true even if the exact constants are nudged again later,
// rather than pinning today's specific values.

// 15: RobotWidthMatchesMiniatureScaleIntent
TEST(RobotCollisionTest, RobotWidthMatchesMiniatureScaleIntent)
{
    EXPECT_GT(RobotDimensions::kBodyWidth, 0.38F);
    EXPECT_LT(RobotDimensions::kBodyWidth, 0.45F);
}

// 16: RobotLengthMatchesMiniatureScaleIntent
TEST(RobotCollisionTest, RobotLengthMatchesMiniatureScaleIntent)
{
    EXPECT_GT(RobotDimensions::kBodyLength, 0.48F);
    EXPECT_LT(RobotDimensions::kBodyLength, 0.58F);
}

// 17: CollisionRadiusContainsFullRobotFootprint
//
// The enclosing circle must never be smaller than the rectangle's own
// half-diagonal - otherwise a corner of the physical body could poke
// outside the collision test's own footprint at some heading.
TEST(RobotCollisionTest, CollisionRadiusContainsFullRobotFootprint)
{
    const float halfDiagonal = std::sqrt(((RobotDimensions::kBodyWidth / 2.0F) * (RobotDimensions::kBodyWidth / 2.0F)) +
                                          ((RobotDimensions::kBodyLength / 2.0F) * (RobotDimensions::kBodyLength / 2.0F)));
    EXPECT_GE(kRobotCollisionRadius, halfDiagonal);
}

// 18: CollisionRadiusIsNotExcessivelyConservative
//
// The safety margin on top of the half-diagonal should stay small (a
// "little", not a second robot-width) - otherwise collision checks would
// reject perfectly navigable gaps.
TEST(RobotCollisionTest, CollisionRadiusIsNotExcessivelyConservative)
{
    const float halfDiagonal = std::sqrt(((RobotDimensions::kBodyWidth / 2.0F) * (RobotDimensions::kBodyWidth / 2.0F)) +
                                          ((RobotDimensions::kBodyLength / 2.0F) * (RobotDimensions::kBodyLength / 2.0F)));
    EXPECT_LT(kRobotCollisionRadius - halfDiagonal, 0.10F);
}

// 19: WheelTrackMatchesBodyWidth
TEST(RobotCollisionTest, WheelTrackMatchesBodyWidth)
{
    EXPECT_FLOAT_EQ(DifferentialDrive::kDefaultWheelTrack, RobotDimensions::kBodyWidth);
}

// 20: ObstacleSensorOriginsMatchRobotGeometry
//
// The left/right ray origins must stay INSIDE the body's own half-width
// (a small inset from the edge, per VirtualObstacleSensorArray's own
// kLateralInset) - never wider than the physical robot itself.
TEST(RobotCollisionTest, ObstacleSensorOriginsMatchRobotGeometry)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    const VirtualObstacleSensorArray array(world);
    const Vec3 frontLeftOrigin = array.rayOrigin(robot::visual::ObstacleRayPosition::FrontLeft);
    const float lateralOffset = std::fabs(frontLeftOrigin.x - world.robotPose().position.x);
    EXPECT_GT(lateralOffset, 0.0F);
    EXPECT_LT(lateralOffset, RobotDimensions::kBodyWidth / 2.0F);
}

// 21: BypassDistanceScalesWithCollisionRadius
TEST(RobotCollisionTest, BypassDistanceScalesWithCollisionRadius)
{
    EXPECT_FLOAT_EQ(ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits, 2.0F * kRobotCollisionRadius);
}

// 22: DockSlotFitsRobotWithMargin
//
// The dock's own outer footprint (basePlatform().size) must be wider than
// the robot's own body width, with room to spare - "slightly larger than
// the robot," not an exact or too-tight fit.
TEST(RobotCollisionTest, DockSlotFitsRobotWithMargin)
{
    VirtualWorld world;
    const auto& base = world.basePlatform();
    EXPECT_GT(base.size.x, RobotDimensions::kBodyWidth);
    EXPECT_LT(base.size.x, RobotDimensions::kBodyWidth * 2.5F);
}
