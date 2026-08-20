#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::BoxObstacle;
using robot::visual::kRobotCollisionRadius;
using robot::visual::robotPositionCollidesWithObstacles;
using robot::visual::Vec3;

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
