#include <gtest/gtest.h>

#include "robot/visual/ManualDriveInput.hpp"

namespace
{

using robot::visual::computeManualWheelSpeeds;
using robot::visual::WheelSpeeds;

constexpr float kWheelSpeed = 1.0F;

} // namespace

// 1: NoKeysHeldProducesZero
TEST(ManualDriveInputTest, NoKeysHeldProducesZero)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/false, /*down=*/false, /*left=*/false, /*right=*/false, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 2: UpOnlyProducesEqualPositive
TEST(ManualDriveInputTest, UpOnlyProducesEqualPositive)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/true, /*down=*/false, /*left=*/false, /*right=*/false, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 1.0F);
    EXPECT_FLOAT_EQ(speeds.right, 1.0F);
}

// 3: DownOnlyProducesEqualNegative
TEST(ManualDriveInputTest, DownOnlyProducesEqualNegative)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/false, /*down=*/true, /*left=*/false, /*right=*/false, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, -1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);
}

// 4: LeftOnlyProducesTurnLeftPattern
TEST(ManualDriveInputTest, LeftOnlyProducesTurnLeftPattern)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/false, /*down=*/false, /*left=*/true, /*right=*/false, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, -1.0F);
    EXPECT_FLOAT_EQ(speeds.right, 1.0F);
}

// 5: RightOnlyProducesTurnRightPattern
TEST(ManualDriveInputTest, RightOnlyProducesTurnRightPattern)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/false, /*down=*/false, /*left=*/false, /*right=*/true, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);
}

// 6: UpAndDownCancelToZero
TEST(ManualDriveInputTest, UpAndDownCancelToZero)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/true, /*down=*/true, /*left=*/false, /*right=*/false, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 7: LeftAndRightCancelToZero
TEST(ManualDriveInputTest, LeftAndRightCancelToZero)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/false, /*down=*/false, /*left=*/true, /*right=*/true, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 8: XOverridesUp
TEST(ManualDriveInputTest, XOverridesUp)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/true, /*down=*/false, /*left=*/false, /*right=*/false, /*x=*/true,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 9: XOverridesAllDirectionsSimultaneously
TEST(ManualDriveInputTest, XOverridesAllDirectionsSimultaneously)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/true, /*down=*/true, /*left=*/true, /*right=*/true, /*x=*/true, kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 10: XAloneProducesZero
TEST(ManualDriveInputTest, XAloneProducesZero)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/false, /*down=*/false, /*left=*/false, /*right=*/false, /*x=*/true,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 11: UpAndRightCombineIntoArc
TEST(ManualDriveInputTest, UpAndRightCombineIntoArc)
{
    const WheelSpeeds speeds =
        computeManualWheelSpeeds(/*up=*/true, /*down=*/false, /*left=*/false, /*right=*/true, /*x=*/false,
                                  kWheelSpeed);

    EXPECT_FLOAT_EQ(speeds.left, 2.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}
