#include <gtest/gtest.h>

#include "robot/RobotState.hpp"
#include "robot/visual/MapResetController.hpp"

namespace
{

using robot::RobotState;
using robot::visual::MapResetController;
using robot::visual::MapResetRequestOutcome;

} // namespace

// 15: FirstNOnlyRequestsConfirmation
TEST(MapResetControllerTest, FirstNOnlyRequestsConfirmation)
{
    MapResetController controller;

    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::Idle);

    EXPECT_EQ(outcome, MapResetRequestOutcome::ConfirmationRequested);
    EXPECT_TRUE(controller.confirmationPending());
}

TEST(MapResetControllerTest, FirstNFromReadyAlsoRequestsConfirmation)
{
    MapResetController controller;

    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::Ready);

    EXPECT_EQ(outcome, MapResetRequestOutcome::ConfirmationRequested);
    EXPECT_TRUE(controller.confirmationPending());
}

// 16: SecondNWithinWindowConfirmsReset
TEST(MapResetControllerTest, SecondNWithinWindowConfirmsReset)
{
    MapResetController controller;
    ASSERT_EQ(controller.requestKeyPress(RobotState::Idle), MapResetRequestOutcome::ConfirmationRequested);

    controller.update(MapResetController::kConfirmationWindowSeconds * 0.5F);
    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::Idle);

    EXPECT_EQ(outcome, MapResetRequestOutcome::Confirmed);
    EXPECT_FALSE(controller.confirmationPending());
}

// 17: ConfirmationExpiresWithoutReset
TEST(MapResetControllerTest, ConfirmationExpiresWithoutReset)
{
    MapResetController controller;
    ASSERT_EQ(controller.requestKeyPress(RobotState::Idle), MapResetRequestOutcome::ConfirmationRequested);

    controller.update(MapResetController::kConfirmationWindowSeconds + 0.01F);

    EXPECT_FALSE(controller.confirmationPending());

    // A press after expiry is a fresh FIRST press, never a stale confirm.
    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::Idle);
    EXPECT_EQ(outcome, MapResetRequestOutcome::ConfirmationRequested);
}

// 14: ResetRejectedDuringActiveMission
TEST(MapResetControllerTest, ResetRejectedDuringActiveMission)
{
    MapResetController controller;

    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::Moving);

    EXPECT_EQ(outcome, MapResetRequestOutcome::RejectedActiveMission);
    EXPECT_FALSE(controller.confirmationPending());
}

TEST(MapResetControllerTest, ActiveMissionStateCancelsAlreadyPendingConfirmation)
{
    MapResetController controller;
    ASSERT_EQ(controller.requestKeyPress(RobotState::Ready), MapResetRequestOutcome::ConfirmationRequested);

    // The robot started a mission while the confirmation was still armed -
    // the confirming press must never be honored underneath an active
    // mission, even though it is technically the SECOND press.
    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::Moving);

    EXPECT_EQ(outcome, MapResetRequestOutcome::RejectedActiveMission);
    EXPECT_FALSE(controller.confirmationPending());
}

TEST(MapResetControllerTest, ResetRejectedDuringReturningHome)
{
    MapResetController controller;

    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::ReturningHome);

    EXPECT_EQ(outcome, MapResetRequestOutcome::RejectedActiveMission);
}

TEST(MapResetControllerTest, ResetRejectedWhileWaitingForObstacleClear)
{
    MapResetController controller;

    const MapResetRequestOutcome outcome = controller.requestKeyPress(RobotState::WaitingForObstacleClear);

    EXPECT_EQ(outcome, MapResetRequestOutcome::RejectedActiveMission);
}
