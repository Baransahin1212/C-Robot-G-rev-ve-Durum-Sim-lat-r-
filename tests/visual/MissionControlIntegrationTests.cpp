#include <cmath>

#include <gtest/gtest.h>

#include "robot/CompositePollingEventSource.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/HomeArrivalEventSource.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/HomeZoneMonitor.hpp"
#include "robot/visual/MissionControlEventSource.hpp"
#include "robot/visual/MissionTask.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::CompositePollingEventSource;
using robot::HardwareEventSource;
using robot::ReturnHomeReason;
using robot::RobotController;
using robot::RobotRuntime;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::RuntimeStepResult;
using robot::visual::BasePlatform;
using robot::visual::CliffSensorReadings;
using robot::visual::deriveMissionTask;
using robot::visual::DriveAuthority;
using robot::visual::ForwardClearanceProbe;
using robot::visual::HomeArrivalEventSource;
using robot::visual::HomeNavigationOutput;
using robot::visual::HomeNavigationState;
using robot::visual::HomeNavigator;
using robot::visual::HomeZoneMonitor;
using robot::visual::MissionControlEventSource;
using robot::visual::MissionTask;
using robot::visual::ReactiveObstacleAvoidance;
using robot::visual::TableEdgeSafetyController;
using robot::visual::Vec3;
using robot::visual::VirtualCliffSensor;
using robot::visual::VirtualDriveCommand;
using robot::visual::VirtualRobotHardware;
using robot::visual::VirtualWorld;
using robot::visual::WheelSpeeds;

// Disables every default demo obstacle - same convention
// VirtualRobotHardwareTests.cpp already uses.
void disableAllObstacles(VirtualWorld& world)
{
    for (std::size_t i = 0; i < world.obstacles().size(); ++i)
    {
        world.setObstacleEnabled(i, false);
    }
}

// Drives the ENTIRE real Phase 13U production stack one frame at a time,
// in exactly main3d.cpp's own order: runtime.step() -> avoidance ->
// table-edge safety -> HomeNavigator -> HomeZoneMonitor -> sync overrides
// (Safety, Autonomous, Navigation) -> hardware.update(dt). Exists purely
// to avoid repeating this ~30-line block identically in every test below
// - it performs no decisions of its own beyond what main3d.cpp itself
// already does.
struct MissionControlHarness
{
    explicit MissionControlHarness(VirtualWorld& world)
        : hardware(world)
        , hardwareEventSource(hardware)
        , missionControl()
        , homeNavigator()
        , homeArrivalEventSource(homeNavigator)
        , homeZone()
        , innerHardwareGroup(hardwareEventSource, homeArrivalEventSource)
        , innerAutoGroup(innerHardwareGroup, homeZone)
        , compositeSource(missionControl, innerAutoGroup)
        , stateMachine()
        , controller(hardware)
        , runtime(compositeSource, stateMachine, controller)
        , avoidance()
        , clearanceProbe(world)
        , cliffSensor(world)
        , tableEdgeSafety()
        , avoidanceEnabled(true)
        , world_(world)
    {
    }

    VirtualRobotHardware hardware;
    HardwareEventSource hardwareEventSource;
    MissionControlEventSource missionControl;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource;
    HomeZoneMonitor homeZone;
    CompositePollingEventSource innerHardwareGroup;
    CompositePollingEventSource innerAutoGroup;
    CompositePollingEventSource compositeSource;
    RobotStateMachine stateMachine;
    RobotController controller;
    RobotRuntime runtime;
    ReactiveObstacleAvoidance avoidance;
    ForwardClearanceProbe clearanceProbe;
    VirtualCliffSensor cliffSensor;
    TableEdgeSafetyController tableEdgeSafety;
    bool avoidanceEnabled;
    VirtualWorld& world_;

    MissionTask currentTask() const
    {
        return deriveMissionTask(stateMachine.currentState(), stateMachine.returnHomeReason());
    }

    float distanceToBase() const
    {
        const BasePlatform& base = world_.basePlatform();
        const float dx = base.position.x - world_.robotPose().position.x;
        const float dz = base.position.z - world_.robotPose().position.z;
        return std::sqrt((dx * dx) + (dz * dz));
    }

    void driveFrame(float dt = 0.05F)
    {
        runtime.step();

        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();
        const bool triggerAvoidance = avoidanceEnabled &&
                                       stateMachine.currentState() == RobotState::WaitingForObstacleClear &&
                                       hardware.obstacleDetected();
        avoidance.update(avoidanceEnabled, triggerAvoidance, forwardCorridorClear);

        const CliffSensorReadings cliffReadings = cliffSensor.readings();
        tableEdgeSafety.update(cliffReadings, world_.robotPose(), world_.tableSurface());

        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world_.robotPose(), world_.basePlatform(), navigationEnabled);

        const bool roamActive = currentTask() == MissionTask::Roam;
        homeZone.update(world_.robotPose(), world_.basePlatform(), roamActive);

        if (tableEdgeSafety.active())
        {
            const WheelSpeeds recovery = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recovery.left, recovery.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        if (avoidance.active())
        {
            const WheelSpeeds turn = avoidance.avoidanceWheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turn.left, turn.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        hardware.update(dt);
    }
};

} // namespace

// --- Full Start-Roam integration test ---
TEST(MissionControlIntegrationTest, FullClosedLoopStartRoamIntegrationTest)
{
    // Arrange
    VirtualWorld world;
    MissionControlHarness h(world);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Idle);

    // Act: request Start Roam through MissionControlEventSource - never a
    // direct FSM mutation.
    h.missionControl.requestStartRoam(h.stateMachine.currentState());

    // Assert: first runtime step consumes ScenarioLoaded.
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Ready);

    // Assert: next normal runtime step consumes StartMission.
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);
    EXPECT_EQ(h.hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    EXPECT_EQ(h.hardware.driveAuthority(), DriveAuthority::Fsm);

    // Assert: the robot physically moves - real DifferentialDrive
    // kinematics, never a teleport.
    const float zBefore = world.robotPose().position.z;
    for (int i = 0; i < 10; ++i)
    {
        h.driveFrame();
    }
    EXPECT_NE(world.robotPose().position.z, zBefore);
}

// --- Stop / restart integration test ---
TEST(MissionControlIntegrationTest, StopRestartIntegrationTest)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    MissionControlHarness h(world);

    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    h.runtime.step();
    h.runtime.step();
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    const float zAfterStart = world.robotPose().position.z;
    for (int i = 0; i < 10; ++i)
    {
        h.driveFrame();
    }
    ASSERT_GT(world.robotPose().position.z, zAfterStart);

    // Act: request Stop Task.
    h.missionControl.requestStopTask();
    const RuntimeStepResult stopResult = h.runtime.step();

    // Assert: consumed normally, FSM -> Ready.
    EXPECT_EQ(stopResult, RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(h.hardware.currentCommand(), VirtualDriveCommand::Stopped);

    const float zAfterStop = world.robotPose().position.z;
    h.driveFrame();
    EXPECT_FLOAT_EQ(world.robotPose().position.z, zAfterStop);

    // Act: request Start Roam again (from Ready this time).
    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    const RuntimeStepResult restartResult = h.runtime.step();

    // Assert: StartMission accepted, FSM -> Moving, robot travels again.
    EXPECT_EQ(restartResult, RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);
    const float zAfterRestart = world.robotPose().position.z;
    for (int i = 0; i < 10; ++i)
    {
        h.driveFrame();
    }
    EXPECT_GT(world.robotPose().position.z, zAfterRestart);
}

// --- Stop-Return-Home integration test ---
TEST(MissionControlIntegrationTest, StopReturnHomeIntegrationTest)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    MissionControlHarness h(world);

    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    h.runtime.step();
    h.runtime.step();
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    // Act: request Return Home.
    h.missionControl.requestReturnHome();
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::ReturningHome);

    // Drive a few frames so Navigation genuinely engages before stopping.
    for (int i = 0; i < 10; ++i)
    {
        h.driveFrame();
    }
    ASSERT_EQ(h.hardware.driveAuthority(), DriveAuthority::Navigation);

    // Act: request Stop Task before arrival.
    h.missionControl.requestStopTask();
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);

    // Assert: FSM Ready, reason cleared.
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(h.stateMachine.returnHomeReason(), ReturnHomeReason::None);

    // Assert: Navigation clears, robot stops.
    h.driveFrame();
    EXPECT_FALSE(h.hardware.navigationOverrideActive());
    EXPECT_EQ(h.hardware.driveAuthority(), DriveAuthority::Fsm);
    EXPECT_EQ(h.homeNavigator.state(), HomeNavigationState::Inactive);
    const WheelSpeeds stopped = h.hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(stopped.left, 0.0F);
    EXPECT_FLOAT_EQ(stopped.right, 0.0F);

    // Act / Assert: Return Home again works.
    h.missionControl.requestReturnHome();
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    EXPECT_EQ(h.stateMachine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(h.stateMachine.returnHomeReason(), ReturnHomeReason::UserRequest);
}

// --- Stop-during-Safety integration test ---
TEST(MissionControlIntegrationTest, StopDuringSafetyIntegrationTest)
{
    // Arrange: front corner will approach the table edge while Roaming
    // straight ahead - same positioning strategy as the Phase 13S edge-
    // safety integration tests.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 5.6F});
    world.setRobotHeading(0.0F);
    MissionControlHarness h(world);

    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    h.runtime.step();
    h.runtime.step();
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    bool everSafety = false;
    for (int frame = 0; frame < 500 && !everSafety; ++frame)
    {
        h.driveFrame();
        if (h.hardware.driveAuthority() == DriveAuthority::Safety)
        {
            everSafety = true;
        }
    }
    ASSERT_TRUE(everSafety);

    // Act: Stop Task while Safety is actively recovering.
    h.missionControl.requestStopTask();
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Ready);

    // Assert: Safety authority MUST remain until recovery genuinely
    // completes - mission cancellation never aborts physical safety.
    EXPECT_EQ(h.hardware.driveAuthority(), DriveAuthority::Safety);

    bool everReleased = false;
    for (int frame = 0; frame < 4000 && !everReleased; ++frame)
    {
        h.driveFrame();
        if (!h.tableEdgeSafety.active() && h.hardware.driveAuthority() == DriveAuthority::Fsm)
        {
            everReleased = true;
        }
    }

    // Assert: after Safety releases, effective FSM stop.
    ASSERT_TRUE(everReleased);
    EXPECT_EQ(h.stateMachine.currentState(), RobotState::Ready);
    const WheelSpeeds finalSpeeds = h.hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(finalSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(finalSpeeds.right, 0.0F);
}

// --- Stop-during-Avoidance integration test ---
TEST(MissionControlIntegrationTest, StopDuringAvoidanceIntegrationTest)
{
    // Arrange: default demo world - the blocking obstacle sits directly
    // ahead of the robot's real forward path.
    VirtualWorld world;
    MissionControlHarness h(world);

    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    h.runtime.step();
    h.runtime.step();
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    bool everAvoidanceActive = false;
    for (int frame = 0; frame < 2000 && !everAvoidanceActive; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
        if (h.avoidance.active())
        {
            everAvoidanceActive = true;
        }
    }
    ASSERT_TRUE(everAvoidanceActive);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::WaitingForObstacleClear);

    // Act: Stop Task while avoidance is actively turning.
    h.missionControl.requestStopTask();
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Ready);

    // Assert: avoidance does not remain a stale physical authority
    // forever - it keeps turning (its own release condition is
    // independent of triggerAvoidance/FSM state, see
    // ReactiveObstacleAvoidance::update()) until the body corridor is
    // genuinely clear, then releases; final effective wheels stop.
    bool everCleared = false;
    for (int frame = 0; frame < 2000 && !everCleared; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
        if (!h.avoidance.active() && h.hardware.driveAuthority() == DriveAuthority::Fsm)
        {
            everCleared = true;
        }
    }

    ASSERT_TRUE(everCleared);
    const WheelSpeeds finalSpeeds = h.hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(finalSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(finalSpeeds.right, 0.0F);
    EXPECT_FALSE(h.hardware.collidedLastUpdate());
}

// --- Home-Zone closed-loop integration test ---
TEST(MissionControlIntegrationTest, HomeZoneClosedLoopIntegrationTest)
{
    // Arrange: positioned on the same X as the base, 5 units away (well
    // inside the 9.0F exit radius) and facing further away, so a plain
    // Roam forward travel deterministically crosses the exit radius
    // within a bounded number of frames.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{4.0F, 0.125F, -1.0F});
    world.setRobotHeading(180.0F);
    MissionControlHarness h(world);

    // 1-2: Start Roam through Mission Control.
    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    // 3-6: drive away until the zone naturally triggers an automatic
    // Return Home - exactly one ReturnHomeRequested, through
    // HomeZoneMonitor's own pollEvent(), never injected directly.
    for (int frame = 0; frame < 3000 && h.stateMachine.currentState() == RobotState::Moving; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
    }
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(h.stateMachine.returnHomeReason(), ReturnHomeReason::UserRequest);
    EXPECT_FALSE(h.homeZone.armed());

    // 7: Navigation authority becomes active.
    h.driveFrame();
    EXPECT_EQ(h.hardware.driveAuthority(), DriveAuthority::Navigation);

    // 8-9: robot turns toward base, distance eventually decreases.
    float previousDistance = h.distanceToBase();
    bool everDecreased = false;
    for (int frame = 0; frame < 3000 && h.stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
        const float current = h.distanceToBase();
        if (current < previousDistance)
        {
            everDecreased = true;
        }
        previousDistance = current;
    }
    EXPECT_TRUE(everDecreased);

    // 10-12: arrival radius reached, HomeReached emitted naturally,
    // FSM -> Ready.
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Ready);
    EXPECT_LE(h.distanceToBase(), HomeNavigator::kHomeArrivalRadius);

    // 13-14: Navigation clears, robot stops.
    h.driveFrame();
    EXPECT_FALSE(h.hardware.navigationOverrideActive());
    const WheelSpeeds stopped = h.hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(stopped.left, 0.0F);
    EXPECT_FLOAT_EQ(stopped.right, 0.0F);

    // 15: the monitor re-arms - but only once evaluated while Roam is
    // active again (HomeZoneMonitor freezes entirely, including the
    // armed/disarmed latch, while the task is not Roam - see
    // DisabledWhenTaskIsNotRoam - so the monitor is still legitimately
    // Disarmed here, throughout the just-finished Return Home trip, and
    // only re-arms once a fresh Roam evaluates it again below).
    EXPECT_FALSE(h.homeZone.armed());

    // 16-17: Start Roam again - repositioned to the same kind of open
    // path away from base used at the top of this test (representing a
    // fresh Roam session, not a mid-mission event), so a second
    // excursion is exercised deterministically. A second automatic
    // Return Home can trigger.
    world.setRobotPosition(Vec3{4.0F, 0.125F, -1.0F});
    world.setRobotHeading(180.0F);
    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted); // StartMission only (Ready)
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    // The very first Roam frame already re-arms the monitor - the robot
    // is still well within the 6.0F rearm radius at (4, -1), distance 5.
    h.driveFrame();
    EXPECT_TRUE(h.homeZone.armed());

    for (int frame = 0; frame < 3000 && h.stateMachine.currentState() == RobotState::Moving; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
    }
    EXPECT_EQ(h.stateMachine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(h.stateMachine.returnHomeReason(), ReturnHomeReason::UserRequest);
}

// --- Obstacle during automatic Home-Zone return ---
//
// The obstacle is cleared via a direct world mutation once
// WaitingForObstacleClear is reached (identical to RobotSimulator3D's own
// `O` key) rather than by driving a real ReactiveObstacleAvoidance turn to
// convergence - this is exactly the same technique Phase 13T's own
// ObstacleDuringReturnHomeInterruptsThenResumesNavigation test already
// uses (VirtualRobotHardwareTests.cpp), reused rather than duplicated
// with new architecture. Direct experimentation while writing this test
// found that genuine avoidance turning CAN resonate indefinitely against
// HomeNavigator's continuous re-aiming in this V1 design, when the two
// repeatedly disagree about the same heading band (WaitingForObstacleClear
// <-> ReturningHome forever, zero net progress) - a real, previously-
// undiscovered V1 limitation, not something either this test or Phase
// 13T's own precedent actually exercises; see docs/technical-decisions.md
// (Phase 13U) for the full writeup. This test still proves everything the
// brief requires: an obstacle genuinely interrupts an AUTOMATICALLY-
// triggered Return Home, and Navigation genuinely resumes and completes
// afterward, through the exact same ReturningHome + ObstacleDetected ->
// WaitingForObstacleClear -> ObstacleCleared -> ReturningHome transitions
// as every other test - never a direct state mutation.
TEST(MissionControlIntegrationTest, ObstacleDuringAutomaticHomeZoneReturnIntegrationTest)
{
    // Arrange: same departure setup as HomeZoneClosedLoopIntegrationTest -
    // already known to safely cross the exit radius and return without
    // touching table-edge recovery.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{4.0F, 0.125F, -1.0F});
    world.setRobotHeading(180.0F);
    world.setObstaclePosition(0, Vec3{4.0F, 0.4F, 2.0F});
    world.setObstacleEnabled(0, true);
    MissionControlHarness h(world);

    h.missionControl.requestStartRoam(h.stateMachine.currentState());
    h.runtime.step();
    h.runtime.step();
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::Moving);

    // Roam away until the zone triggers automatically.
    for (int frame = 0; frame < 3000 && h.stateMachine.currentState() == RobotState::Moving; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
    }
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(h.stateMachine.returnHomeReason(), ReturnHomeReason::UserRequest);

    // Continue toward base until the obstacle interrupts navigation -
    // reusing the exact Phase 13T ReturningHome + ObstacleDetected ->
    // WaitingForObstacleClear behavior, never a separate mechanism.
    for (int frame = 0; frame < 3000 && h.stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
    }
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    EXPECT_EQ(h.stateMachine.returnHomeReason(), ReturnHomeReason::UserRequest);
    EXPECT_EQ(h.homeNavigator.state(), HomeNavigationState::Inactive);

    // Act: clear the obstacle - identical world-only mutation to
    // RobotSimulator3D's "O" key; never a manual Event injection.
    world.setObstacleEnabled(0, false);

    // Assert: HardwareEventSource observes the edge and emits
    // ObstacleCleared; RobotStateMachine resumes ReturningHome (the
    // remembered resumeState_) - never plain Moving.
    ASSERT_EQ(h.runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(h.stateMachine.currentState(), RobotState::ReturningHome);

    // Continue until the robot genuinely reaches home - HomeNavigator
    // recomputes a fresh target from wherever the robot ended up.
    for (int frame = 0; frame < 3000 && h.stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        h.driveFrame();
        ASSERT_FALSE(h.hardware.collidedLastUpdate());
    }
    EXPECT_EQ(h.stateMachine.currentState(), RobotState::Ready);
    EXPECT_LE(h.distanceToBase(), HomeNavigator::kHomeArrivalRadius);
}
