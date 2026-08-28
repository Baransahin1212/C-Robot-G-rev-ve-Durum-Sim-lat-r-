#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/ExplorationMapStorage.hpp"
#include "robot/visual/ExplorationSessionReset.hpp"
#include "robot/visual/FrontierExplorer.hpp"

namespace
{

using robot::RobotState;
using robot::RobotStateMachine;
using robot::visual::CoverageTrail;
using robot::visual::ExplorationCompletion;
using robot::visual::ExplorationCompletionSignal;
using robot::visual::ExplorationMap;
using robot::visual::ExplorationMapStorage;
using robot::visual::ExplorationSessionState;
using robot::visual::FrontierExplorer;
using robot::visual::FrontierTarget;
using robot::visual::GridCoord;
using robot::visual::MapCell;
using robot::visual::MapLoadResult;
using robot::visual::RobotPose;
using robot::visual::TableSurface;
using robot::visual::Vec3;
using robot::visual::WaypointNavigator;
using robot::visual::WaypointNavigatorOutput;
using robot::visual::WaypointNavigatorState;
using robot::visual::resetExplorationSession;

TableSurface testBounds()
{
    return TableSurface{-2.4F, 2.4F, -2.4F, 2.4F};
}

// Never the real runtime map path - always a TEST_OUTPUT_DIR-rooted
// temporary file, mirroring ExplorationMapStorageTests.cpp's own
// convention exactly ("Tests must use temporary files. Never write to the
// real user map path.").
std::string testFilePath(const std::string& name)
{
    const std::filesystem::path dir = std::filesystem::path(TEST_OUTPUT_DIR) / "exploration_session_reset";
    std::filesystem::create_directories(dir);
    return (dir / name).string();
}

void markAllFree(ExplorationMap& map)
{
    for (int row = 0; row < map.height(); ++row)
    {
        for (int col = 0; col < map.width(); ++col)
        {
            map.markFree(col, row);
        }
    }
}

// Every piece of state one exploration-mapping SESSION owns (mirrors
// ExplorationSessionState's own field list exactly) - a plain owning
// bundle tests construct, mutate to simulate an "old, dirty" session, then
// bind into an ExplorationSessionState for resetExplorationSession() to
// act on. Never used by production code - main3d.cpp owns these as
// separate local variables (see that file's own KEY_N handler).
struct SessionHarness
{
    ExplorationMap map;
    CoverageTrail trail;
    WaypointNavigator navigator;
    ExplorationCompletionSignal completionSignal;
    std::optional<FrontierTarget> currentFrontierTarget;
    std::vector<GridCoord> frontierBlacklist;
    int framesSinceLastFrontierAttempt = 0;
    int consecutiveNoTargetFound = 0;
    std::vector<GridCoord> returnHomeBlockedCells;
    WaypointNavigatorOutput previousNavOutput;
    float bestDistanceToHomeThisSession = 0.0F;
    int framesSinceDistanceImproved = 0;
    bool mapAlreadyCompleteNoticeActive = false;
    float mapSaveTimer = 0.0F;
    bool mapWasLoadedAtStartup = false;

    explicit SessionHarness(const TableSurface& bounds) : map(bounds) {}

    ExplorationSessionState state()
    {
        return ExplorationSessionState{
            map,
            trail,
            navigator,
            completionSignal,
            currentFrontierTarget,
            frontierBlacklist,
            framesSinceLastFrontierAttempt,
            consecutiveNoTargetFound,
            returnHomeBlockedCells,
            previousNavOutput,
            bestDistanceToHomeThisSession,
            framesSinceDistanceImproved,
            mapAlreadyCompleteNoticeActive,
            mapSaveTimer,
            mapWasLoadedAtStartup,
        };
    }
};

} // namespace

// 1: MissingPersistedMapResetSucceeds
TEST(ExplorationSessionResetTest, MissingPersistedMapResetSucceeds)
{
    const std::string path = testFilePath("missing_file.json");
    std::filesystem::remove(path);
    ASSERT_FALSE(std::filesystem::exists(path));

    SessionHarness harness(testBounds());
    ExplorationSessionState session = harness.state();

    EXPECT_NO_FATAL_FAILURE(resetExplorationSession(path, /*mapPersistenceAvailable=*/true, session));
    EXPECT_EQ(harness.map.exploredCellCount(), 0U);
}

// 2: ExistingPersistedMapIsDeleted
TEST(ExplorationSessionResetTest, ExistingPersistedMapIsDeleted)
{
    const std::string path = testFilePath("existing_file.json");
    ExplorationMap savedMap(testBounds());
    savedMap.markOccupied(1, 1);
    ASSERT_TRUE(ExplorationMapStorage::save(path, savedMap));
    ASSERT_TRUE(std::filesystem::exists(path));

    SessionHarness harness(testBounds());
    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/true, session);

    EXPECT_FALSE(std::filesystem::exists(path));
}

// 3: InMemoryExplorationMapBecomesFresh
TEST(ExplorationSessionResetTest, InMemoryExplorationMapBecomesFresh)
{
    const std::string path = testFilePath("in_memory_fresh.json");
    SessionHarness harness(testBounds());
    harness.map.markFree(2, 2);
    harness.map.markOccupied(3, 3);
    ASSERT_GT(harness.map.exploredCellCount(), 0U);

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_EQ(harness.map.exploredCellCount(), 0U);
    EXPECT_EQ(harness.map.cellAt(2, 2), MapCell::Unknown);
    EXPECT_EQ(harness.map.cellAt(3, 3), MapCell::Unknown);
}

// 4: FreshMapUsesCurrentTableGeometry
TEST(ExplorationSessionResetTest, FreshMapUsesCurrentTableGeometry)
{
    const std::string path = testFilePath("geometry.json");
    SessionHarness harness(testBounds());
    const int expectedWidth = harness.map.width();
    const int expectedHeight = harness.map.height();
    const float expectedCellSize = harness.map.cellSize();

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    // Never hardcoded - derived from the SAME TableSurface + resolution the
    // map already had (this phase's own brief).
    EXPECT_EQ(harness.map.width(), expectedWidth);
    EXPECT_EQ(harness.map.height(), expectedHeight);
    EXPECT_FLOAT_EQ(harness.map.cellSize(), expectedCellSize);
    EXPECT_EQ(harness.map.totalCellCount(), static_cast<std::size_t>(expectedWidth) * static_cast<std::size_t>(expectedHeight));
}

// 5: CoverageTrailClearedOnNewMap
TEST(ExplorationSessionResetTest, CoverageTrailClearedOnNewMap)
{
    const std::string path = testFilePath("trail_cleared.json");
    SessionHarness harness(testBounds());
    harness.trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    harness.trail.update(RobotPose{Vec3{1.0F, 0.0F, 1.0F}, 0.0F});
    ASSERT_FALSE(harness.trail.points().empty());

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_TRUE(harness.trail.points().empty());
}

// 6: FrontierStateClearedOnNewMap
TEST(ExplorationSessionResetTest, FrontierStateClearedOnNewMap)
{
    const std::string path = testFilePath("frontier_cleared.json");
    SessionHarness harness(testBounds());
    FrontierTarget target;
    target.found = true;
    target.worldPosition = Vec3{0.5F, 0.0F, 0.5F};
    harness.currentFrontierTarget = target;
    harness.frontierBlacklist.push_back(GridCoord{4, 4});
    harness.framesSinceLastFrontierAttempt = 7;
    harness.consecutiveNoTargetFound = 3;

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_FALSE(harness.currentFrontierTarget.has_value());
    EXPECT_TRUE(harness.frontierBlacklist.empty());
    EXPECT_EQ(harness.framesSinceLastFrontierAttempt, 0);
    EXPECT_EQ(harness.consecutiveNoTargetFound, 0);
}

// 7: NavigationRouteClearedOnNewMap
TEST(ExplorationSessionResetTest, NavigationRouteClearedOnNewMap)
{
    const std::string path = testFilePath("route_cleared.json");
    SessionHarness harness(testBounds());
    markAllFree(harness.map);
    const Vec3 start = harness.map.cellToWorld(8, 8);
    const Vec3 goal = harness.map.cellToWorld(30, 30);
    const WaypointNavigatorOutput driving =
        harness.navigator.update(RobotPose{start, 0.0F}, harness.map, goal, true, false);
    ASSERT_EQ(driving.state, WaypointNavigatorState::Following);
    ASSERT_FALSE(harness.navigator.currentRoute().empty());

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_EQ(harness.navigator.state(), WaypointNavigatorState::Inactive);
    EXPECT_TRUE(harness.navigator.currentRoute().empty());
}

// 8: CompletionLatchClearedOnNewMap
TEST(ExplorationSessionResetTest, CompletionLatchClearedOnNewMap)
{
    const std::string path = testFilePath("completion_cleared.json");
    SessionHarness harness(testBounds());
    harness.completionSignal.complete = true;
    harness.mapAlreadyCompleteNoticeActive = true;

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_FALSE(harness.completionSignal.complete);
    EXPECT_FALSE(harness.mapAlreadyCompleteNoticeActive);
}

// 9: TemporaryBlockedCellsClearedOnNewMap
TEST(ExplorationSessionResetTest, TemporaryBlockedCellsClearedOnNewMap)
{
    const std::string path = testFilePath("blocked_cells_cleared.json");
    SessionHarness harness(testBounds());
    harness.returnHomeBlockedCells.push_back(GridCoord{6, 6});
    harness.returnHomeBlockedCells.push_back(GridCoord{7, 7});
    harness.previousNavOutput.state = WaypointNavigatorState::Following;
    harness.previousNavOutput.route.push_back(Vec3{1.0F, 0.0F, 1.0F});
    harness.bestDistanceToHomeThisSession = 0.5F;
    harness.framesSinceDistanceImproved = 200;

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_TRUE(harness.returnHomeBlockedCells.empty());
    EXPECT_EQ(harness.previousNavOutput.state, WaypointNavigatorState::Inactive);
    EXPECT_TRUE(harness.previousNavOutput.route.empty());
    EXPECT_TRUE(std::isinf(harness.bestDistanceToHomeThisSession));
    EXPECT_EQ(harness.framesSinceDistanceImproved, 0);
}

// 10: LoadedCompletedMapCanBeResetWithoutRestart
TEST(ExplorationSessionResetTest, LoadedCompletedMapCanBeResetWithoutRestart)
{
    const std::string path = testFilePath("loaded_completed.json");
    SessionHarness harness(testBounds());
    markAllFree(harness.map);
    ASSERT_TRUE(ExplorationMapStorage::save(path, harness.map));

    // Simulate a session that started from a loaded, already-complete map -
    // the exact "application started with an existing saved map" scenario
    // this phase's brief calls out.
    ExplorationMap reloaded(testBounds());
    ASSERT_EQ(ExplorationMapStorage::load(path, reloaded), MapLoadResult::Loaded);
    harness.map.setCells(reloaded.cells());
    harness.mapWasLoadedAtStartup = true;
    harness.completionSignal.complete = true;
    harness.mapAlreadyCompleteNoticeActive = true;
    ASSERT_GT(harness.map.exploredCellCount(), 0U);

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/true, session);

    EXPECT_FALSE(std::filesystem::exists(path));
    EXPECT_EQ(harness.map.exploredCellCount(), 0U);
    EXPECT_FALSE(harness.mapWasLoadedAtStartup);
    EXPECT_FALSE(harness.completionSignal.complete);
    EXPECT_FALSE(harness.mapAlreadyCompleteNoticeActive);
}

// 11: NewExplorationCanStartAfterReset
TEST(ExplorationSessionResetTest, NewExplorationCanStartAfterReset)
{
    const std::string path = testFilePath("new_exploration.json");
    SessionHarness harness(testBounds());
    markAllFree(harness.map); // simulates a previously fully-mapped session
    harness.completionSignal.complete = true;

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    // No stale completion/frontier state may make a fresh mission
    // immediately report Complete - mark a small explored footprint (as
    // the real ExplorationMapper would over the first few frames of a new
    // session) and confirm the map is genuinely exploring again (a real,
    // REACHABLE frontier target is found), not complete. A 7x7 block
    // (rather than a single cell) mirrors FrontierExplorerTests.cpp's own
    // convention - large enough that GridPathPlanner's clearance/planning
    // margin around the candidate target cell is satisfied, and that the
    // resulting frontier cluster clears kMinimumFrontierClusterSize.
    const int centerCol = harness.map.width() / 2;
    const int centerRow = harness.map.height() / 2;
    for (int dRow = -3; dRow <= 3; ++dRow)
    {
        for (int dCol = -3; dCol <= 3; ++dCol)
        {
            harness.map.markFree(centerCol + dCol, centerRow + dRow);
        }
    }

    const FrontierExplorer explorer(harness.map);
    const Vec3 robotPosition = harness.map.cellToWorld(centerCol, centerRow);
    EXPECT_EQ(explorer.completion(robotPosition), ExplorationCompletion::Exploring);
    EXPECT_FALSE(explorer.frontierCells().empty());
    EXPECT_TRUE(explorer.selectTarget(robotPosition).found);
}

// 12: ResetDoesNotChangeRobotPose
TEST(ExplorationSessionResetTest, ResetDoesNotChangeRobotPose)
{
    const std::string path = testFilePath("pose_unaffected.json");
    SessionHarness harness(testBounds());

    // resetExplorationSession()/ExplorationSessionState are structurally
    // incapable of touching robot pose - neither type ever takes a
    // RobotPose reference. This test still exercises a real external pose
    // value across the call to guard against a future accidental
    // signature change re-introducing one.
    const RobotPose poseBefore{Vec3{1.23F, 0.0F, -4.56F}, 77.0F};
    RobotPose pose = poseBefore;

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_FLOAT_EQ(pose.position.x, poseBefore.position.x);
    EXPECT_FLOAT_EQ(pose.position.z, poseBefore.position.z);
    EXPECT_FLOAT_EQ(pose.headingDegrees, poseBefore.headingDegrees);
}

// 13: ResetDoesNotChangeRobotState
TEST(ExplorationSessionResetTest, ResetDoesNotChangeRobotState)
{
    const std::string path = testFilePath("robot_state_unaffected.json");
    SessionHarness harness(testBounds());

    // resetExplorationSession()/ExplorationSessionState never take a
    // RobotStateMachine reference either - map reset is an application/
    // session operation, never an FSM transition (this phase's own
    // brief). A real RobotStateMachine instance is still exercised across
    // the call for the same future-signature-change guard as above.
    RobotStateMachine stateMachine;
    const RobotState stateBefore = stateMachine.currentState();

    ExplorationSessionState session = harness.state();
    resetExplorationSession(path, /*mapPersistenceAvailable=*/false, session);

    EXPECT_EQ(stateMachine.currentState(), stateBefore);
}
