#pragma once

#include <vector>

#include "robot/visual/DockApproachController.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13Y dock-capture handoff fix (real-GUI-traced) - the orchestration-
// level counterpart to DockApproachController::kDockStagingCaptureRadius's
// own docs. DockApproachController itself deliberately never reads
// VirtualWorld::obstacles() (see that class's own header docs on its
// perception boundary), so the physical-safety/hazard-attribution reasoning
// that needs the real obstacle list lives here instead, in
// robot_visual_world (raylib-free, shared by main3d.cpp and every test
// harness that mirrors it) - never duplicated ad hoc in either caller.

// True if `pose` is within DockApproachController::kDockStagingCaptureRadius
// of the exact staging point (computeDockStagingPoint()) - purely a
// distance check, independent of WaypointNavigatorState entirely. See
// isDockCaptureEligible()'s own docs for the full eligibility contract this
// is only one part of.
bool isInsideDockStagingCaptureRegion(const RobotPose& pose, const BasePlatform& base,
                                       const TableSurface& tableSurface) noexcept;

// True if the ENABLED obstacle currently nearest `pose` is the SAME
// obstacle nearest `base` - i.e. whatever the robot is closest to right now
// is, by physical proximity, the same object that lives right next to the
// dock (this project's fixed layout: the charging dock's own rear housing -
// see VirtualWorld.cpp's own kDockHousing* docs - the monitor/keyboard/
// mouse all sit far enough from the dock that they are never the "nearest
// to base" obstacle). Never a hardcoded dock-housing coordinate - this is a
// purely RELATIVE, derived proximity comparison, so it keeps working even
// if the production desk layout is ever rearranged. False (never
// attributed) if there are no enabled obstacles at all - a hazard with no
// known cause is never assumed to be the dock's own.
bool isNearestHazardAttributableToDockGeometry(const RobotPose& pose, const BasePlatform& base,
                                                const std::vector<BoxObstacle>& obstacles) noexcept;

// Phase 13Y dock-capture handoff fix - the FULL eligibility contract for
// letting DockApproachController take ownership from Stage 1 WITHOUT
// waiting on WaypointNavigatorState::Arrived specifically (see
// DockApproachController::kDockStagingCaptureRadius's own docs for why that
// alone is not robust). ALL of the following must hold:
//   - `physicallySupported` (caller-computed, e.g. !CliffSensorReadings::
//     anyCliff() - kept as a parameter rather than this function owning its
//     own VirtualCliffSensor, mirroring how DockApproachController's own
//     `arrivedAtStagingPoint` parameter is caller-computed) - Safety's own
//     unconditional top-priority already protects against an off-table
//     robot regardless, but this keeps capture itself from ever firing on
//     the exact same frame Safety would otherwise need to intervene;
//   - `pose` is inside the dock staging capture region (see
//     isInsideDockStagingCaptureRegion() above);
//   - `pose`'s own position does not currently collide with any obstacle
//     (robotPositionCollidesWithObstacles() - the same check
//     VirtualRobotHardware's own pose-commit path already trusts, never a
//     new collision model);
//   - the exact staging point itself does not collide with any obstacle
//     either (proves the short final target is physically reachable, not
//     merely the robot's current spot);
//   - isNearestHazardAttributableToDockGeometry() above - i.e. nothing
//     OTHER than the dock's own known geometry is intruding nearby right
//     now (an unrelated desk object sitting closer than the dock itself
//     withholds capture, so DockApproachController never blindly drives
//     toward/through something it has no contact-geometry knowledge of).
bool isDockCaptureEligible(const RobotPose& pose, const BasePlatform& base, const TableSurface& tableSurface,
                            const std::vector<BoxObstacle>& obstacles, bool physicallySupported) noexcept;

} // namespace robot::visual
