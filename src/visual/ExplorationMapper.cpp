#include "robot/visual/ExplorationMapper.hpp"

#include <cmath>

#include "robot/visual/VisualMath.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

// Integer Bresenham line algorithm over grid (col, row) coordinates -
// deterministic, frame-rate-independent, no floating-point step
// accumulation. Marks every cell from (col0, row0) to (col1, row1)
// inclusive as Free via `map`, except the final cell, which is marked
// Occupied instead when `markEndpointOccupied` is true. Cells outside the
// grid are silently skipped (ExplorationMap::markFree()/markOccupied()
// already no-op for those) - this still walks the full line's cell count,
// bounded by RangeObservation::maxRange / ExplorationMap::cellSize()
// (at most a few dozen cells for this project's actual sensor range/grid
// resolution), so there is no unbounded-iteration risk even when an
// endpoint lands outside the grid.
void traceLine(ExplorationMap& map, int col0, int row0, int col1, int row1, bool markEndpointOccupied)
{
    const int dx = std::abs(col1 - col0);
    const int dy = std::abs(row1 - row0);
    const int stepCol = (col0 < col1) ? 1 : -1;
    const int stepRow = (row0 < row1) ? 1 : -1;
    int error = dx - dy;

    int col = col0;
    int row = row0;
    while (true)
    {
        const bool isEndpoint = (col == col1 && row == row1);
        if (isEndpoint && markEndpointOccupied)
        {
            map.markOccupied(col, row);
        }
        else
        {
            map.markFree(col, row);
        }

        if (isEndpoint)
        {
            break;
        }

        const int doubledError = 2 * error;
        if (doubledError > -dy)
        {
            error -= dy;
            col += stepCol;
        }
        if (doubledError < dx)
        {
            error += dx;
            row += stepRow;
        }
    }
}

} // namespace

ExplorationMapper::ExplorationMapper(ExplorationMap& map) noexcept
    : map_(map)
{
}

void ExplorationMapper::update(const RobotPose& pose, const std::vector<RangeObservation>& observations)
{
    for (const RangeObservation& observation : observations)
    {
        traceObservation(observation);
    }
    markRobotFootprint(pose);
}

void ExplorationMapper::traceObservation(const RangeObservation& observation)
{
    // RangeObservation.hpp's own convention: `distance` is already the
    // correct traced length regardless of `hit` (measured hit distance
    // when true, maxRange when false) - no branch needed here to pick
    // which field to read.
    const Vec3 endpoint{observation.origin.x + (observation.direction.x * observation.distance), observation.origin.y,
                         observation.origin.z + (observation.direction.z * observation.distance)};

    int originCol = 0;
    int originRow = 0;
    int endCol = 0;
    int endRow = 0;
    const bool originInside = map_.worldToCell(observation.origin, originCol, originRow);
    const bool endInside = map_.worldToCell(endpoint, endCol, endRow);

    if (!originInside && !endInside)
    {
        // Neither end of the ray touches the grid at all - nothing this
        // mapper can mark (this project's sensor range is small relative
        // to the table, so this is only reachable right at/beyond the
        // table edge).
        return;
    }

    // A ray whose origin or endpoint falls fractionally outside the grid
    // (e.g. the robot standing exactly at the table edge) still traces
    // deterministically: worldToCell() clamps neither, so an outside
    // origin/endpoint keeps its nearest valid cell coordinate via the
    // grid-relative floor() division ExplorationMap::worldToCell()
    // performs internally - traceLine() itself simply skips any cell
    // along the way that isInsideGrid() rejects (see traceLine()'s own
    // docs above). Falling back to the observation's own origin/endpoint
    // cell (0,0)-clamped-by-construction keeps this deterministic even
    // in that edge case rather than silently doing nothing.
    traceLine(map_, originCol, originRow, endCol, endRow, observation.hit);
}

void ExplorationMapper::markRobotFootprint(const RobotPose& pose)
{
    // Small rotated-rectangle footprint, not a circle (this phase's own
    // brief: "do not paint a huge circle around the robot") - the exact
    // physical body rectangle (RobotDimensions::kBodyWidth x kBodyLength,
    // VisualRobot.hpp), tested by transforming each CANDIDATE cell's
    // world-space center into the robot's own local frame (inverse
    // rotation via forwardDirection()/rightDirection(), the same
    // convention every other body-relative geometry component in this
    // codebase already uses) and checking it falls within the half-
    // width/half-length box - never an independently-invented rotation.
    const float halfWidth = RobotDimensions::kBodyWidth / 2.0F;
    const float halfLength = RobotDimensions::kBodyLength / 2.0F;
    // Generous bounding radius for the candidate-cell search below - the
    // rectangle's own half-diagonal, so no corner cell is ever missed.
    const float searchRadius = std::sqrt((halfWidth * halfWidth) + (halfLength * halfLength));

    const Vec3 forward = forwardDirection(pose);
    const Vec3 right = rightDirection(pose);

    int centerCol = 0;
    int centerRow = 0;
    if (!map_.worldToCell(pose.position, centerCol, centerRow))
    {
        // Robot center itself is off the grid entirely (e.g. mid-recovery
        // right at the table edge) - nothing to mark this call.
        return;
    }

    const int cellSearchSpan = static_cast<int>(std::ceil(searchRadius / map_.cellSize())) + 1;
    for (int rowOffset = -cellSearchSpan; rowOffset <= cellSearchSpan; ++rowOffset)
    {
        for (int colOffset = -cellSearchSpan; colOffset <= cellSearchSpan; ++colOffset)
        {
            const int col = centerCol + colOffset;
            const int row = centerRow + rowOffset;
            if (!map_.isInsideGrid(col, row))
            {
                continue;
            }

            const Vec3 cellWorld = map_.cellToWorld(col, row);
            const float dx = cellWorld.x - pose.position.x;
            const float dz = cellWorld.z - pose.position.z;

            // Project into the robot's local forward/right frame - the
            // same inverse-rotation technique VirtualCliffSensor.hpp's
            // corner-placement logic uses, just applied to a query point
            // instead of placing a sensor.
            const float localForward = (dx * forward.x) + (dz * forward.z);
            const float localRight = (dx * right.x) + (dz * right.z);

            if (std::fabs(localRight) <= halfWidth && std::fabs(localForward) <= halfLength)
            {
                map_.markFree(col, row);
            }
        }
    }
}

} // namespace robot::visual
