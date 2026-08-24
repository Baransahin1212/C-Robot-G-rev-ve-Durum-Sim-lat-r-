#pragma once

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13V: one sensor ray's result, in the exact shape ExplorationMapper
// consumes - raylib-free, no FSM/Event/IRobotHardware knowledge, matching
// every other plain-data struct in this codebase (RobotPose, BoxObstacle,
// ...). Deliberately its own header (not folded into
// VirtualObstacleSensorArray.hpp) so ExplorationMapper.hpp can depend on
// this single small struct without pulling in VirtualObstacleSensorArray's
// own class/geometry-implementation surface - the mapper only ever needs
// the RESULT of a ray cast, never the ray-casting logic itself (see
// ExplorationMapper.hpp's own docs for why it must never duplicate
// VirtualObstacleSensorArray's/VirtualDistanceSensor's ray/AABB math).
//
// `distance` convention: when `hit` is true, this is the measured distance
// to the obstacle (<= maxRange); when `hit` is false, this is `maxRange`
// itself - so a caller can always treat [origin, origin + direction *
// distance] as "the segment this ray observed as clear," regardless of
// whether it ended in a hit, with no separate branch needed to compute the
// traced length.
struct RangeObservation
{
    Vec3 origin;
    Vec3 direction;
    float distance = 0.0F;
    float maxRange = 0.0F;
    bool hit = false;
};

} // namespace robot::visual
