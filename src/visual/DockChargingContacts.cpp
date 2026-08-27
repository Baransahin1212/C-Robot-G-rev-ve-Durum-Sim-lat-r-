#include "robot/visual/DockChargingContacts.hpp"

#include <algorithm>
#include <cmath>

#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

namespace
{
enum class Axis
{
    X,
    Z
};

struct NearestEdge
{
    Axis axis;
    float distance;
    // +1 if the NEAREST edge is the max-bound edge (inward therefore
    // points toward the min-bound side), -1 if the nearest edge is the
    // min-bound edge (inward points toward the max-bound side).
    float inwardSign;
};

NearestEdge nearestTableEdge(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const float distToMinX = base.position.x - tableSurface.minX;
    const float distToMaxX = tableSurface.maxX - base.position.x;
    const float distToMinZ = base.position.z - tableSurface.minZ;
    const float distToMaxZ = tableSurface.maxZ - base.position.z;

    NearestEdge best{Axis::Z, distToMinZ, 1.0F};
    if (distToMaxZ < best.distance)
    {
        best = NearestEdge{Axis::Z, distToMaxZ, -1.0F};
    }
    if (distToMinX < best.distance)
    {
        best = NearestEdge{Axis::X, distToMinX, 1.0F};
    }
    if (distToMaxX < best.distance)
    {
        best = NearestEdge{Axis::X, distToMaxX, -1.0F};
    }
    return best;
}

float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}
} // namespace

Vec3 dockInwardDirection(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const NearestEdge edge = nearestTableEdge(base, tableSurface);
    if (edge.axis == Axis::X)
    {
        return Vec3{edge.inwardSign, 0.0F, 0.0F};
    }
    return Vec3{0.0F, 0.0F, edge.inwardSign};
}

DockChargingContactPair computeDockChargingContacts(const BasePlatform& base,
                                                      const TableSurface& tableSurface) noexcept
{
    const Vec3 inward = dockInwardDirection(base, tableSurface);
    // Entry heading - the direction FROM the desk interior TOWARD the
    // dock - is simply the negation of inward; its own rightDirection()
    // is the dock's fixed lateral axis (see this function's own header
    // docs).
    const float entryHeadingDegrees = normalizeHeadingDegrees(headingDegreesFromDirection(-inward.x, -inward.z));
    const Vec3 right = rightDirection(RobotPose{Vec3{}, entryHeadingDegrees});
    const float halfSpacing = kContactPairSpacingWorldUnits / 2.0F;

    const DockChargingContact left{
        Vec3{base.position.x - (right.x * halfSpacing), kContactHeightWorldUnits,
             base.position.z - (right.z * halfSpacing)},
        kContactRadiusWorldUnits};
    const DockChargingContact rightContact{
        Vec3{base.position.x + (right.x * halfSpacing), kContactHeightWorldUnits,
             base.position.z + (right.z * halfSpacing)},
        kContactRadiusWorldUnits};
    return DockChargingContactPair{left, rightContact};
}

DockChargingContactPair computeRobotRearChargingContacts(const RobotPose& pose) noexcept
{
    const Vec3 forward = forwardDirection(pose);
    const Vec3 right = rightDirection(pose);
    const float halfLength = RobotDimensions::kBodyLength / 2.0F;
    const Vec3 rearCenter{pose.position.x - (forward.x * halfLength), kContactHeightWorldUnits,
                           pose.position.z - (forward.z * halfLength)};
    const float halfSpacing = kContactPairSpacingWorldUnits / 2.0F;

    const DockChargingContact left{
        Vec3{rearCenter.x - (right.x * halfSpacing), kContactHeightWorldUnits,
             rearCenter.z - (right.z * halfSpacing)},
        kContactRadiusWorldUnits};
    const DockChargingContact rightContact{
        Vec3{rearCenter.x + (right.x * halfSpacing), kContactHeightWorldUnits,
             rearCenter.z + (right.z * halfSpacing)},
        kContactRadiusWorldUnits};
    return DockChargingContactPair{left, rightContact};
}

bool chargingContactsAligned(const DockChargingContactPair& dock, const DockChargingContactPair& robot,
                              float toleranceWorldUnits) noexcept
{
    const bool sameOrder = distanceWorld(dock.left.position, robot.left.position) <= toleranceWorldUnits &&
                            distanceWorld(dock.right.position, robot.right.position) <= toleranceWorldUnits;
    const bool crossOrder = distanceWorld(dock.left.position, robot.right.position) <= toleranceWorldUnits &&
                             distanceWorld(dock.right.position, robot.left.position) <= toleranceWorldUnits;
    return sameOrder || crossOrder;
}

DockChargingContactErrors computeChargingContactErrors(const DockChargingContactPair& dock,
                                                         const DockChargingContactPair& robot) noexcept
{
    const float sameOrderLeft = distanceWorld(dock.left.position, robot.left.position);
    const float sameOrderRight = distanceWorld(dock.right.position, robot.right.position);
    const float crossOrderLeft = distanceWorld(dock.left.position, robot.right.position);
    const float crossOrderRight = distanceWorld(dock.right.position, robot.left.position);

    const float sameOrderMax = std::max(sameOrderLeft, sameOrderRight);
    const float crossOrderMax = std::max(crossOrderLeft, crossOrderRight);

    if (sameOrderMax <= crossOrderMax)
    {
        return DockChargingContactErrors{sameOrderLeft, sameOrderRight, sameOrderMax};
    }
    return DockChargingContactErrors{crossOrderRight, crossOrderLeft, crossOrderMax};
}

} // namespace robot::visual
