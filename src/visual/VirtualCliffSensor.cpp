#include "robot/visual/VirtualCliffSensor.hpp"

#include "robot/visual/VisualMath.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

// +1.0F selects the front/right side of the corresponding axis, -1.0F
// the rear/left side - see cliffSensorWorldPosition() below.
struct CornerSigns
{
    float forwardSign;
    float rightSign;
};

CornerSigns cornerSignsFor(CliffSensorPosition sensorPosition) noexcept
{
    switch (sensorPosition)
    {
        case CliffSensorPosition::FrontLeft: return CornerSigns{1.0F, -1.0F};
        case CliffSensorPosition::FrontRight: return CornerSigns{1.0F, 1.0F};
        case CliffSensorPosition::RearLeft: return CornerSigns{-1.0F, -1.0F};
        case CliffSensorPosition::RearRight: return CornerSigns{-1.0F, 1.0F};
    }
    return CornerSigns{0.0F, 0.0F};
}

} // namespace

Vec3 cliffSensorWorldPosition(const RobotPose& pose, CliffSensorPosition sensorPosition) noexcept
{
    const Vec3 forward = forwardDirection(pose);
    const Vec3 right = rightDirection(pose);
    const float halfLength = RobotDimensions::kBodyLength / 2.0F;
    const float halfWidth = RobotDimensions::kBodyWidth / 2.0F;
    const CornerSigns signs = cornerSignsFor(sensorPosition);

    return Vec3{pose.position.x + (forward.x * signs.forwardSign * halfLength) +
                    (right.x * signs.rightSign * halfWidth),
                pose.position.y,
                pose.position.z + (forward.z * signs.forwardSign * halfLength) +
                    (right.z * signs.rightSign * halfWidth)};
}

bool isPointOnTable(const Vec3& point, const TableSurface& table) noexcept
{
    return point.x >= table.minX && point.x <= table.maxX && point.z >= table.minZ && point.z <= table.maxZ;
}

CliffSensorReadings computeCliffSensorReadings(const RobotPose& pose, const TableSurface& table) noexcept
{
    CliffSensorReadings readings;
    readings.frontLeft = !isPointOnTable(cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft), table);
    readings.frontRight = !isPointOnTable(cliffSensorWorldPosition(pose, CliffSensorPosition::FrontRight), table);
    readings.rearLeft = !isPointOnTable(cliffSensorWorldPosition(pose, CliffSensorPosition::RearLeft), table);
    readings.rearRight = !isPointOnTable(cliffSensorWorldPosition(pose, CliffSensorPosition::RearRight), table);
    return readings;
}

bool isPointSafelyInsideTable(const Vec3& point, const TableSurface& table, float margin) noexcept
{
    return point.x >= (table.minX + margin) && point.x <= (table.maxX - margin) &&
           point.z >= (table.minZ + margin) && point.z <= (table.maxZ - margin);
}

bool areAllCornersSafelyInsideTable(const RobotPose& pose, const TableSurface& table, float margin) noexcept
{
    return isPointSafelyInsideTable(cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft), table, margin) &&
           isPointSafelyInsideTable(cliffSensorWorldPosition(pose, CliffSensorPosition::FrontRight), table, margin) &&
           isPointSafelyInsideTable(cliffSensorWorldPosition(pose, CliffSensorPosition::RearLeft), table, margin) &&
           isPointSafelyInsideTable(cliffSensorWorldPosition(pose, CliffSensorPosition::RearRight), table, margin);
}

VirtualCliffSensor::VirtualCliffSensor(const VirtualWorld& world)
    : world_(world)
{
}

CliffSensorReadings VirtualCliffSensor::readings() const
{
    return computeCliffSensorReadings(world_.robotPose(), world_.tableSurface());
}

Vec3 VirtualCliffSensor::sensorWorldPosition(CliffSensorPosition sensorPosition) const
{
    return cliffSensorWorldPosition(world_.robotPose(), sensorPosition);
}

} // namespace robot::visual
