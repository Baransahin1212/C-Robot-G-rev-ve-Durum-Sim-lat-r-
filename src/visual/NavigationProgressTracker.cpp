#include "robot/visual/NavigationProgressTracker.hpp"

#include <cmath>

#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

void NavigationProgressTracker::update(const RobotPose& pose) noexcept
{
    if (!anchored_)
    {
        anchored_ = true;
        anchorPosition_ = pose.position;
        latestPosition_ = pose.position;
        previousHeadingDegrees_ = pose.headingDegrees;
        accumulatedAbsoluteRotationDegrees_ = 0.0F;
        samplesSinceAnchor_ = 0;
        return;
    }

    const float headingDelta =
        std::fabs(shortestSignedHeadingErrorDegrees(previousHeadingDegrees_, pose.headingDegrees));
    accumulatedAbsoluteRotationDegrees_ += headingDelta;
    previousHeadingDegrees_ = pose.headingDegrees;
    latestPosition_ = pose.position;
    ++samplesSinceAnchor_;
}

void NavigationProgressTracker::reset() noexcept
{
    anchored_ = false;
    accumulatedAbsoluteRotationDegrees_ = 0.0F;
    samplesSinceAnchor_ = 0;
}

bool NavigationProgressTracker::isStuck() const noexcept
{
    if (!anchored_ || samplesSinceAnchor_ < kStuckSampleWindow)
    {
        return false;
    }
    if (accumulatedAbsoluteRotationDegrees_ < kStuckRotationThresholdDegrees)
    {
        return false;
    }
    const float dx = latestPosition_.x - anchorPosition_.x;
    const float dz = latestPosition_.z - anchorPosition_.z;
    const float displacement = std::sqrt((dx * dx) + (dz * dz));
    return displacement < kStuckDisplacementThreshold;
}

} // namespace robot::visual
