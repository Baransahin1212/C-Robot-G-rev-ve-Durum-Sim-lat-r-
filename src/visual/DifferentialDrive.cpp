#include "robot/visual/DifferentialDrive.hpp"

#include <cmath>

namespace robot::visual
{

namespace
{

constexpr float kPi = 3.14159265358979323846F;

// Below this angular rate (radians/second), turning is treated as exactly
// straight - avoids dividing by a near-zero omega in the exact arc
// integration below, and matches equal (or near-equal) wheel speeds
// exactly, not just in the limit.
constexpr float kOmegaEpsilon = 1.0e-5F;

float degreesToRadians(float degrees) noexcept
{
    return degrees * (kPi / 180.0F);
}

float radiansToDegrees(float radians) noexcept
{
    return radians * (180.0F / kPi);
}

// Normalizes a heading into [0, 360) - fmod alone can return a negative
// result for a negative input, so a single conditional correction follows
// it, exactly like the project's existing wraparound-handling style
// elsewhere (see VirtualDistanceSensor.cpp's own small helpers).
float wrapHeadingDegrees(float degrees) noexcept
{
    float wrapped = std::fmod(degrees, 360.0F);
    if (wrapped < 0.0F)
    {
        wrapped += 360.0F;
    }
    return wrapped;
}

} // namespace

DifferentialDrive::DifferentialDrive(float wheelTrack) noexcept
    : wheelTrack_(wheelTrack)
{
}

void DifferentialDrive::setWheelSpeeds(float left, float right) noexcept
{
    speeds_.left = left;
    speeds_.right = right;
}

WheelSpeeds DifferentialDrive::wheelSpeeds() const noexcept
{
    return speeds_;
}

void DifferentialDrive::stop() noexcept
{
    speeds_ = WheelSpeeds{};
}

float DifferentialDrive::wheelTrack() const noexcept
{
    return wheelTrack_;
}

void DifferentialDrive::update(RobotPose& pose, float deltaSeconds) const noexcept
{
    // v = (vR + vL) / 2 ; omega = (vR - vL) / L - see docs/technical-
    // decisions.md (Phase 13P) for the full derivation and heading-
    // convention discussion.
    const float v = (speeds_.right + speeds_.left) / 2.0F;
    const float omega = (speeds_.right - speeds_.left) / wheelTrack_;

    const float theta = degreesToRadians(pose.headingDegrees);

    float dx = 0.0F;
    float dz = 0.0F;

    if (std::fabs(omega) < kOmegaEpsilon)
    {
        // Straight motion: dx = sin(theta)*distance, dz = cos(theta)*distance
        // - the project's one heading-to-direction convention (see
        // VisualMath.hpp's forwardDirection()), applied here directly rather
        // than through that helper since DifferentialDrive has no
        // RobotPose-reading dependency of its own beyond the pose it is
        // handed.
        const float distance = v * deltaSeconds;
        dx = std::sin(theta) * distance;
        dz = std::cos(theta) * distance;
    }
    else
    {
        // Exact constant-wheel-speed arc integration (not a discrete Euler
        // step): the closed-form solution of dx/dt = v*sin(theta(t)),
        // dz/dt = v*cos(theta(t)), theta(t) = theta0 + omega*t, integrated
        // over [0, deltaSeconds]. When v is 0 (in-place rotation), dx and dz
        // both come out exactly 0 regardless of omega - position stays
        // exactly put while heading changes.
        const float newTheta = theta + (omega * deltaSeconds);
        const float radius = v / omega;
        dx = radius * (std::cos(theta) - std::cos(newTheta));
        dz = radius * (std::sin(newTheta) - std::sin(theta));
    }

    pose.position.x += dx;
    pose.position.z += dz;

    const float newHeadingDegrees = pose.headingDegrees + radiansToDegrees(omega * deltaSeconds);
    pose.headingDegrees = wrapHeadingDegrees(newHeadingDegrees);
}

} // namespace robot::visual
