#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

namespace
{

// Phase 13W final workspace redesign: physical design scale, derived from
// the robot's own RobotDimensions::kBodyWidth (0.40F) - NOT a unit-system
// conversion exposed anywhere in code, purely the ratio used to choose
// every other number below so proportions read as believable ("0.40 world
// units looks like an ~8cm-wide miniature robot on a desk," conceptually
// 1.0 world unit =~ 20cm) - see docs/technical-decisions.md (Phase 13W
// final redesign) for the full worked ratios. Every constant below was
// chosen against this scale, not against arbitrary round numbers.

// Desk (Phase 13W v2): a normal computer-desk aspect ratio (~2:1),
// conceptually ~160cm x 80cm, replacing the old square 12x12 tabletop that
// human visual validation rejected as "visually enormous" with dominant
// empty space. X = width (the desk's long axis), Z = depth (short axis).
constexpr float kTableHalfWidth = 4.0F; // X: [-4, 4] -> 8.0 wide
constexpr float kTableHalfDepth = 2.0F; // Z: [-2, 2] -> 4.0 deep

// Charging dock (Phase 13W final redesign): EXPLICIT product requirement -
// immediately beside the monitor, near the desk's -Z ("rear") edge, the
// same row the monitor itself sits in - replacing the earlier v2 pass's
// front-edge placement, which human validation rejected ("the dock must
// be immediately beside the monitor, not off near the front edge"). Small
// - only slightly larger than the now-miniature robot. Position is the
// ONE semantic home point every navigation/mapping component targets
// (HomeNavigator, HomeArrivalEventSource, the exploration map's home
// marker) - never duplicated elsewhere (see docs/technical-decisions.md).
constexpr float kBaseX = 1.3F;
constexpr float kBaseZ = -1.5F;
constexpr float kBaseWidth = 0.65F;
constexpr float kBaseHeight = 0.05F;
constexpr float kBaseDepth = 0.45F;

// Robot starts parked at the dock - not already inside
// HomeNavigator::kHomeArrivalRadius (0.40F) of it, so a Return Home
// request right after startup is never trivially already-home.
constexpr float kRobotStartX = kBaseX;
// Half of RobotDimensions::kBodyHeight (0.16F) - VirtualWorld.cpp cannot
// include VisualRobot.hpp (VisualRobot.hpp already includes
// VirtualWorld.hpp; a reverse include would cycle), so this stays a
// hand-kept literal, same as before the Phase 13W rescale.
constexpr float kRobotStartY = 0.08F;
// 1.0F out from the dock's own arrival point, toward the open desk
// interior (+Z, AWAY from the dock now that it sits at the rear -Z edge -
// the opposite direction from the earlier front-edge dock's own "1.5F
// toward -Z" offset). Scaled down from that 1.5F alongside the robot's
// own ~2/3 rescale (1.5F * 0.4F/0.6F =~ 1.0F) - close enough to read as
// "just off the dock," far enough that HomeNavigator's own Aligning turn
// (see kRobotStartHeadingDegrees below) never sweeps near it at close
// range.
constexpr float kRobotStartZ = kBaseZ + 1.0F;
// Heading 90 (+X, per VisualMath.hpp's forwardDirection() convention: 0 =
// +Z, 90 = +X) faces across the open desk interior, away from the dock
// entirely - a sensible "roam" starting orientation (Start Gezinme should
// drive the robot OUT into the desk, never straight back at its own
// dock), and only a quarter-turn from the dock's own bearing from this
// position (heading 180, since dx=0/dz<0 from here to the dock). A full
// 180-degree antipodal start was tried in an earlier pass and regressed
// Return Home: HomeNavigator's Aligning phase always turns via a FIXED
// direction at exactly +-180 degrees of heading error
// (shortestSignedHeadingErrorDegrees never picks "whichever way is
// closer" at that exact antipodal case), forcing a full sweep through
// every heading in between - including ones that happened to point
// straight at a nearby desk object, producing a false
// obstacleDetected() mid-turn that handed off to ReactiveObstacleAvoidance
// and derailed the whole maneuver (see docs/technical-decisions.md, Phase
// 13W v2, "Return Home regression"). Starting at 90 avoids that class of
// bug generically: the Aligning sweep is always the SHORT arc between two
// non-antipodal headings.
constexpr float kRobotStartHeadingDegrees = 90.0F;

// Charging dock's rear housing (Phase 13W final redesign) - the one
// physically collidable dock piece (guide arms/contact pads/platform
// floor are visual-only - see Renderer3D::drawChargingDock()). Faces the
// REAR table edge: placed on the dock's -Z side (toward the nearby -2.0
// table boundary), OPPOSITE the platform's open +Z entrance (which now
// faces the desk interior, where the robot actually parks from/departs
// to - see kRobotStartZ above). Rescaled down alongside the robot's own
// ~2/3 factor (0.4F * 2/3 =~ 0.28F etc.), and empirically verified (see
// VirtualRobotHardwareTests.cpp's ReturnHomeReachesChargingDockWithoutOscillating)
// to leave the platform's own home point comfortably clear and to sit
// well inside the table's -Z boundary with margin.
constexpr float kDockHousingX = kBaseX;
constexpr float kDockHousingZ = kBaseZ - 0.35F;
constexpr float kDockHousingWidth = 0.28F;
constexpr float kDockHousingHeight = 0.22F;
constexpr float kDockHousingDepth = 0.10F;

} // namespace

VirtualWorld::VirtualWorld()
    : robotPose_{Vec3{kRobotStartX, kRobotStartY, kRobotStartZ}, kRobotStartHeadingDegrees}
    , basePlatform_{Vec3{kBaseX, kBaseHeight / 2.0F, kBaseZ}, Vec3{kBaseWidth, kBaseHeight, kBaseDepth}}
    , tableSurface_{-kTableHalfWidth, kTableHalfWidth, -kTableHalfDepth, kTableHalfDepth}
{
    // Phase 13W final workspace redesign: the production desk holds ONLY
    // the three objects an actual computer desk needs - monitor, keyboard,
    // mouse (explicit product requirement) - the mug/notebook/lamp-base
    // desk-clutter items from the earlier v2 pass are gone entirely, both
    // visually and physically (DeskObjectType/Renderer3D's drawX() helpers
    // for them remain defined - see DeskObjectType's own docs - purely to
    // avoid unnecessary churn, but nothing in this constructor ever
    // instantiates one). Monitor near rear-center; the charging dock (set
    // up via basePlatform_ above) sits immediately to its right, same row;
    // keyboard in front of the monitor; mouse to the keyboard's right,
    // clear of the dock's own straight-ahead exit corridor; the entire
    // front half of the desk is deliberately left open (see
    // docs/technical-decisions.md, Phase 13W final redesign, "workspace
    // layout" for the full coordinate table and clearance audit).
    addDeskObject(DeskObjectType::Monitor, Vec3{-1.2F, 0.25F, -1.2F}, Vec3{1.2F, 0.5F, 0.5F});
    addDeskObject(DeskObjectType::Keyboard, Vec3{-1.2F, 0.05F, -0.4F}, Vec3{2.3F, 0.1F, 0.75F});
    addDeskObject(DeskObjectType::Mouse, Vec3{0.6F, 0.05F, -0.4F}, Vec3{0.4F, 0.1F, 0.6F});

    // Charging dock's rear housing - see kDockHousing*'s own docs above. A
    // plain BoxObstacle, not a DeskObject (Renderer3D's drawChargingDock()
    // draws it directly from basePlatform(), not from deskObjects()).
    obstacles_.push_back(BoxObstacle{Vec3{kDockHousingX, kDockHousingHeight / 2.0F, kDockHousingZ},
                                      Vec3{kDockHousingWidth, kDockHousingHeight, kDockHousingDepth}});
}

void VirtualWorld::addDeskObject(DeskObjectType type, const Vec3& position, const Vec3& size)
{
    obstacles_.push_back(BoxObstacle{position, size, true});
    deskObjects_.push_back(DeskObject{type, position, size, true});
}

const RobotPose& VirtualWorld::robotPose() const noexcept
{
    return robotPose_;
}

const BasePlatform& VirtualWorld::basePlatform() const noexcept
{
    return basePlatform_;
}

const std::vector<BoxObstacle>& VirtualWorld::obstacles() const noexcept
{
    return obstacles_;
}

const std::vector<DeskObject>& VirtualWorld::deskObjects() const noexcept
{
    return deskObjects_;
}

std::size_t VirtualWorld::deskObjectObstacleIndex(DeskObjectType type) const noexcept
{
    // deskObjects_ and the leading run of obstacles_ are pushed together,
    // one-for-one, in addDeskObject() - so deskObjects_[i]'s obstacle is
    // always obstacles_[i] (the dock housing is pushed only after every
    // desk object, so it can never shift this correspondence).
    for (std::size_t i = 0; i < deskObjects_.size(); ++i)
    {
        if (deskObjects_[i].type == type)
        {
            return i;
        }
    }
    return obstacles_.size();
}

const TableSurface& VirtualWorld::tableSurface() const noexcept
{
    return tableSurface_;
}

void VirtualWorld::setRobotPosition(const Vec3& position)
{
    robotPose_.position = position;
}

void VirtualWorld::setRobotHeading(float headingDegrees)
{
    robotPose_.headingDegrees = headingDegrees;
}

bool VirtualWorld::setObstaclePosition(std::size_t index, const Vec3& position)
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    obstacles_[index].position = position;
    return true;
}

bool VirtualWorld::setObstacleSize(std::size_t index, const Vec3& size)
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    obstacles_[index].size = size;
    return true;
}

bool VirtualWorld::setObstacleEnabled(std::size_t index, bool enabled)
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    obstacles_[index].enabled = enabled;
    return true;
}

bool VirtualWorld::obstacleEnabled(std::size_t index) const
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    return obstacles_[index].enabled;
}

} // namespace robot::visual
