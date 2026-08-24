#pragma once

#include <cstddef>
#include <vector>

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13V: one grid cell's exploration state - genuinely three-valued,
// never collapsed to a bool, so "we have not looked here yet" (Unknown)
// stays visually and logically distinct from "we looked, and it's clear"
// (Free) - the entire point of progressive exploration mapping (see
// ExplorationMap's own class docs below).
enum class MapCell
{
    Unknown,
    Free,
    Occupied
};

// Raylib-free 2D occupancy-style grid over VirtualWorld's TableSurface,
// in world X/Z (Y is not modeled - this is a top-down map). Genuinely
// starts every cell Unknown; nothing pre-populates Free/Occupied cells
// from VirtualWorld::obstacles() or any other ground-truth source - see
// ExplorationMapper.hpp for the one legitimate way cells ever change
// (sensor observations), which is a strictly separate class from this
// one. ExplorationMap itself has no sensing/tracing logic of its own -
// it is pure grid storage plus the deterministic world<->cell
// conversions every caller (ExplorationMapper, Renderer3D,
// ExplorationMapStorage) shares, so none of them can independently
// (mis)compute a different mapping between world coordinates and grid
// indices.
//
// Resolution (Phase 13V brief: target 80x80-120x120 cells for the demo
// table, 0.10-0.15 world units/cell): kCellSizeWorldUnits = 0.12F chosen
// deliberately against VirtualWorld's actual TableSurface (12x12 world
// units, see VirtualWorld.cpp's kTableHalfExtent = 6.0F) - 12 / 0.12 =
// 100 exactly, landing at the center of both target ranges with no
// rounding remainder, so grid cells tile the table exactly with none left
// over at the far edge. Grid dimensions are still computed from `bounds`
// at construction time (never hardcoded to 100x100), so this class keeps
// working correctly if a future table size changes, just at whatever
// cell count that produces.
//
// Coordinate convention: matches this project's one X/Z convention
// throughout (VisualMath.hpp's forwardDirection()/rightDirection()) -
// column increases with world X, row increases with world Z, with no
// mirroring or rotation. column 0/row 0 is the cell touching
// (bounds.minX, bounds.minZ| the "near/left" corner as heading-0 (+Z)
// sees it; column/row both increase moving toward (bounds.maxX,
// bounds.maxZ).
class ExplorationMap
{
public:
    // See this class's own docs above for the derivation against the
    // actual demo TableSurface (12x12 world units -> exactly 100x100
    // cells).
    static constexpr float kCellSizeWorldUnits = 0.12F;

    // Constructs a grid covering `bounds` at kCellSizeWorldUnits
    // resolution, every cell Unknown. width()/height() are derived from
    // `bounds`, rounded to the nearest whole cell (never silently
    // truncated/floored, so a bounds size that is not an exact multiple
    // of kCellSizeWorldUnits still covers the full table rather than
    // clipping a partial cell off the far edge).
    explicit ExplorationMap(const TableSurface& bounds);

    int width() const noexcept;
    int height() const noexcept;
    float cellSize() const noexcept;
    const TableSurface& bounds() const noexcept;

    // False for a genuinely out-of-range (col, row) - callers must check
    // this (or use worldToCell()'s own bool return) before cellAt()/
    // markFree()/markOccupied(); those three are UB-free but simply
    // no-op/return Unknown on an out-of-range index rather than assert,
    // matching VirtualWorld::setObstaclePosition()'s own
    // out-of-range-returns-false convention.
    bool isInsideGrid(int col, int row) const noexcept;

    // MapCell::Unknown for a genuinely out-of-range (col, row).
    MapCell cellAt(int col, int row) const noexcept;

    // Converts a world X/Z position into the (col, row) it falls inside;
    // returns false (outCol/outRow left unchanged) when the position is
    // outside `bounds` entirely. Only X/Z are read - Y is ignored, this
    // is a top-down map.
    bool worldToCell(const Vec3& worldPosition, int& outCol, int& outRow) const noexcept;

    // Inverse of worldToCell(): the world-space CENTER of cell (col,
    // row), at world Y = 0.0F (this map carries no height information of
    // its own). Deterministic round-trip with worldToCell() for any
    // world position genuinely inside `bounds` (see
    // ExplorationMapTests.cpp's CellToWorldRoundTrip).
    Vec3 cellToWorld(int col, int row) const noexcept;

    // Upgrades cell (col, row) to Free - but ONLY if it is currently
    // Unknown; a cell already Occupied is never downgraded by this call
    // (see this class's own "occupancy precedence" docs below). No-op
    // for an out-of-range (col, row).
    void markFree(int col, int row) noexcept;

    // Upgrades cell (col, row) to Occupied unconditionally (from Unknown
    // OR Free) - see "occupancy precedence" below. No-op for an
    // out-of-range (col, row).
    void markOccupied(int col, int row) noexcept;

    // Occupancy precedence (Phase 13V brief, "conservative Occupied
    // precedence acceptable for V1"): Occupied > Free > Unknown, and
    // once a cell is Occupied nothing in this class's public API can
    // ever move it back to Free or Unknown - not a later markFree() call
    // sweeping through it (a ray "grazing" past a corner it already
    // marked Occupied does not erase that observation), and not any
    // bulk/load operation short of replacing the whole grid via
    // setCells(). Rationale: a false negative (an Occupied cell that
    // later evidence could have cleared) is safe - a robot avoids a cell
    // it does not strictly need to enter; a false positive introduced by
    // downgrading a real obstacle back to Free on noisy/grazing evidence
    // is the failure mode worth avoiding for a V1 exploration display.
    // Deliberately no explicit "clear" API exists yet for this reason.

    // Total number of Free + Occupied cells (Unknown does not count as
    // explored) - see exploredPercentage() below.
    std::size_t exploredCellCount() const noexcept;

    // width() * height() - every cell in the grid, regardless of state.
    std::size_t totalCellCount() const noexcept;

    // 100.0F * exploredCellCount() / totalCellCount(), or 0.0F for a
    // degenerate zero-size grid (never divides by zero).
    float exploredPercentage() const noexcept;

    // Row-major (index = row * width() + col) - exposed for
    // ExplorationMapStorage's bulk save, and Renderer3D's map-panel
    // drawing loop, so neither needs to call cellAt() once per cell
    // through a virtual-free but still per-call-overhead accessor.
    const std::vector<MapCell>& cells() const noexcept;

    // Replaces the entire grid's contents in one call - the one way an
    // Occupied cell can ever become non-Occupied again, since this
    // discards the whole previous grid rather than selectively
    // downgrading a single cell (see "occupancy precedence" above).
    // Returns false (no-op) if `cells.size()` does not equal width() *
    // height() - ExplorationMapStorage's own load-compatibility check
    // relies on this never silently accepting a mismatched grid.
    // Intended caller: ExplorationMapStorage::load() only.
    bool setCells(const std::vector<MapCell>& cells);

    // True since the last consumeDirty() call (or construction) if any
    // markFree()/markOccupied()/setCells() call actually changed a cell
    // - i.e. never set merely because markFree() was called on an
    // already-Free cell. Drives main3d.cpp's periodic-save-when-dirty
    // policy (Phase 13V brief, "do NOT write every frame").
    bool consumeDirty() noexcept;

private:
    std::size_t indexOf(int col, int row) const noexcept;

    TableSurface bounds_;
    int width_ = 0;
    int height_ = 0;
    std::vector<MapCell> cells_;
    bool dirty_ = false;
};

} // namespace robot::visual
