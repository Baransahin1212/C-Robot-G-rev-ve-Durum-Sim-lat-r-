#include "robot/visual/ExplorationMap.hpp"

#include <algorithm>
#include <cmath>

namespace robot::visual
{

namespace
{
int roundedCellCount(float worldSize, float cellSize)
{
    return std::max(1, static_cast<int>(std::lround(worldSize / cellSize)));
}
} // namespace

ExplorationMap::ExplorationMap(const TableSurface& bounds)
    : bounds_(bounds)
    , width_(roundedCellCount(bounds.maxX - bounds.minX, kCellSizeWorldUnits))
    , height_(roundedCellCount(bounds.maxZ - bounds.minZ, kCellSizeWorldUnits))
    , cells_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), MapCell::Unknown)
{
}

int ExplorationMap::width() const noexcept
{
    return width_;
}

int ExplorationMap::height() const noexcept
{
    return height_;
}

float ExplorationMap::cellSize() const noexcept
{
    return kCellSizeWorldUnits;
}

const TableSurface& ExplorationMap::bounds() const noexcept
{
    return bounds_;
}

bool ExplorationMap::isInsideGrid(int col, int row) const noexcept
{
    return col >= 0 && col < width_ && row >= 0 && row < height_;
}

std::size_t ExplorationMap::indexOf(int col, int row) const noexcept
{
    return (static_cast<std::size_t>(row) * static_cast<std::size_t>(width_)) + static_cast<std::size_t>(col);
}

MapCell ExplorationMap::cellAt(int col, int row) const noexcept
{
    if (!isInsideGrid(col, row))
    {
        return MapCell::Unknown;
    }
    return cells_[indexOf(col, row)];
}

bool ExplorationMap::worldToCell(const Vec3& worldPosition, int& outCol, int& outRow) const noexcept
{
    if (worldPosition.x < bounds_.minX || worldPosition.x >= bounds_.maxX || worldPosition.z < bounds_.minZ ||
        worldPosition.z >= bounds_.maxZ)
    {
        return false;
    }
    const int col = static_cast<int>(std::floor((worldPosition.x - bounds_.minX) / kCellSizeWorldUnits));
    const int row = static_cast<int>(std::floor((worldPosition.z - bounds_.minZ) / kCellSizeWorldUnits));
    if (!isInsideGrid(col, row))
    {
        return false;
    }
    outCol = col;
    outRow = row;
    return true;
}

Vec3 ExplorationMap::cellToWorld(int col, int row) const noexcept
{
    return Vec3{bounds_.minX + ((static_cast<float>(col) + 0.5F) * kCellSizeWorldUnits), 0.0F,
                bounds_.minZ + ((static_cast<float>(row) + 0.5F) * kCellSizeWorldUnits)};
}

void ExplorationMap::markFree(int col, int row) noexcept
{
    if (!isInsideGrid(col, row))
    {
        return;
    }
    const std::size_t index = indexOf(col, row);
    if (cells_[index] == MapCell::Unknown)
    {
        cells_[index] = MapCell::Free;
        dirty_ = true;
        ++revision_;
    }
}

void ExplorationMap::markOccupied(int col, int row) noexcept
{
    if (!isInsideGrid(col, row))
    {
        return;
    }
    const std::size_t index = indexOf(col, row);
    if (cells_[index] != MapCell::Occupied)
    {
        cells_[index] = MapCell::Occupied;
        dirty_ = true;
        ++revision_;
    }
}

std::size_t ExplorationMap::exploredCellCount() const noexcept
{
    return static_cast<std::size_t>(
        std::count_if(cells_.begin(), cells_.end(), [](MapCell cell) { return cell != MapCell::Unknown; }));
}

std::size_t ExplorationMap::totalCellCount() const noexcept
{
    return cells_.size();
}

float ExplorationMap::exploredPercentage() const noexcept
{
    if (cells_.empty())
    {
        return 0.0F;
    }
    return 100.0F * static_cast<float>(exploredCellCount()) / static_cast<float>(totalCellCount());
}

const std::vector<MapCell>& ExplorationMap::cells() const noexcept
{
    return cells_;
}

bool ExplorationMap::setCells(const std::vector<MapCell>& cells)
{
    if (cells.size() != cells_.size())
    {
        return false;
    }
    cells_ = cells;
    // Freshly loaded from disk - matches what is already persisted, so
    // this does not itself need an immediate re-save (mirrors
    // CoverageTrail::loadPoints()'s own reasoning).
    dirty_ = false;
    ++revision_;
    return true;
}

bool ExplorationMap::consumeDirty() noexcept
{
    const bool wasDirty = dirty_;
    dirty_ = false;
    return wasDirty;
}

std::uint64_t ExplorationMap::revision() const noexcept
{
    return revision_;
}

} // namespace robot::visual
