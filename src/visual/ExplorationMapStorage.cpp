#include "robot/visual/ExplorationMapStorage.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace robot::visual
{

namespace
{

int cellToInt(MapCell cell)
{
    switch (cell)
    {
        case MapCell::Unknown: return 0;
        case MapCell::Free: return 1;
        case MapCell::Occupied: return 2;
    }
    return 0;
}

// Any out-of-range/unrecognized integer maps to Unknown rather than
// throwing - a corrupt cell value should not crash the whole load; the
// caller-visible compatibility check (dimensions/resolution/bounds) is
// what actually decides MapLoadResult, not individual cell values.
MapCell intToCell(int value)
{
    switch (value)
    {
        case 1: return MapCell::Free;
        case 2: return MapCell::Occupied;
        default: return MapCell::Unknown;
    }
}

constexpr float kToleranceWorldUnits = 0.0001F;

bool nearlyEqual(float a, float b)
{
    return std::fabs(a - b) <= kToleranceWorldUnits;
}

} // namespace

MapLoadResult ExplorationMapStorage::load(const std::string& path, ExplorationMap& map, CoverageTrail* trail)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
    {
        return MapLoadResult::MissingFile;
    }

    nlohmann::json root;
    try
    {
        file >> root;
    }
    catch (const nlohmann::json::exception&)
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" is not valid JSON - starting a new map.\n", path.c_str());
        return MapLoadResult::Corrupt;
    }

    if (!root.is_object() || !root.contains("version") || !root.contains("width") || !root.contains("height") ||
        !root.contains("resolution") || !root.contains("bounds") || !root.contains("cells"))
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" is missing a required field - starting a new map.\n",
                      path.c_str());
        return MapLoadResult::Corrupt;
    }

    const nlohmann::json& bounds = root["bounds"];
    if (!bounds.is_object() || !bounds.contains("minX") || !bounds.contains("maxX") || !bounds.contains("minZ") ||
        !bounds.contains("maxZ") || !root["cells"].is_array())
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" is missing a required field - starting a new map.\n",
                      path.c_str());
        return MapLoadResult::Corrupt;
    }

    int version = 0;
    int width = 0;
    int height = 0;
    float resolution = 0.0F;
    try
    {
        version = root["version"].get<int>();
        width = root["width"].get<int>();
        height = root["height"].get<int>();
        resolution = root["resolution"].get<float>();
    }
    catch (const nlohmann::json::exception&)
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" has a malformed field - starting a new map.\n",
                      path.c_str());
        return MapLoadResult::Corrupt;
    }

    if (version != ExplorationMapStorage::kFormatVersion)
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" is version %d, this build expects version %d - starting a new map.\n",
                      path.c_str(), version, ExplorationMapStorage::kFormatVersion);
        return MapLoadResult::Incompatible;
    }

    const TableSurface& currentBounds = map.bounds();
    const bool boundsMatch = nearlyEqual(bounds.value("minX", 0.0F), currentBounds.minX) &&
                              nearlyEqual(bounds.value("maxX", 0.0F), currentBounds.maxX) &&
                              nearlyEqual(bounds.value("minZ", 0.0F), currentBounds.minZ) &&
                              nearlyEqual(bounds.value("maxZ", 0.0F), currentBounds.maxZ);

    if (width != map.width() || height != map.height() || !nearlyEqual(resolution, map.cellSize()) || !boundsMatch)
    {
        std::fprintf(stderr,
                      "ExplorationMapStorage: \"%s\" does not match this build's current grid dimensions/resolution/"
                      "bounds - starting a new map.\n",
                      path.c_str());
        return MapLoadResult::Incompatible;
    }

    const nlohmann::json& cellsJson = root["cells"];
    if (cellsJson.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" has a cell count that does not match its own width/height - starting a new map.\n",
                      path.c_str());
        return MapLoadResult::Corrupt;
    }

    std::vector<MapCell> cells;
    cells.reserve(cellsJson.size());
    try
    {
        for (const nlohmann::json& cellValue : cellsJson)
        {
            cells.push_back(intToCell(cellValue.get<int>()));
        }
    }
    catch (const nlohmann::json::exception&)
    {
        std::fprintf(stderr, "ExplorationMapStorage: \"%s\" has a malformed cell value - starting a new map.\n",
                      path.c_str());
        return MapLoadResult::Corrupt;
    }

    std::vector<Vec3> trailPoints;
    if (trail != nullptr && root.contains("trail") && root["trail"].is_array())
    {
        try
        {
            for (const nlohmann::json& pointJson : root["trail"])
            {
                trailPoints.push_back(
                    Vec3{pointJson.value("x", 0.0F), pointJson.value("y", 0.0F), pointJson.value("z", 0.0F)});
            }
        }
        catch (const nlohmann::json::exception&)
        {
            // A malformed trail does not invalidate an otherwise-valid
            // map - fall back to an empty trail rather than discarding
            // the whole load.
            trailPoints.clear();
        }
    }

    // Compatibility fully confirmed - mutate map/trail only now, all at
    // once (never a partial update on a later failure).
    map.setCells(cells);
    if (trail != nullptr)
    {
        trail->loadPoints(std::move(trailPoints));
    }

    return MapLoadResult::Loaded;
}

bool ExplorationMapStorage::save(const std::string& path, const ExplorationMap& map, const CoverageTrail* trail)
{
    const std::filesystem::path filePath(path);
    std::error_code errorCode;
    if (filePath.has_parent_path())
    {
        std::filesystem::create_directories(filePath.parent_path(), errorCode);
        if (errorCode)
        {
            std::fprintf(stderr, "ExplorationMapStorage: could not create directory for \"%s\" - map not saved.\n",
                          path.c_str());
            return false;
        }
    }

    nlohmann::json root;
    root["version"] = kFormatVersion;
    root["width"] = map.width();
    root["height"] = map.height();
    root["resolution"] = map.cellSize();
    root["bounds"] = {
        {"minX", map.bounds().minX},
        {"maxX", map.bounds().maxX},
        {"minZ", map.bounds().minZ},
        {"maxZ", map.bounds().maxZ},
    };

    nlohmann::json cellsJson = nlohmann::json::array();
    for (MapCell cell : map.cells())
    {
        cellsJson.push_back(cellToInt(cell));
    }
    root["cells"] = std::move(cellsJson);

    if (trail != nullptr)
    {
        nlohmann::json trailJson = nlohmann::json::array();
        for (const Vec3& point : trail->points())
        {
            trailJson.push_back({{"x", point.x}, {"y", point.y}, {"z", point.z}});
        }
        root["trail"] = std::move(trailJson);
    }

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open())
    {
        std::fprintf(stderr, "ExplorationMapStorage: could not open \"%s\" for writing - map not saved.\n", path.c_str());
        return false;
    }
    file << root.dump();
    return file.good();
}

bool ExplorationMapStorage::remove(const std::string& path)
{
    std::error_code errorCode;
    std::filesystem::remove(path, errorCode);
    if (errorCode)
    {
        std::fprintf(stderr, "ExplorationMapStorage: could not delete \"%s\" - %s.\n", path.c_str(),
                      errorCode.message().c_str());
        return false;
    }
    return true;
}

} // namespace robot::visual
