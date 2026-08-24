#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "robot/visual/CoverageTrail.hpp"
#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/ExplorationMapStorage.hpp"

namespace
{

using robot::visual::CoverageTrail;
using robot::visual::ExplorationMap;
using robot::visual::ExplorationMapStorage;
using robot::visual::MapCell;
using robot::visual::MapLoadResult;
using robot::visual::RobotPose;
using robot::visual::TableSurface;
using robot::visual::Vec3;

TableSurface demoTable()
{
    return TableSurface{-6.0F, 6.0F, -6.0F, 6.0F};
}

// Never the real runtime map path (executableDirectory() + "runtime/
// maps/...") - always a TEST_OUTPUT_DIR-rooted temporary file, per this
// phase's own brief ("Tests must use temporary files. Never write to the
// real user map path.").
std::string testFilePath(const std::string& name)
{
    const std::filesystem::path dir = std::filesystem::path(TEST_OUTPUT_DIR) / "exploration_map_storage";
    std::filesystem::create_directories(dir);
    return (dir / name).string();
}

} // namespace

// 1: MissingFileStartsFresh
TEST(ExplorationMapStorageTest, MissingFileStartsFresh)
{
    ExplorationMap map(demoTable());
    const std::string path = testFilePath("missing_file_does_not_exist.json");
    std::filesystem::remove(path);

    const MapLoadResult result = ExplorationMapStorage::load(path, map);

    EXPECT_EQ(result, MapLoadResult::MissingFile);
    EXPECT_EQ(map.exploredCellCount(), 0U);
}

// 2: SaveThenLoadPreservesMap
TEST(ExplorationMapStorageTest, SaveThenLoadPreservesMap)
{
    const std::string path = testFilePath("save_then_load.json");

    ExplorationMap savedMap(demoTable());
    savedMap.markFree(1, 1);
    savedMap.markOccupied(2, 2);
    ASSERT_TRUE(ExplorationMapStorage::save(path, savedMap));

    ExplorationMap loadedMap(demoTable());
    const MapLoadResult result = ExplorationMapStorage::load(path, loadedMap);

    ASSERT_EQ(result, MapLoadResult::Loaded);
    EXPECT_EQ(loadedMap.cellAt(1, 1), MapCell::Free);
    EXPECT_EQ(loadedMap.cellAt(2, 2), MapCell::Occupied);
}

// 3: VersionIsStored
TEST(ExplorationMapStorageTest, VersionIsStored)
{
    const std::string path = testFilePath("version_is_stored.json");
    ExplorationMap map(demoTable());
    ASSERT_TRUE(ExplorationMapStorage::save(path, map));

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("\"version\""), std::string::npos);
}

// 4: CorruptFileFailsSafely
TEST(ExplorationMapStorageTest, CorruptFileFailsSafely)
{
    const std::string path = testFilePath("corrupt.json");
    std::ofstream file(path);
    file << "{ this is not valid json";
    file.close();

    ExplorationMap map(demoTable());
    map.markOccupied(0, 0);
    const MapLoadResult result = ExplorationMapStorage::load(path, map);

    EXPECT_EQ(result, MapLoadResult::Corrupt);
    // Left unchanged, never crashes.
    EXPECT_EQ(map.cellAt(0, 0), MapCell::Occupied);
}

// 5: UnsupportedVersionFailsSafely
TEST(ExplorationMapStorageTest, UnsupportedVersionFailsSafely)
{
    const std::string path = testFilePath("unsupported_version.json");
    {
        ExplorationMap map(demoTable());
        ASSERT_TRUE(ExplorationMapStorage::save(path, map));
    }

    // Bump the version field to something this build does not support.
    std::string contents;
    {
        std::ifstream in(path);
        contents.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const std::string original = "\"version\":" + std::to_string(ExplorationMapStorage::kFormatVersion);
    const std::string replaced = "\"version\":9999";
    const std::size_t pos = contents.find(original);
    ASSERT_NE(pos, std::string::npos);
    contents.replace(pos, original.size(), replaced);
    {
        std::ofstream out(path, std::ios::trunc);
        out << contents;
    }

    ExplorationMap map(demoTable());
    const MapLoadResult result = ExplorationMapStorage::load(path, map);

    EXPECT_EQ(result, MapLoadResult::Incompatible);
    EXPECT_EQ(map.exploredCellCount(), 0U);
}

// 6: ResolutionPreserved
TEST(ExplorationMapStorageTest, ResolutionPreserved)
{
    const std::string path = testFilePath("resolution_preserved.json");
    ExplorationMap savedMap(demoTable());
    ASSERT_TRUE(ExplorationMapStorage::save(path, savedMap));

    ExplorationMap loadedMap(demoTable());
    ASSERT_EQ(ExplorationMapStorage::load(path, loadedMap), MapLoadResult::Loaded);
    EXPECT_FLOAT_EQ(loadedMap.cellSize(), savedMap.cellSize());
    EXPECT_EQ(loadedMap.width(), savedMap.width());
    EXPECT_EQ(loadedMap.height(), savedMap.height());
}

// 7: BoundsPreserved
TEST(ExplorationMapStorageTest, BoundsPreserved)
{
    const std::string path = testFilePath("bounds_preserved.json");
    ExplorationMap savedMap(demoTable());
    ASSERT_TRUE(ExplorationMapStorage::save(path, savedMap));

    // A map constructed with DIFFERENT bounds must be rejected as
    // Incompatible - bounds are never silently reinterpreted.
    ExplorationMap differentBoundsMap(TableSurface{-3.0F, 3.0F, -3.0F, 3.0F});
    EXPECT_EQ(ExplorationMapStorage::load(path, differentBoundsMap), MapLoadResult::Incompatible);

    ExplorationMap sameBoundsMap(demoTable());
    EXPECT_EQ(ExplorationMapStorage::load(path, sameBoundsMap), MapLoadResult::Loaded);
}

// 8: OccupiedCellsPreserved
TEST(ExplorationMapStorageTest, OccupiedCellsPreserved)
{
    const std::string path = testFilePath("occupied_cells_preserved.json");

    ExplorationMap savedMap(demoTable());
    savedMap.markOccupied(10, 20);
    savedMap.markOccupied(50, 50);
    savedMap.markFree(0, 0);
    ASSERT_TRUE(ExplorationMapStorage::save(path, savedMap));

    ExplorationMap loadedMap(demoTable());
    ASSERT_EQ(ExplorationMapStorage::load(path, loadedMap), MapLoadResult::Loaded);
    EXPECT_EQ(loadedMap.cellAt(10, 20), MapCell::Occupied);
    EXPECT_EQ(loadedMap.cellAt(50, 50), MapCell::Occupied);
    EXPECT_EQ(loadedMap.cellAt(0, 0), MapCell::Free);
    EXPECT_EQ(loadedMap.cellAt(1, 1), MapCell::Unknown);
}

// 9: TrailPreservedIfStored
TEST(ExplorationMapStorageTest, TrailPreservedIfStored)
{
    const std::string path = testFilePath("trail_preserved.json");

    ExplorationMap savedMap(demoTable());
    CoverageTrail savedTrail;
    savedTrail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    savedTrail.update(RobotPose{Vec3{1.0F, 0.0F, 1.0F}, 0.0F});
    ASSERT_TRUE(ExplorationMapStorage::save(path, savedMap, &savedTrail));

    ExplorationMap loadedMap(demoTable());
    CoverageTrail loadedTrail;
    ASSERT_EQ(ExplorationMapStorage::load(path, loadedMap, &loadedTrail), MapLoadResult::Loaded);

    ASSERT_EQ(loadedTrail.points().size(), savedTrail.points().size());
    EXPECT_FLOAT_EQ(loadedTrail.points().back().x, 1.0F);
    EXPECT_FLOAT_EQ(loadedTrail.points().back().z, 1.0F);
}
