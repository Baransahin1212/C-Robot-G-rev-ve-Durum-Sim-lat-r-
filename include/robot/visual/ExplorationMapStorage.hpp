#pragma once

#include <string>

#include "robot/visual/CoverageTrail.hpp"
#include "robot/visual/ExplorationMap.hpp"

namespace robot::visual
{

// Outcome of ExplorationMapStorage::load() - deliberately distinct from a
// bool so main3d.cpp can show "Oluşturuluyor" (fresh) vs "Yüklendi"
// (loaded) without re-deriving which happened from side effects.
enum class MapLoadResult
{
    // A compatible map was found and `map`/`trail` now hold its contents.
    Loaded,
    // No file exists at `path` - not an error; `map`/`trail` are left
    // exactly as the caller constructed them (still all-Unknown/empty).
    MissingFile,
    // A file exists at `path` but is not valid JSON, or is missing a
    // required field - `map`/`trail` are left unchanged.
    Corrupt,
    // A file exists and is valid JSON, but its version/width/height/
    // resolution/bounds do not match `map`'s own construction - never
    // silently reinterpreted against a different grid shape. `map`/
    // `trail` are left unchanged.
    Incompatible
};

// Phase 13V: simple JSON persistence for one ExplorationMap (and
// optionally its paired CoverageTrail), via nlohmann/json - the same
// library robot_scenario/JsonScenarioSource.cpp already uses, PRIVATE to
// this target's own .cpp exactly like that precedent (never appearing in
// this header's public API). Raylib-free; has no FSM/Event/
// IRobotHardware/VirtualWorld knowledge of its own beyond the plain
// ExplorationMap/CoverageTrail types it reads and writes.
//
// Format (version 1): a JSON object with "version" (int), "width"/
// "height" (int, must equal map.width()/map.height() to load),
// "resolution" (float, must equal map.cellSize()), "bounds" (object:
// minX/maxX/minZ/maxZ, must equal map.bounds()), "cells" (a flat
// row-major array of ints, 0=Unknown/1=Free/2=Occupied, length
// width*height), and an optional "trail" (array of {x,y,z} objects).
class ExplorationMapStorage
{
public:
    static constexpr int kFormatVersion = 1;

    // Attempts to load `path` into `map` (and, if `trail` is non-null,
    // into `trail`). Never throws and never crashes on a missing,
    // corrupt, or version/shape-incompatible file - `map`/`trail` are
    // left exactly as passed in on anything other than MapLoadResult::
    // Loaded, so the caller can simply keep using them as a fresh map.
    // Compatibility is checked before ANY mutation - loading either
    // fully replaces `map`'s cells (and `trail`'s points, if given) or
    // touches neither at all, never a partial update.
    static MapLoadResult load(const std::string& path, ExplorationMap& map, CoverageTrail* trail = nullptr);

    // Writes `map` (and `trail`, if non-null) to `path` as JSON,
    // creating any missing parent directories first. Returns false (and
    // writes nothing) if the parent directory could not be created or
    // the file could not be opened for writing - never throws.
    static bool save(const std::string& path, const ExplorationMap& map, const CoverageTrail* trail = nullptr);

    // Phase 13Z: deletes ONLY the persisted exploration-map file at
    // `path` - never a directory, never anything else on disk (New Map's
    // own brief: "Do NOT recursively delete directories. Do NOT delete
    // logs/reports/assets/scenarios."). Returns true if the file was
    // deleted OR did not exist to begin with - a missing file is a
    // successful, no-op reset, mirroring MapLoadResult::MissingFile's own
    // "no file yet is not an error" precedent elsewhere in this class.
    // Only returns false on a genuine filesystem removal error (e.g.
    // permissions), logged to stderr exactly like every other failure
    // path in this class. Never throws.
    static bool remove(const std::string& path);
};

} // namespace robot::visual
