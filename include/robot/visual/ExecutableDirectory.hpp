#pragma once

#include <string>

namespace robot::visual
{

// Final font delivery fix / Phase 13V: the directory the current process's
// own executable (RobotSimulator3D.exe) is running from - resolved at
// runtime via the OS, never a compile-time source-tree path, so a
// checkout built on a different machine still finds its runtime-relative
// assets/data. Two callers as of Phase 13V: Renderer3D's constructor
// (assets/fonts/anonymous_pro_bold.ttf) and main3d.cpp's exploration-map
// persistence path (runtime/maps/exploration_map.json) - extracted here,
// out of Renderer3D.cpp where it originated, so both can share one
// implementation instead of two copies. Raylib-free (plain <windows.h>,
// no raylib.h anywhere in this translation unit), so it needs none of
// Renderer3D.cpp's NOGDI/NOUSER/NOMINMAX guards.
//
// Returns an empty string if the OS call fails for any reason - never
// throws or crashes; callers treat that exactly like "path could not be
// resolved" and fall back accordingly (Renderer3D falls back to
// GetFontDefault(); the map storage path falls back to starting an
// unpersisted in-memory map for that session).
std::string executableDirectory();

} // namespace robot::visual
