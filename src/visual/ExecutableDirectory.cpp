#include "robot/visual/ExecutableDirectory.hpp"

#include <windows.h>

namespace robot::visual
{

std::string executableDirectory()
{
    char buffer[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
    if (length == 0 || length == MAX_PATH)
    {
        return std::string();
    }
    const std::string path(buffer, length);
    const std::size_t lastSlash = path.find_last_of("\\/");
    return (lastSlash == std::string::npos) ? std::string() : path.substr(0, lastSlash);
}

} // namespace robot::visual
