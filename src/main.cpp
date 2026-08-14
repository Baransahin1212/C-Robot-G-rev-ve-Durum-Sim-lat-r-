#include <filesystem>
#include <iostream>

#include "robot/Application.hpp"

int main(int argc, char** argv)
{
    return robot::app::runApplication(argc, argv,
                                       std::filesystem::path(LOGS_DIR),
                                       std::filesystem::path(REPORTS_DIR),
                                       std::cout,
                                       std::cerr);
}
