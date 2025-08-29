cmake_minimum_required(VERSION 3.12)
project(JetRacerPathfinding VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use pkg-config to find SDL3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL3 REQUIRED sdl3)

# Source files
set(SOURCES
    src/environment.cpp
    src/kino_search.cpp
    src/OBB.cpp
    src/wall.cpp
    src/jetracer.cpp
    src/goal.cpp
)

# Include directories
include_directories(include ${SDL3_INCLUDE_DIRS})

# Executable
add_executable(JetRacerDemo ${SOURCES})

# Link SDL3
target_link_libraries(JetRacerDemo ${SDL3_LIBRARIES})
