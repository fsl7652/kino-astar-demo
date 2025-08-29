cmake_minimum_required(VERSION 3.12)
project(JetRacerPathfinding VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific settings
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Find SDL3
find_package(SDL3 REQUIRED)

message(STATUS "SDL3 found: ${SDL3_LIBRARIES}")
message(STATUS "SDL3 includes: ${SDL3_INCLUDE_DIRS}")

# Include directories
include_directories(
    include
    ${SDL3_INCLUDE_DIRS}
)

# Source files 
set(SOURCES
    src/environment.cpp
    src/kino_search.cpp
    src/OBB.cpp
    src/wall.cpp
    src/jetracer.cpp
    src/goal.cpp
    src/render_circle.cpp
)


# Executable
add_executable(JetRacerDemo ${SOURCES})

# Link libraries 
target_link_libraries(JetRacerDemo ${SDL3_LIBRARIES})

# Set output directories
set_target_properties(JetRacerDemo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
)

# Installation
install(TARGETS JetRacerDemo RUNTIME DESTINATION bin)

# Post-build copy for Windows 
if(WIN32 AND SDL3_LIBRARY)
    # Get the path to the SDL3 DLL
    get_filename_component(SDL3_DLL_DIR "${SDL3_LIBRARY}" DIRECTORY)
    find_file(SDL3_DLL
        NAMES SDL3.dll
        PATHS ${SDL3_DLL_DIR} ${SDL3_ROOT}/bin
        NO_DEFAULT_PATH
    )
    
    if(SDL3_DLL)
        add_custom_command(TARGET JetRacerDemo POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SDL3_DLL}"
            "$<TARGET_FILE_DIR:JetRacerDemo>"
            COMMENT "Copying SDL3.dll to output directory"
        )
        
        # Also copy for install
        install(FILES ${SDL3_DLL} DESTINATION bin)
    endif()
endif()

# Enable testing
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    # Add tests here
endif()


include(CPack)