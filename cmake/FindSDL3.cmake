
find_path(SDL3_INCLUDE_DIR
    NAMES SDL3/SDL.h
    PATHS
        ${SDL3_ROOT}/include
        /usr/include
        /usr/local/include
        /opt/homebrew/include
        C:/SDL3/include
)

find_library(SDL3_LIBRARY
    NAMES SDL3
    PATHS
        ${SDL3_ROOT}/lib
        /usr/lib
        /usr/local/lib
        /opt/homebrew/lib
        C:/SDL3/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SDL3
    REQUIRED_VARS SDL3_LIBRARY SDL3_INCLUDE_DIR
)

if(SDL3_FOUND)
    set(SDL3_LIBRARIES ${SDL3_LIBRARY})
    set(SDL3_INCLUDE_DIRS ${SDL3_INCLUDE_DIR})
    
    if(NOT TARGET SDL3::SDL3)
        add_library(SDL3::SDL3 UNKNOWN IMPORTED)
        set_target_properties(SDL3::SDL3 PROPERTIES
            IMPORTED_LOCATION "${SDL3_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${SDL3_INCLUDE_DIR}"
        )
    endif()
endif()

mark_as_advanced(SDL3_INCLUDE_DIR SDL3_LIBRARY)