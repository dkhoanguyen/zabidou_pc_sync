# FindArenaSDK.cmake
#
# Finds the Lucid Vision Arena SDK and exposes an imported target:
#   ArenaSDK::ArenaSDK
#
# Search order:
#   1. ARENA_ROOT environment variable
#   2. Common install prefixes: /opt/lucid/ArenaSDK, /opt/lucid, /usr/local, /usr
#
# Result variables:
#   ArenaSDK_FOUND
#   ArenaSDK_INCLUDE_DIRS
#   ArenaSDK_LIBRARIES
#   ArenaSDK_HAS_METAVISION   — TRUE if Metavision event-camera libs are usable

include(FindPackageHandleStandardArgs)

# Candidate root directories
set(_arena_search_roots
    "$ENV{ARENA_ROOT}"
    "/opt/lucid/ArenaSDK"
    "/opt/lucid"
    "/opt/ArenaSDK"
    "/usr/local"
    "/usr"
)

# Find the include directory (look for ArenaApi.h as the sentinel header)
find_path(ArenaSDK_INCLUDE_DIR
    NAMES ArenaApi.h
    PATHS ${_arena_search_roots}
    PATH_SUFFIXES
        include/Arena
        include
        Arena
    DOC "Arena SDK include directory (contains ArenaApi.h)"
)

# Find the main Arena library
find_library(ArenaSDK_LIBRARY
    NAMES arena Arena ArenaC ArenaApi
    PATHS ${_arena_search_roots}
    PATH_SUFFIXES
        lib64
        lib
        lib/linux_x64_release
        lib/x64Release
    DOC "Arena SDK main library"
)

# Also look for the GenICam include path (often a sub-directory of Arena)
find_path(ArenaSDK_GENICAM_INCLUDE_DIR
    NAMES Base/GCBase.h
    PATHS ${_arena_search_roots}
    PATH_SUFFIXES
        GenICam/library/CPP/include
        include/GenICam
        include
)

find_package_handle_standard_args(ArenaSDK
    REQUIRED_VARS ArenaSDK_INCLUDE_DIR ArenaSDK_LIBRARY
    VERSION_VAR   ArenaSDK_VERSION
)

# ---------------------------------------------------------------------------
# Optional: check whether Metavision event-camera support is usable.
# These checks are ADVISORY only — a missing Metavision runtime does not
# prevent using the core Arena API for RGB/depth cameras.
# ---------------------------------------------------------------------------
if(ArenaSDK_FOUND)
    # Derive the SDK root from the library path (e.g. /opt/lucid/ArenaSDK)
    get_filename_component(_arena_lib_dir "${ArenaSDK_LIBRARY}" DIRECTORY)
    get_filename_component(_arena_sdk_root "${_arena_lib_dir}" DIRECTORY)

    find_library(_ArenaSDK_METAVISION_CORE_LIB
        NAMES metavision_sdk_core
        PATHS "${_arena_sdk_root}/Metavision/lib"
        NO_DEFAULT_PATH
    )
    find_library(_ArenaSDK_METAVISION_BASE_LIB
        NAMES metavision_sdk_base
        PATHS "${_arena_sdk_root}/Metavision/lib"
        NO_DEFAULT_PATH
    )

    set(ArenaSDK_HAS_METAVISION FALSE)
    if(_ArenaSDK_METAVISION_CORE_LIB AND _ArenaSDK_METAVISION_BASE_LIB)
        # Metavision libs are present; check that their bundled ABI deps exist.
        find_file(_arena_mv_opencv_core
            NAMES libopencv_core.so.4.0
            PATHS "${_arena_sdk_root}/OpenCV/lib" "${_arena_sdk_root}/Metavision/lib" "${_arena_sdk_root}/lib64"
            NO_DEFAULT_PATH
        )
        find_file(_arena_mv_opencv_videoio
            NAMES libopencv_videoio.so.4.0
            PATHS "${_arena_sdk_root}/OpenCV/lib" "${_arena_sdk_root}/Metavision/lib" "${_arena_sdk_root}/lib64"
            NO_DEFAULT_PATH
        )
        if(_arena_mv_opencv_core AND _arena_mv_opencv_videoio)
            set(ArenaSDK_HAS_METAVISION TRUE)
        else()
            message(STATUS
                "ArenaSDK: Metavision event-camera support disabled "
                "(bundled OpenCV 4.0 sonames not found in ${_arena_sdk_root})")
        endif()
    endif()

    mark_as_advanced(
        _ArenaSDK_METAVISION_CORE_LIB
        _ArenaSDK_METAVISION_BASE_LIB
        _arena_mv_opencv_core
        _arena_mv_opencv_videoio
    )
endif()

# ---------------------------------------------------------------------------
# Create the imported target
# ---------------------------------------------------------------------------
if(ArenaSDK_FOUND AND NOT TARGET ArenaSDK::ArenaSDK)
    get_filename_component(_arena_lib_dir "${ArenaSDK_LIBRARY}" DIRECTORY)
    get_filename_component(_arena_sdk_root "${_arena_lib_dir}" DIRECTORY)

    set(_arena_genicam_lib_dir
        "${_arena_sdk_root}/GenICam/library/lib/Linux64_x64"
    )

    # Build the include list
    set(_arena_inc_dirs "${ArenaSDK_INCLUDE_DIR}")
    if(ArenaSDK_GENICAM_INCLUDE_DIR)
        list(APPEND _arena_inc_dirs "${ArenaSDK_GENICAM_INCLUDE_DIR}")
    endif()

    # Optional Metavision link libraries
    set(_arena_extra_libs "")
    if(ArenaSDK_HAS_METAVISION)
        list(APPEND _arena_extra_libs
            "${_ArenaSDK_METAVISION_CORE_LIB}"
            "${_ArenaSDK_METAVISION_BASE_LIB}"
        )
    endif()

    add_library(ArenaSDK::ArenaSDK SHARED IMPORTED)
    set_target_properties(ArenaSDK::ArenaSDK PROPERTIES
        IMPORTED_LOCATION             "${ArenaSDK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${_arena_inc_dirs}"
        INTERFACE_LINK_LIBRARIES      "${_arena_extra_libs}"
        # Link against the SDK lib directories so transitive deps resolve
        INTERFACE_LINK_DIRECTORIES    "${_arena_lib_dir};${_arena_genicam_lib_dir}"
        # Embed RPATH so executables find the SDK .so files at runtime
        INTERFACE_LINK_OPTIONS
            "-Wl,-rpath,${_arena_lib_dir},-rpath,${_arena_genicam_lib_dir}"
    )
endif()

mark_as_advanced(
    ArenaSDK_INCLUDE_DIR
    ArenaSDK_GENICAM_INCLUDE_DIR
    ArenaSDK_LIBRARY
)
