# Dependencies that have to be installed externally, e.g., via a package manager

# Find OpenMP (optional) installed via package manager
find_package(OpenMP)

# Find Intel TBB installed via package manager
find_package(TBB REQUIRED)

# Find HWLOC Library installed via package manager
find_path(HWLOC_INCLUDE_DIR NAME hwloc.h
        HINTS $ENV{HOME}/local/include /opt/local/include /usr/local/include /usr/include)
find_library(HWLOC_LIBRARY NAME hwloc
        HINTS $ENV{HOME}/local/lib64 $ENV{HOME}/local/lib /usr/local/lib64 /usr/local/lib /opt/local/lib64 /opt/local/lib /usr/lib64 /usr/lib
)

IF (HWLOC_INCLUDE_DIR AND HWLOC_LIBRARY)
    message(STATUS "Found hwloc library: inc=${HWLOC_INCLUDE_DIR}, lib=${HWLOC_LIBRARY}")
    add_library(hwloc UNKNOWN IMPORTED)
    set_property(TARGET hwloc PROPERTY
            IMPORTED_LOCATION ${HWLOC_LIBRARY})
    target_include_directories(hwloc INTERFACE ${HWLOC_INCLUDE_DIR})
ELSE ()
    message(FATAL_ERROR "
    HwLoc library not found. Install HwLoc on your system.")
ENDIF ()