# Specify all dependencies fetched at configure time using FetchContent
include(FetchContent)

# Declare kassert dependency
FetchContent_Declare(
        kassert
        GIT_REPOSITORY https://github.com/kamping-site/kassert
        GIT_TAG v0.1.0
)

# Declare fast-cpp-csv-parser dependency
FetchContent_Declare(
        fast_cpp_csv_parser
        GIT_REPOSITORY https://github.com/ben-strasser/fast-cpp-csv-parser
)

# Declare vectorlclass dependency
FetchContent_Declare(
        vectorclass
        GIT_REPOSITORY https://github.com/vectorclass/version2.git
)

# Declare nlohmann_json dependency
FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        OVERRIDE_FIND_PACKAGE # Set so find_package(nlohmann_json) call in proj can redirect to this
)

# Declare proj dependency
FetchContent_Declare(
        proj
        URL https://download.osgeo.org/proj/proj-9.5.0.tar.gz
        URL_MD5 ac46b4e31562890d012ea6b31e579cf6
)

# Fetch kassert
message("Fetching kassert library...")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(KASSERT_ASSERTION_LEVEL 40)
else ()
    set(KASSERT_ASSERTION_LEVEL 20)
endif ()
FetchContent_MakeAvailable(kassert)

# Fetch fast-cpp-csv-parser (header only library, create interface target)
FetchContent_MakeAvailable(fast_cpp_csv_parser)
FetchContent_GetProperties(fast_cpp_csv_parser SOURCE_DIR fast_cpp_csv_parser_SOURCE_DIR)
add_library(fast_cpp_csv_parser INTERFACE)
target_include_directories(fast_cpp_csv_parser SYSTEM INTERFACE ${fast_cpp_csv_parser_SOURCE_DIR})

# Fetch vectorclass (header only library, create interface target)
FetchContent_MakeAvailable(vectorclass)
FetchContent_GetProperties(vectorclass SOURCE_DIR vectorclass_SOURCE_DIR)
add_library(vectorclass INTERFACE)
target_include_directories(vectorclass SYSTEM INTERFACE ${vectorclass_SOURCE_DIR})

# Fetch nlohmann_json
message("Fetching nlohmann_json library...")
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "")
FetchContent_MakeAvailable(nlohmann_json)
# proj finds nlohmann_json using a find_package() call. Using OVERRIDE_FIND_PACKAGE in FetchContent_Declare() of
# nlohmann_json makes sure that this find_package() call is redirected to the fetched nlohmann_json, ignoring
# version requirements.
# Unfortunately, proj manually checks version requirements using the nlohmann_json_VERSION variable. This variable is
# usually set by find_package() but cannot be set by FetchContent_Declare() with OVERRIDE_FIND_PACKAGE. Thus, we have
# to set the PACKAGE_VERSION variable manually in the find_package() redirect config for nlohmann_json:
set(nlohmann_json_VERSION 3.11.3)
file(APPEND
        ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/nlohmann_json-config-version.cmake
        "\n# Manually added PACKAGE_VERSION variable\nset(PACKAGE_VERSION ${nlohmann_json_VERSION})"
)

# Fetch proj
message("Fetching proj library...")
set(BUILD_APPS OFF)
set(BUILD_TESTING OFF)
set(ENABLE_CURL OFF)
set(ENABLE_TIFF OFF)
set(TESTING_USE_NETWORK OFF)
FetchContent_MakeAvailable(proj)