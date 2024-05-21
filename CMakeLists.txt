# ******************************************************************************
# MIT License
#
# Copyright (c) 2020 Valentin Buchhold
# Copyright (c) 2023 Moritz Laupichler
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ******************************************************************************


cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(RoutingFramework CXX)

# Directories containing .cc source files.
set(SOURCE_DIRECTORIES Launchers RawData)

# Determine the language standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Flags when building for the Devel configuration.
if(CMAKE_CXX_COMPILER_ID MATCHES GNU|Clang)
  set(CMAKE_CXX_FLAGS_DEVEL -O3)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Devel)
endif()

# Enable the compiler to use extended instructions in generated code.
if(CMAKE_CXX_COMPILER_ID MATCHES GNU|Clang)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
option(DISABLE_AVX "Disable use of instructions in the AVX extended instruction set." OFF)
if(DISABLE_AVX)
  if(CMAKE_CXX_COMPILER_ID MATCHES GNU|Clang)
    message("Disabling use of AVX instructions.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-avx")
  endif()
endif()

# Request warnings.
if(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wfatal-errors -Wpedantic -pedantic-errors\
      -Wall -Wextra -ftemplate-backtrace-limit=1 -Wno-unknown-pragmas -Wno-sign-compare")
elseif(CMAKE_CXX_COMPILER_ID MATCHES Clang)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -pedantic-errors -ferror-limit=1\
      -ftemplate-backtrace-limit=1 -Wno-sign-compare")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

include_directories(${CMAKE_SOURCE_DIR})

# Find openmp and proj libraries installed via package manager
find_package(OpenMP)
find_library(PROJ_LIBRARY proj)

# Add json library from git submodule
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "")
add_subdirectory(External/json)

# Add RoutingKit library (needs to be built beforehand!) and include headers from git submodule
find_library(RoutingKit_LIBRARY routingkit PATHS External/RoutingKit/lib REQUIRED)
include_directories(External/RoutingKit/include)

# Add kassert library from git submodule
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(KASSERT_ASSERTION_LEVEL 40)
else()
  set(KASSERT_ASSERTION_LEVEL 20)
endif ()
add_subdirectory(External/kassert)

# Include headers for header-only libraries from git submodules and embedded dependencies
include_directories(SYSTEM External/fast-cpp-csv-parser)
include_directories(External/randomc)
include_directories(External/rapidxml)
include_directories(External/stocc)
include_directories(External/vectorclass)


foreach(SRC_DIR ${SOURCE_DIRECTORIES})
  add_subdirectory(${SRC_DIR})
endforeach()
