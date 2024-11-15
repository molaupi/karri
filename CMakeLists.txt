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
set(CMAKE_CXX_STANDARD 20)
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
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  set(FULL_WARNINGS "-Werror" "-Wfatal-errors" "-Wpedantic" "-pedantic-errors" "-Wall" "-Wextra" "-ftemplate-backtrace-limit=1" "-Wno-unknown-pragmas" "-Wno-sign-compare")
elseif(CMAKE_CXX_COMPILER_ID MATCHES Clang)
  set(FULL_WARNINGS "-Werror" "-Wall" "-Wextra" "-pedantic-errors" "-ferror-limit=1" "-ftemplate-backtrace-limit=1" "-Wno-sign-compare")
endif()

include_directories(${CMAKE_SOURCE_DIR})

# Find openmp libraries installed via package manager
find_package(OpenMP)
find_library(PROJ_LIBRARY proj)

# Add json library from git submodule
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "")
add_subdirectory(External/json)

# Add RoutingKit library (needs to be built beforehand!) and include headers from git submodule.
# Use interface target to bundle sources and headers.
find_library(RoutingKit_LIBRARY routingkit PATHS External/RoutingKit/lib)
if(NOT RoutingKit_LIBRARY)
  message(FATAL_ERROR "RoutingKit library not found. Did you forget to build the library (see ReadMe)?")
endif()
set(RoutingKit_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/External/RoutingKit/include/")
add_library(routingkit INTERFACE)
target_link_libraries(routingkit INTERFACE ${RoutingKit_LIBRARY})
target_include_directories(routingkit SYSTEM INTERFACE ${RoutingKit_INCLUDE_DIR}) # include as SYSTEM to avoid warnings for features deprecated in C++20 as RoutingKit is C++17

# Add kassert library from git submodule
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(KASSERT_ASSERTION_LEVEL 40)
else()
  set(KASSERT_ASSERTION_LEVEL 20)
endif ()
add_subdirectory(External/kassert)

set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/External)


foreach(SRC_DIR ${SOURCE_DIRECTORIES})
  add_subdirectory(${SRC_DIR})
endforeach()
