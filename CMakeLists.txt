# ******************************************************************************
# MIT License
#
# Copyright (c) 2020 Valentin Buchhold
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
option(DISABLE_AVX "Disable use of instructions in the AVX extended instruction set.")
if(DISABLE_AVX)
  if(CMAKE_CXX_COMPILER_ID MATCHES GNU|Clang)
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

find_package(OpenMP)
find_library(PROJ_LIBRARY proj)
find_library(RoutingKit_LIBRARY routingkit)



find_path(CSV_INCLUDE_PATH csv.h REQUIRED)
if (NOT CSV_INCLUDE_PATH)
  message(FATAL_ERROR "csv.h cannot be found. Aborting.")
endif ()
message("Found csv.h in ${CSV_INCLUDE_PATH}")
include_directories(SYSTEM ${CSV_INCLUDE_PATH})

find_path(RANDOMC_INCLUDE_PATH randomc.h REQUIRED PATHS ${RANDOMC_INCLUDE_PATH} HINTS ${CSV_INCLUDE_PATH} PATH_SUFFIXES randomc)
if (NOT RANDOMC_INCLUDE_PATH)
  message(FATAL_ERROR "randomc.h cannot be found. Aborting.")
endif ()
message("Found randomc.h in ${RANDOMC_INCLUDE_PATH}")
include_directories(${RANDOMC_INCLUDE_PATH})

find_path(ROUTINGKIT_INCLUDE_PATH contraction_hierarchy.h REQUIRED PATHS ${ROUTINGKIT_INCLUDE_PATH} HINTS ${CSV_INCLUDE_PATH} PATH_SUFFIXES routingkit)
if (NOT ROUTINGKIT_INCLUDE_PATH)
  message(FATAL_ERROR "RoutingKit headers cannot be found (searched for contraction_hierarchy.h). Aborting.")
endif ()
message("Found RoutingKit headers in ${ROUTINGKIT_INCLUDE_PATH} (searched for contraction_hierarchy.h)")
include_directories(${ROUTINGKIT_INCLUDE_PATH})

find_path(STOCC_INCLUDE_PATH stocc.h REQUIRED PATHS ${STOCC_INCLUDE_PATH} HINTS ${CSV_INCLUDE_PATH} PATH_SUFFIXES stocc)
if (NOT STOCC_INCLUDE_PATH)
  message(FATAL_ERROR "stocc.h cannot be found. Aborting.")
endif ()
message("Found stocc.h in ${STOCC_INCLUDE_PATH}")
include_directories(${STOCC_INCLUDE_PATH})

find_path(VECTORCLASS_INCLUDE_PATH vectorclass.h REQUIRED PATHS ${VECTORCLASS_INCLUDE_PATH} HINTS ${CSV_INCLUDE_PATH} PATH_SUFFIXES vectorclass)
if (NOT VECTORCLASS_INCLUDE_PATH)
  message(FATAL_ERROR "vectorclass.h cannot be found. Aborting.")
endif ()
message("Found vectorclass.h in ${VECTORCLASS_INCLUDE_PATH}")
include_directories(${VECTORCLASS_INCLUDE_PATH})

foreach(SRC_DIR ${SOURCE_DIRECTORIES})
  add_subdirectory(${SRC_DIR})
endforeach()
