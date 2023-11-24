# ******************************************************************************
# MIT License
#
# Copyright (c) 2020 Valentin Buchhold
# Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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


add_executable(RunP2PAlgo RunP2PAlgo.cc)
target_compile_definitions(RunP2PAlgo PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(RunP2PAlgo ${RoutingKit_LIBRARY})
if(OpenMP_FOUND)
    target_link_libraries(RunP2PAlgo OpenMP::OpenMP_CXX)
endif()


add_executable(TransportSimulationMock TransportSimulationMock.cpp)

# KaRRi Simulation
add_executable(karri KaRRi.cpp
        MockMobitoppRequests.h)
target_compile_definitions(karri PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(karri ${RoutingKit_LIBRARY})
if(OpenMP_FOUND)
    target_link_libraries(karri OpenMP::OpenMP_CXX)
endif()

# KaRRi compile time parameters

## Output vehicle paths?
option(KARRI_OUTPUT_VEHICLE_PATHS "Output vehicle paths." OFF)
if (KARRI_OUTPUT_VEHICLE_PATHS)
    target_compile_definitions(karri PRIVATE KARRI_OUTPUT_VEHICLE_PATHS)
endif (KARRI_OUTPUT_VEHICLE_PATHS)

# Cost function tuning
set(KARRI_PSG_COST_SCALE 1 CACHE STRING "Importance of passenger trip times in cost function.")
target_compile_definitions(karri PRIVATE KARRI_PSG_COST_SCALE=${KARRI_PSG_COST_SCALE})

set(KARRI_WALKING_COST_SCALE 0 CACHE STRING "Importance of walking times in cost function")
target_compile_definitions(karri PRIVATE KARRI_WALKING_COST_SCALE=${KARRI_WALKING_COST_SCALE})

set(KARRI_VEH_COST_SCALE 1 CACHE STRING "Importance of vehicle travel times in cost function.")
target_compile_definitions(karri PRIVATE KARRI_VEH_COST_SCALE=${KARRI_VEH_COST_SCALE})

set(KARRI_WAIT_PENALTY_SCALE 1 CACHE STRING "Weights penalties for violating wait time soft constraint.")
target_compile_definitions(karri PRIVATE KARRI_WAIT_PENALTY_SCALE=${KARRI_WAIT_PENALTY_SCALE})

set(KARRI_TRIP_PENALTY_SCALE 10 CACHE STRING "Weights penalties for violating trip time soft constraint.")
target_compile_definitions(karri PRIVATE KARRI_TRIP_PENALTY_SCALE=${KARRI_TRIP_PENALTY_SCALE})

# Use CCHs?
option(KARRI_USE_CCHS "Use CCHs instead of standard CHs." OFF)
if (KARRI_USE_CCHS)
    target_compile_definitions(karri PRIVATE KARRI_USE_CCHS)
endif (KARRI_USE_CCHS)

## Elliptic BCH Searches Config
set(KARRI_ELLIPTIC_BCH_LOG_K 0 CACHE STRING "Given value i, KaRRi runs 2^i elliptic BCH searches simultaneously as bundled search.")
target_compile_definitions(karri PRIVATE KARRI_ELLIPTIC_BCH_LOG_K=${KARRI_ELLIPTIC_BCH_LOG_K})

option(KARRI_ELLIPTIC_BCH_USE_SIMD "Use SIMD instructions for bundled elliptic BCH searches." OFF)
if (KARRI_ELLIPTIC_BCH_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_ELLIPTIC_BCH_USE_SIMD=true)
else(KARRI_ELLIPTIC_BCH_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_ELLIPTIC_BCH_USE_SIMD=false)
endif (KARRI_ELLIPTIC_BCH_USE_SIMD)

option(KARRI_ELLIPTIC_BCH_SORTED_BUCKETS "Use sorted buckets for elliptic BCH searches." ON)
if (KARRI_ELLIPTIC_BCH_SORTED_BUCKETS)
    target_compile_definitions(karri PRIVATE KARRI_ELLIPTIC_BCH_SORTED_BUCKETS=true)
else(KARRI_ELLIPTIC_BCH_SORTED_BUCKETS)
    target_compile_definitions(karri PRIVATE KARRI_ELLIPTIC_BCH_SORTED_BUCKETS=false)
endif (KARRI_ELLIPTIC_BCH_SORTED_BUCKETS)

## PD-Distance Searches Config
set(PD_BCH_CODE 1)
set(PD_CH_CODE 2)
target_compile_definitions(karri PRIVATE KARRI_BCH_PD_STRAT=${PD_BCH_CODE})
target_compile_definitions(karri PRIVATE KARRI_CH_PD_STRAT=${PD_CH_CODE})
set(KARRI_PD_STRAT_ARG_VALUES BCH CH)

set(KARRI_PD_STRATEGY BCH CACHE STRING "PD-Distance query strategy to use. Possible values: BCH (dflt), CH")
set_property(CACHE KARRI_PD_STRATEGY PROPERTY STRINGS ${KARRI_PD_STRAT_ARG_VALUES})
list(FIND KARRI_PD_STRAT_ARG_VALUES ${KARRI_PD_STRATEGY} index)
if (index EQUAL -1)
    message(FATAL_ERROR "KARRI_PD_STRATEGY must be one of ${KARRI_PD_STRAT_ARG_VALUES}")
endif ()

if (${KARRI_PD_STRATEGY} STREQUAL BCH)
    target_compile_definitions(karri PRIVATE KARRI_PD_STRATEGY=${PD_BCH_CODE}) # BCH Strategy
else()
    target_compile_definitions(karri PRIVATE KARRI_PD_STRATEGY=${PD_CH_CODE}) # P2P CH Strategy
endif()

set(KARRI_PD_DISTANCES_LOG_K 0 CACHE STRING "Given value i, KaRRi runs 2^i PD-distance searches simultaneously as bundled search.")
target_compile_definitions(karri PRIVATE KARRI_PD_DISTANCES_LOG_K=${KARRI_PD_DISTANCES_LOG_K})

option(KARRI_PD_DISTANCES_USE_SIMD "Use SIMD instructions for bundled PD-distance searches." OFF)
if (KARRI_PD_DISTANCES_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_PD_DISTANCES_USE_SIMD=true)
else(KARRI_PD_DISTANCES_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_PD_DISTANCES_USE_SIMD=false)
endif (KARRI_PD_DISTANCES_USE_SIMD)

## Last Stop Search Strategies (PALS and DALS)
set(COL_CODE 1)
set(IND_CODE 2)
set(DIJ_CODE 3)
target_compile_definitions(karri PRIVATE KARRI_COL=${COL_CODE})
target_compile_definitions(karri PRIVATE KARRI_IND=${IND_CODE})
target_compile_definitions(karri PRIVATE KARRI_DIJ=${DIJ_CODE})
set(KARRI_STRAT_ARG_VALUES COL IND DIJ)

set(KARRI_PALS_STRATEGY COL CACHE STRING "Pickup-After-Last-Stop strategy to use. Possible values: COL, IND, DIJ")
set_property(CACHE KARRI_PALS_STRATEGY PROPERTY STRINGS ${KARRI_STRAT_ARG_VALUES})
list(FIND KARRI_STRAT_ARG_VALUES ${KARRI_PALS_STRATEGY} index)
if (index EQUAL -1)
    message(FATAL_ERROR "KARRI_PALS_STRATEGY must be one of ${KARRI_STRAT_ARG_VALUES}")
endif ()

if (${KARRI_PALS_STRATEGY} STREQUAL COL)
    target_compile_definitions(karri PRIVATE KARRI_PALS_STRATEGY=${COL_CODE}) # Collective BCH
elseif(${KARRI_PALS_STRATEGY} STREQUAL IND)
    target_compile_definitions(karri PRIVATE KARRI_PALS_STRATEGY=${IND_CODE}) # Individual BCH
else()
    target_compile_definitions(karri PRIVATE KARRI_PALS_STRATEGY=${DIJ_CODE}) # Dijkstra
endif()

set(KARRI_DALS_STRATEGY COL CACHE STRING "Dropoff-After-Last-Stop strategy to use. Possible values: COL, IND, DIJ")
set_property(CACHE KARRI_DALS_STRATEGY PROPERTY STRINGS ${KARRI_STRAT_ARG_VALUES})
list(FIND KARRI_STRAT_ARG_VALUES ${KARRI_DALS_STRATEGY} index)
if (index EQUAL -1)
    message(FATAL_ERROR "KARRI_DALS_STRATEGY must be one of ${KARRI_STRAT_ARG_VALUES}")
endif ()

if (${KARRI_DALS_STRATEGY} STREQUAL COL)
    target_compile_definitions(karri PRIVATE KARRI_DALS_STRATEGY=${COL_CODE}) # Collective BCH
elseif(${KARRI_DALS_STRATEGY} STREQUAL IND)
    target_compile_definitions(karri PRIVATE KARRI_DALS_STRATEGY=${IND_CODE}) # Individual BCH
else()
    target_compile_definitions(karri PRIVATE KARRI_DALS_STRATEGY=${DIJ_CODE}) # Dijkstra
endif()

## Options for last stop search strategies (not all options apply to all strategies)

### Applies for IND, DIJ, and fallback of COL
set(KARRI_PALS_LOG_K 0 CACHE STRING "Given value i, KaRRi runs 2^i PALS searches simultaneously as bundled search. (Applies for IND, DIJ, and fallback of COL)")
target_compile_definitions(karri PRIVATE KARRI_PALS_LOG_K=${KARRI_PALS_LOG_K})

### Applies for IND, DIJ, and fallback of COL
option(KARRI_PALS_USE_SIMD "Use SIMD instructions for bundled PALS searches. (Applies for IND, DIJ, and fallback of COL)" OFF)
if (KARRI_PALS_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_PALS_USE_SIMD=true)
else(KARRI_PALS_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_PALS_USE_SIMD=false)
endif (KARRI_PALS_USE_SIMD)

### Applies for IND, DIJ
set(KARRI_DALS_LOG_K 0 CACHE STRING "Given value i, KaRRi runs 2^i DALS searches simultaneously as bundled search. (Applies for IND, DIJ)")
target_compile_definitions(karri PRIVATE KARRI_DALS_LOG_K=${KARRI_DALS_LOG_K})

### Applies for IND, DIJ
option(KARRI_DALS_USE_SIMD "Use SIMD instructions for bundled DALS searches. (Applies for IND, DIJ)" OFF)
if (KARRI_DALS_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_DALS_USE_SIMD=true)
else(KARRI_DALS_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_DALS_USE_SIMD=false)
endif (KARRI_DALS_USE_SIMD)

### Applies for COL, IND
option(KARRI_LAST_STOP_BCH_SORTED_BUCKETS "Use sorted buckets for last stop BCH searches. (Applies for COL, IND)" ON)
if (KARRI_LAST_STOP_BCH_SORTED_BUCKETS)
    target_compile_definitions(karri PRIVATE KARRI_LAST_STOP_BCH_SORTED_BUCKETS=true)
else(KARRI_LAST_STOP_BCH_SORTED_BUCKETS)
    target_compile_definitions(karri PRIVATE KARRI_LAST_STOP_BCH_SORTED_BUCKETS=false)
endif (KARRI_LAST_STOP_BCH_SORTED_BUCKETS)

### Applies for COL
option(KARRI_COL_PALS_ONLY_PROMISING_DROPOFFS "Precompute promising dropoffs and use only those in MinCostPair search. (Applies for PALS Strategy = COL)" OFF)
if (KARRI_COL_PALS_ONLY_PROMISING_DROPOFFS)
    target_compile_definitions(karri PRIVATE KARRI_COL_PALS_ONLY_PROMISING_DROPOFFS=true)
else(KARRI_COL_PALS_ONLY_PROMISING_DROPOFFS)
    target_compile_definitions(karri PRIVATE KARRI_COL_PALS_ONLY_PROMISING_DROPOFFS=false)
endif (KARRI_COL_PALS_ONLY_PROMISING_DROPOFFS)

# End of KaRRi compile time parameters
