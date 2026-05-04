# ******************************************************************************
# MIT License
#
# Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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


# KaRRi compile time parameters

## Output vehicle paths?
option(KARRI_OUTPUT_VEHICLE_PATHS "Output vehicle paths." OFF)
if (KARRI_OUTPUT_VEHICLE_PATHS)
    target_compile_definitions(karri PRIVATE KARRI_OUTPUT_VEHICLE_PATHS)
endif (KARRI_OUTPUT_VEHICLE_PATHS)

## Cost function tuning
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


## Compile time parameters for transfers
option(KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION "Use Dijkstra searches to find transfer points (slow)." OFF)
if (KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION=true)
else(KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION=false)
endif (KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION)

set(KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K 3 CACHE STRING "Given value i, KaRRi runs 2^i ellipse reconstruction searches simultaneously as bundled search.")
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K=${KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K})

option(KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD "Use SIMD instructions for bundled ellipse reconstruction searches." ON)
if (KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD=true)
else(KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD=false)
endif (KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD)


set(KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K 3 CACHE STRING "Given value i, KaRRi runs 2^i direct transfer distance searches simultaneously as bundled search.")
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K=${KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K})

option(KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD "Use SIMD instructions for bundled direct transfer distance searches." ON)
if (KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD=true)
else(KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD=false)
endif (KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD)


### Strategy for calculating shortest paths in the transfer ALS case
set(TALS_CH_CODE 1)
set(TALS_PHAST_CODE 2)
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_CH=${TALS_CH_CODE})
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_PHAST=${TALS_PHAST_CODE})
set(KARRI_TRANSFER_TALS_STRAT_ARG_VALUES CH PHAST)

set(KARRI_TRANSFER_TALS_STRAT PHAST CACHE STRING "Strategy for computing shortest path in the transfer ALS case. Possible values: CH, PHAST")
set_property(CACHE KARRI_TRANSFER_TALS_STRAT PROPERTY STRINGS ${KARRI_TRANSFER_TALS_STRAT_ARG_VALUES})
list(FIND KARRI_TRANSFER_TALS_STRAT_ARG_VALUES ${KARRI_TRANSFER_TALS_STRAT} index)
if (index EQUAL -1)
    message(FATAL_ERROR "KARRI_TRANSFER_TALS_STRAT must be one of ${KARRI_TRANSFER_TALS_STRAT_ARG_VALUES}")
endif ()

if (${KARRI_TRANSFER_TALS_STRAT} STREQUAL CH)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_STRAT=${TALS_CH_CODE}) # Use CH
else()
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_STRAT=${TALS_PHAST_CODE}) # Use PHAST
endif()

#[[set(TALS_CH 1)
set(TALS_PHAST 2)

set(KARRI_TRANSFER_TALS_CH TALS_CH CACHE STRING "Transfer ALS using CH Approach.")
set(KARRI_TRANSFER_TALS_PHAST TALS_PHAST CACHE STRING "Transfer ALS using PHAST Approach.")

target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_CH=1)
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_PHAST=2)

## Set the strategy here...
set(KARRI_TRANSFER_TALS_STRAT KARRI_TRANSFER_TALS_PHAST CACHE STRING "Strategy for computing shortest path in the transfer ALS case.")
message("TALS strat: ${KARRI_TRANSFER_TALS_STRAT}")
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_STRAT=${KARRI_TRANSFER_TALS_STRAT})]]

set(KARRI_TRANSFER_TALS_LOG_K 3 CACHE STRING "Given value i, KaRRi runs 2^i TALS searches simultaneously as bundled search.")
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_LOG_K=${KARRI_TRANSFER_TALS_LOG_K})

option(KARRI_TRANSFER_TALS_USE_SIMD "Use SIMD instructions for bundled TALS searches." ON)
if (KARRI_TRANSFER_TALS_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_USE_SIMD=true)
else(KARRI_TRANSFER_TALS_USE_SIMD)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_TALS_USE_SIMD=false)
endif (KARRI_TRANSFER_TALS_USE_SIMD)

## Parallelize PHAST queries for computation of detour ellipses?
option(KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE "Parallelize PHAST queries for computation of detour ellipses. If set, use -max-num-threads." OFF)
if (KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE=true)
else(KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE=false)
endif (KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE)

### Heuristic levels for KaRRi with transfers. Level 0 means exact assignments, level 1 reduces set of vertices that
### are eligible as transfer points to high ranked vertices, and level 2 disables ordinary transfers entirely and
### limits ALS transfers to transfers at existing stops of one of the participating vehicles. Any value greater than 2
### has same effect as 2.
set(KARRI_TRANSFER_HEURISTIC_LEVEL 0 CACHE STRING "Heuristic levels for KaRRi with transfers. Level 0 means exact assignments.")
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_HEURISTIC_LEVEL=${KARRI_TRANSFER_HEURISTIC_LEVEL})

### Set severity of heuristic level 1. Expect larger values to decrease running time but also quality.
### Only has an effect if KARRI_TRANSFER_HEURISTIC_LEVEL == 1.
set(KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_TOP_VERTICES_DIVISOR 10 CACHE STRING "Only top n / KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_TOP_VERTICES_DIVISOR vertices are eligible for transfer points.")
target_compile_definitions(karri PRIVATE KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_TOP_VERTICES_DIVISOR=${KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_TOP_VERTICES_DIVISOR})


option(KARRI_TRANSFER_USE_COST_LOWER_BOUNDS "Enable use of cost lower bounds to reduce number of assignments checked." ON)
if (KARRI_TRANSFER_USE_COST_LOWER_BOUNDS)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_USE_COST_LOWER_BOUNDS=true)
else(KARRI_TRANSFER_USE_COST_LOWER_BOUNDS)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_USE_COST_LOWER_BOUNDS=false)
endif (KARRI_TRANSFER_USE_COST_LOWER_BOUNDS)

option(KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS "Enable Pareto checks for transfer points." ON)
if (KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS=true)
else(KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS)
    target_compile_definitions(karri PRIVATE KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS=false)
endif (KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS)