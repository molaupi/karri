/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"

namespace karri {

// Filters information about feasible distances found by elliptic BCH searches to pickups/dropoffs that are relevant
// for certain stops by considering the leeway and the current best known assignment cost.
    template<typename FeasibleDistancesT, typename InputGraphT, typename CHEnvT>
    class RelevantPDLocsReorderer {

    public:

        RelevantPDLocsReorderer(const Fleet &fleet,
                                RequestState &requestState,
                                const RouteState &routeState,
                                const FeasibleDistancesT &feasiblePickupDistances,
                                const FeasibleDistancesT &feasibleDropoffDistances,
                                RelevantPDLocs &relOrdinaryPickups, RelevantPDLocs &relOrdinaryDropoffs,
                                RelevantPDLocs &relPickupsBeforeNextStop, RelevantPDLocs &relDropoffsBeforeNextStop)
                : fleet(fleet),
                  requestState(requestState),
                  routeState(routeState),
                  feasiblePickupDistances(feasiblePickupDistances),
                  feasibleDropoffDistances(feasibleDropoffDistances),
                  relOrdinaryPickups(relOrdinaryPickups),
                  relOrdinaryDropoffs(relOrdinaryDropoffs),
                  relPickupsBeforeNextStop(relPickupsBeforeNextStop),
                  relDropoffsBeforeNextStop(relDropoffsBeforeNextStop) {}


        void reorderAll() {
            reorderPickups();
            reorderDropoffs();
        }

        void reorderPickups() {
            int numRelOrdinaryStops = 0;
            int numRelBnsStops = 0;
            Timer timer;

            reorder(feasiblePickupDistances, relOrdinaryPickups, relPickupsBeforeNextStop, numRelOrdinaryStops,
                    numRelBnsStops);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().ordAssignmentsStats.numRelevantStopsForPickups += numRelOrdinaryStops;
            requestState.stats().pbnsAssignmentsStats.numRelevantStopsForPickups += numRelBnsStops;
        }

        void reorderDropoffs() {
            int numRelOrdinaryStops = 0;
            int numRelBnsStops = 0;
            Timer timer;

            reorder(feasibleDropoffDistances, relOrdinaryDropoffs, relDropoffsBeforeNextStop, numRelOrdinaryStops,
                    numRelBnsStops);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().ordAssignmentsStats.numRelevantStopsForDropoffs += numRelOrdinaryStops;
            requestState.stats().pbnsAssignmentsStats.numRelevantStopsForDropoffs += numRelBnsStops;
        }

    private:

        void reorder(const FeasibleDistancesT &feasible, RelevantPDLocs &relOrdinary, RelevantPDLocs &relBns,
                     int &numRelOrdinaryStops, int &numRelBnsStops) {
            // Compute prefix sum over number of relevant PDLocs per vehicle to find starts of ranges in RelevantPDLocs
            // data structure. Additionally compute auxiliary idx counters for every stop used when populating the
            // RelevantPDLocs data structure in parallel later.
            if (routeState.getMaxStopId() >= nextIdxForStop.size())
                nextIdxForStop.resize(routeState.getMaxStopId() + 1, CAtomic<int>(INVALID_INDEX));
            for (auto &a: nextIdxForStop)
                a.store(0);

            int sumOrdinary = 0;
            int sumBns = 0;
            for (int vehId = 0; vehId < fleet.size(); ++vehId) {
                const int &numStops = routeState.numStopsOf(vehId);
                relOrdinary.startOfRelevantPDLocs[vehId] = sumOrdinary;
                relBns.startOfRelevantPDLocs[vehId] = sumBns;

                if (numStops == 1)
                    continue;

                const auto &stopIds = routeState.stopIdsFor(vehId);
                nextIdxForStop[stopIds[0]].store(relBns.startOfRelevantPDLocs[vehId], std::memory_order_seq_cst);
                const int numRelPdLocsForVehBns = feasible.getNumRelPdLocsForStop(stopIds[0]);
                numRelBnsStops += (numRelPdLocsForVehBns > 0);

                int numRelPdLocsForVehOrdinary = 0;
                for (int stopPos = 1; stopPos < numStops; ++stopPos) {
                    const auto &stopId = stopIds[stopPos];
                    nextIdxForStop[stopId].store(relOrdinary.startOfRelevantPDLocs[vehId] + numRelPdLocsForVehOrdinary,
                                                 std::memory_order_seq_cst);
                    const auto &numRelForStop = feasible.getNumRelPdLocsForStop(stopId);
                    numRelPdLocsForVehOrdinary += numRelForStop;

                    numRelOrdinaryStops += (numRelForStop > 0);
                }

                sumOrdinary += numRelPdLocsForVehOrdinary;
                sumBns += numRelPdLocsForVehBns;

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (numRelPdLocsForVehOrdinary > 0)
                    relOrdinary.vehiclesWithRelevantSpots.insert(vehId);
                if (numRelPdLocsForVehBns > 0)
                    relBns.vehiclesWithRelevantSpots.insert(vehId);
            }
            assert(std::all_of(nextIdxForStop.begin(), nextIdxForStop.end(),
                               [&](const auto &a) { return a.load() != INVALID_INDEX; }));

            relOrdinary.relevantSpots.resize(sumOrdinary);
            relBns.relevantSpots.resize(sumBns);


            // Iterate through unordered feasible elliptic distances in parallel.
            // For every RelevantPDLoc, write it into RelevantPDLocs structure at right index
            // (offsets computed in prefix sum earlier).
            const auto &unordered = feasible.getGlobalResults();
            parallel_for(int(0), static_cast<int>(unordered.size()), [&](int i) {
                const auto &e = unordered[i];
                const int stopPos = routeState.stopPositionOf(e.stopId);
                const int idxInRel = nextIdxForStop[e.stopId].fetch_add(1, std::memory_order_relaxed);
                if (stopPos == 0) {
                    relBns.relevantSpots[idxInRel] = e;
                } else {
                    relOrdinary.relevantSpots[idxInRel] = e;
                }
            });
        }

        const Fleet &fleet;
        RequestState &requestState;
        const RouteState &routeState;

        const FeasibleDistancesT &feasiblePickupDistances;
        const FeasibleDistancesT &feasibleDropoffDistances;

        RelevantPDLocs &relOrdinaryPickups;
        RelevantPDLocs &relOrdinaryDropoffs;
        RelevantPDLocs &relPickupsBeforeNextStop;
        RelevantPDLocs &relDropoffsBeforeNextStop;

        std::vector<CAtomic<int>> nextIdxForStop;
    };
}