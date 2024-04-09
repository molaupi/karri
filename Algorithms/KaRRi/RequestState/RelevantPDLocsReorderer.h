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
                  relDropoffsBeforeNextStop(relDropoffsBeforeNextStop),
                  nextIdxForStop(0, INVALID_INDEX) {}


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
//                nextIdxForStop.resize(routeState.getMaxStopId() + 1, CAtomic<int>(INVALID_INDEX));
                nextIdxForStop.resize(routeState.getMaxStopId() + 1);
//            for (auto &a: nextIdxForStop)
//                a.store(0);
            nextIdxForStop.clear();

            for (const auto& vehId : relOrdinary.vehiclesWithRelevantSpots)
                relOrdinary.rangeOfRelevantPdLocs[vehId] = {0, 0};
            relOrdinary.vehiclesWithRelevantSpots.clear();

            for (const auto& vehId : relBns.vehiclesWithRelevantSpots)
                relBns.rangeOfRelevantPdLocs[vehId] = {0, 0};
            relBns.vehiclesWithRelevantSpots.clear();

            int sumOrdinary = 0;
            int sumBns = 0;

            for (const int& vehId : feasible.getVehiclesWithFeasibleDistances()) {
                const int &numStops = routeState.numStopsOf(vehId);
                assert(numStops > 1);
                relOrdinary.rangeOfRelevantPdLocs[vehId].start = sumOrdinary;
                relBns.rangeOfRelevantPdLocs[vehId].start = sumBns;

                const auto &stopIds = routeState.stopIdsFor(vehId);
                nextIdxForStop[stopIds[0]] = sumBns;
                const int numRelPdLocsForVehBns = feasible.getNumRelPdLocsForStop(stopIds[0]);
                numRelBnsStops += (numRelPdLocsForVehBns > 0);

                int numRelPdLocsForVehOrdinary = 0;
                for (int stopPos = 1; stopPos < numStops; ++stopPos) {
                    const auto &stopId = stopIds[stopPos];
                    nextIdxForStop[stopId] = sumOrdinary + numRelPdLocsForVehOrdinary;
                    const auto &numRelForStop = feasible.getNumRelPdLocsForStop(stopId);
                    numRelPdLocsForVehOrdinary += numRelForStop;

                    numRelOrdinaryStops += (numRelForStop > 0);
                }

                sumOrdinary += numRelPdLocsForVehOrdinary;
                sumBns += numRelPdLocsForVehBns;

                relOrdinary.rangeOfRelevantPdLocs[vehId].end = sumOrdinary;
                relBns.rangeOfRelevantPdLocs[vehId].end = sumBns;

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (numRelPdLocsForVehOrdinary > 0)
                    relOrdinary.vehiclesWithRelevantSpots.insert(vehId);
                if (numRelPdLocsForVehBns > 0)
                    relBns.vehiclesWithRelevantSpots.insert(vehId);
            }
//            for (int vehId = 0; vehId < fleet.size(); ++vehId) {
//                const int &numStops = routeState.numStopsOf(vehId);
//                relOrdinary.startOfRelevantPDLocs[vehId] = sumOrdinary;
//                relBns.startOfRelevantPDLocs[vehId] = sumBns;
//
//                if (numStops == 1)
//                    continue;
//
//                const auto &stopIds = routeState.stopIdsFor(vehId);
//                nextIdxForStop[stopIds[0]] = relBns.startOfRelevantPDLocs[vehId];
//                const int numRelPdLocsForVehBns = feasible.getNumRelPdLocsForStop(stopIds[0]);
//                numRelBnsStops += (numRelPdLocsForVehBns > 0);
//
//                int numRelPdLocsForVehOrdinary = 0;
//                for (int stopPos = 1; stopPos < numStops; ++stopPos) {
//                    const auto &stopId = stopIds[stopPos];
//                    nextIdxForStop[stopId] = relOrdinary.startOfRelevantPDLocs[vehId] + numRelPdLocsForVehOrdinary;
//                    const auto &numRelForStop = feasible.getNumRelPdLocsForStop(stopId);
//                    numRelPdLocsForVehOrdinary += numRelForStop;
//
//                    numRelOrdinaryStops += (numRelForStop > 0);
//                }
//
//                sumOrdinary += numRelPdLocsForVehOrdinary;
//                sumBns += numRelPdLocsForVehBns;
//
//                // If vehicle has at least one stop with relevant PD loc, add the vehicle
//                if (numRelPdLocsForVehOrdinary > 0)
//                    relOrdinary.vehiclesWithRelevantSpots.insert(vehId);
//                if (numRelPdLocsForVehBns > 0)
//                    relBns.vehiclesWithRelevantSpots.insert(vehId);
//            }

            relOrdinary.relevantSpots.resize(sumOrdinary);
            relBns.relevantSpots.resize(sumBns);


            // Iterate through unordered feasible elliptic distances in parallel.
            // For every RelevantPDLoc, write it into RelevantPDLocs structure at right index
            // (offsets computed in prefix sum earlier).
            const auto &unordered = feasible.getGlobalResults();

            for (const auto &e: unordered) {
                const int stopPos = routeState.stopPositionOf(e.stopId);
                const int idxInRel = nextIdxForStop[e.stopId]++;
                if (stopPos == 0) {
                    relBns.relevantSpots[idxInRel] = e;
                } else {
                    relOrdinary.relevantSpots[idxInRel] = e;
                }
            }

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

//        std::vector<CAtomic<int>> nextIdxForStop;
        TimestampedVector<int> nextIdxForStop;
    };
}