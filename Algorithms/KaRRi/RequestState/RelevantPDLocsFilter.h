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
    class RelevantPDLocsFilter {

    public:

        RelevantPDLocsFilter(const Fleet &fleet, const InputGraphT &inputGraph, const CHEnvT &chEnv,
                             const CostCalculator &calculator,
                             RequestState &requestState, const RouteState &routeState,
                             const InputConfig &inputConfig, const FeasibleDistancesT &feasiblePickupDistances,
                             const FeasibleDistancesT &feasibleDropoffDistances,
                             RelevantPDLocs &relOrdinaryPickups, RelevantPDLocs &relOrdinaryDropoffs,
                             RelevantPDLocs &relPickupsBeforeNextStop, RelevantPDLocs &relDropoffsBeforeNextStop)
                : fleet(fleet),
                  inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  chQuery(chEnv.template getFullCHQuery<>()),
                  calculator(calculator),
                  requestState(requestState),
                  routeState(routeState),
                  inputConfig(inputConfig),
                  feasiblePickupDistances(feasiblePickupDistances),
                  feasibleDropoffDistances(feasibleDropoffDistances),
                  relOrdinaryPickups(relOrdinaryPickups),
                  relOrdinaryDropoffs(relOrdinaryDropoffs),
                  relPickupsBeforeNextStop(relPickupsBeforeNextStop),
                  relDropoffsBeforeNextStop(relDropoffsBeforeNextStop) {}


        void filterOrdinary() {
            Timer timer;

            const int numRelStopsForPickups = filterOrdinaryPickups();
            const int numRelStopsForDropoffs = filterOrdinaryDropoffs();

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().ordAssignmentsStats.numRelevantStopsForPickups += numRelStopsForPickups;
            requestState.stats().ordAssignmentsStats.numRelevantStopsForDropoffs += numRelStopsForDropoffs;
        }

        int filterOrdinaryPickups() {
            return filter<false, false>(feasiblePickupDistances, relOrdinaryPickups, requestState.numPickups());
        }

        int filterOrdinaryDropoffs() {
            return filter<false, true>(feasibleDropoffDistances, relOrdinaryDropoffs, requestState.numDropoffs());
        }

        void filterBeforeNextStop() {
            Timer timer;

            const int numRelStopsForPickups = filterPickupsBeforeNextStop();
            const int numRelStopsForDropoffs = filterDropoffsBeforeNextStop();

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().pbnsAssignmentsStats.numRelevantStopsForPickups += numRelStopsForPickups;
            requestState.stats().pbnsAssignmentsStats.numRelevantStopsForDropoffs += numRelStopsForDropoffs;
        }

        int filterPickupsBeforeNextStop() {
            return filter<true, false>(feasiblePickupDistances, relPickupsBeforeNextStop, requestState.numPickups());
        }

        int filterDropoffsBeforeNextStop() {
            return filter<true, true>(feasibleDropoffDistances, relDropoffsBeforeNextStop, requestState.numDropoffs());
        }

    private:


        template<bool beforeNextStop, bool isDropoff>
        int filter(const FeasibleDistancesT &feasible, RelevantPDLocs &rel, const int numPDLocs) {

            // For each stop s, prune the pickups and dropoffs deemed relevant for an ordinary assignment after s by
            // checking them against constraints and lower bounds.
            using namespace time_utils;

            int numStopsRelevant = 0;

            rel.relevantSpots.clear();
            rel.vehiclesWithRelevantSpots.clear();


            const auto &vehiclesWithFeasibleDistances = feasible.getVehiclesWithRelevantPDLocs();

            for (int vehId = 0; vehId < fleet.size(); ++vehId) {
                const auto &veh = fleet[vehId];
                rel.startOfRelevantPDLocs[vehId] = rel.relevantSpots.size();
                rel.startOfRelevantPDLocs[vehId] = rel.relevantSpots.size();

                if (!vehiclesWithFeasibleDistances.contains(vehId))
                    continue;

                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &stopIds = routeState.stopIdsFor(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);

                if (numStops <= 1)
                    continue;

                // Track relevant PD locs for each stop in the relevant PD locs data structure.
                // Entries are ordered by vehicle and by stop.
                const int beginStopIdx = beforeNextStop ? 0 : 1;
                const int endStopIdx = beforeNextStop ? 1 : (isDropoff ? numStops : numStops - 1);
                for (int i = beginStopIdx; i < endStopIdx; ++i) {

                    if ((!isDropoff || beforeNextStop) && occupancies[i] + requestState.originalRequest.numRiders > veh.capacity)
                        continue;

                    const auto &stopId = stopIds[i];

                    // Insert entries at this stop
                    if (feasible.hasPotentiallyRelevantPDLocs(stopId)) {
                        assert(vehiclesWithFeasibleDistances.contains(vehId));

                        // Check with lower bounds on dist to and from PD loc whether this stop needs to be regarded
                        const int minDistToPDLoc = feasible.minDistToRelevantPDLocsFor(stopId);
                        const int minDistFromPDLoc = feasible.minDistFromPDLocToNextStopOf(stopId);

                        // Compute lower bound cost based on whether we are dealing with pickups or dropoffs
                        int minCost;
                        if constexpr (isDropoff) {
                            minCost = getMinCostForDropoff(fleet[vehId], i, minDistToPDLoc, minDistFromPDLoc);
                        } else {
                            minCost = getMinCostForPickup(fleet[vehId], i, minDistToPDLoc, minDistFromPDLoc);
                        }

                        if (minCost <= requestState.getBestCost()) {

                            ++numStopsRelevant;
                            // Check each PD loc
                            const auto &distsToPDLocs = feasible.distancesToRelevantPDLocsFor(stopId);
                            const auto &distsFromPDLocs = feasible.distancesFromRelevantPDLocsToNextStopOf(stopId);
                            for (unsigned int id = 0; id < numPDLocs; ++id) {
                                const auto &distToPDLoc = distsToPDLocs[id];
                                const auto &distFromPDLoc = distsFromPDLocs[id];

                                bool isRelevant;
                                if constexpr (isDropoff) {
                                    isRelevant = isDropoffRelevant(fleet[vehId], i, id, distToPDLoc, distFromPDLoc);
                                } else {
                                    isRelevant = isPickupRelevant(fleet[vehId], i, id, distToPDLoc, distFromPDLoc);
                                }

                                if (isRelevant) {
                                    rel.relevantSpots.push_back({i, id, distToPDLoc, distFromPDLoc});
                                }
                            }
                        }
                    }
                }

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (rel.relevantSpots.size() > rel.startOfRelevantPDLocs[vehId])
                    rel.vehiclesWithRelevantSpots.insert(vehId);
            }

            rel.startOfRelevantPDLocs[fleet.size()] = rel.relevantSpots.size();

            assert(std::all_of(rel.relevantSpots.begin(), rel.relevantSpots.end(),
                               [&](const auto &h) {
                                   return h.distToPDLoc < INFTY && h.distFromPDLocToNextStop < INFTY;
                               }));

            return numStopsRelevant;
        }

        inline bool isPickupRelevant(const Vehicle &veh, const int stopIndex, const unsigned int pickupId,
                                     const int distFromStopToPickup,
                                     const int distFromPickupToNextStop) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            assert(routeState.occupanciesFor(vehId)[stopIndex] + requestState.originalRequest.numRiders <= veh.capacity);
            if (distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY)
                return false;

            assert(distFromStopToPickup + distFromPickupToNextStop >=
                   calcLengthOfLegStartingAt(stopIndex, vehId, routeState));

            const auto &p = requestState.pickups[pickupId];

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, distFromStopToPickup, p,
                                                                  requestState, routeState, inputConfig);
            const auto initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX, depTimeAtPickup,
                                                                     distFromPickupToNextStop, requestState,
                                                                     routeState);

            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return false;


            const int curKnownCost = calculator.calcMinKnownPickupSideCost(veh, stopIndex, initialPickupDetour,
                                                                           p.walkingDist, depTimeAtPickup,
                                                                           requestState);

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            if (curKnownCost > requestState.getBestCost())
                return false;

            return true;
        }

        inline bool isDropoffRelevant(const Vehicle &veh, const int stopIndex, const unsigned int dropoffId,
                                      const int distFromStopToDropoff,
                                      const int distFromDropoffToNextStop) {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;
            const auto &d = requestState.dropoffs[dropoffId];

            // If this is the last stop in the route, we only consider this dropoff for ordinary assignments if it is at the
            // last stop. Similarly, if the vehicle is full after this stop, we can't perform the dropoff here unless the
            // dropoff coincides with the stop. A dropoff at an existing stop causes no detour, so it is always relevant.
            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &occupancy = routeState.occupanciesFor(vehId)[stopIndex];
            const auto &stopLocations = routeState.stopLocationsFor(vehId);
            assert(d.loc != stopLocations[stopIndex] || distFromStopToDropoff == 0);
            if (stopIndex == numStops - 1 || occupancy + requestState.originalRequest.numRiders > veh.capacity)
                return d.loc == stopLocations[stopIndex];

            if (stopLocations[stopIndex + 1] == d.loc)
                return false;

            if (distFromStopToDropoff >= INFTY || distFromDropoffToNextStop >= INFTY)
                return false;

            const bool isDropoffAtExistingStop = d.loc == stopLocations[stopIndex];
            const int initialDropoffDetour = calcInitialDropoffDetour(vehId, stopIndex, distFromStopToDropoff,
                                                                      distFromDropoffToNextStop,
                                                                      isDropoffAtExistingStop,
                                                                      routeState, inputConfig);
            assert(initialDropoffDetour >= 0);
            if (doesDropoffDetourViolateHardConstraints(veh, requestState, stopIndex, initialDropoffDetour,
                                                        routeState))
                return false;

            const int curMinCost = calculator.calcMinKnownDropoffSideCost(veh, stopIndex, initialDropoffDetour,
                                                                          d.walkingDist, requestState);

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            if (curMinCost > requestState.getBestCost())
                return false;

            return true;
        }

        inline int getMinCostForPickup(const Vehicle &veh, const int stopIndex, const int minDistToPickup,
                                       const int minDistFromPickup) const {
            using namespace time_utils;
            const int minVehDepTimeAtPickup =
                    getVehDepTimeAtStopForRequest(veh.vehicleId, stopIndex, requestState, routeState)
                    + minDistToPickup;
            const int minDepTimeAtPickup = std::max(requestState.originalRequest.requestTime, minVehDepTimeAtPickup);
            int minInitialPickupDetour = calcInitialPickupDetour(veh.vehicleId, stopIndex, INVALID_INDEX,
                                                                 minDepTimeAtPickup, minDistFromPickup, requestState,
                                                                 routeState);
            minInitialPickupDetour = std::max(minInitialPickupDetour, 0);
            return calculator.calcMinKnownPickupSideCost(veh, stopIndex, minInitialPickupDetour, 0, minDepTimeAtPickup,
                                                         requestState);
        }

        inline int getMinCostForDropoff(const Vehicle &veh, const int stopIndex, const int minDistToDropoff,
                                        const int minDistFromDropoff) const {
            using namespace time_utils;
            int minInitialDropoffDetour = calcInitialDropoffDetour(veh.vehicleId, stopIndex, minDistToDropoff,
                                                                   minDistFromDropoff, false, routeState, inputConfig);
            minInitialDropoffDetour = std::max(minInitialDropoffDetour, 0);
            return calculator.calcMinKnownDropoffSideCost(veh, stopIndex, minInitialDropoffDetour, 0, requestState);
        }

        int recomputeDistToPDLocDirectly(const int vehId, const int stopIdxBefore, const int pdLocLocation) {
            auto src = ch.rank(inputGraph.edgeHead(routeState.stopLocationsFor(vehId)[stopIdxBefore]));
            auto tar = ch.rank(inputGraph.edgeTail(pdLocLocation));
            auto offset = inputGraph.travelTime(pdLocLocation);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }

        int recomputeDistFromPDLocDirectly(const int vehId, const int stopIdxAfter, const int pdLocLocation) {
            auto src = ch.rank(inputGraph.edgeHead(pdLocLocation));
            auto tar = ch.rank(inputGraph.edgeTail(routeState.stopLocationsFor(vehId)[stopIdxAfter]));
            auto offset = inputGraph.travelTime(routeState.stopLocationsFor(vehId)[stopIdxAfter]);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }


        const Fleet &fleet;
        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        const CostCalculator &calculator;
        RequestState &requestState;
        const RouteState &routeState;
        const InputConfig &inputConfig;

        const FeasibleDistancesT &feasiblePickupDistances;
        const FeasibleDistancesT &feasibleDropoffDistances;

        RelevantPDLocs &relOrdinaryPickups;
        RelevantPDLocs &relOrdinaryDropoffs;
        RelevantPDLocs &relPickupsBeforeNextStop;
        RelevantPDLocs &relDropoffsBeforeNextStop;
    };
}