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
                             const RequestState &requestState,
                             stats::OrdAssignmentsPerformanceStats &ordStats,
                             stats::PbnsAssignmentsPerformanceStats &pbnsStats,
                             const FeasibleDistancesT &feasiblePickupDistances,
                             const FeasibleDistancesT &feasibleDropoffDistances,
                             RelevantPDLocs &relOrdinaryPickups, RelevantPDLocs &relOrdinaryDropoffs,
                             RelevantPDLocs &relPickupsBeforeNextStop, RelevantPDLocs &relDropoffsBeforeNextStop)
                : fleet(fleet),
                  inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  chQuery(chEnv.template getFullCHQuery<>()),
                  requestState(requestState),
                  ordStats(ordStats),
                  pbnsStats(pbnsStats),
                  feasiblePickupDistances(feasiblePickupDistances),
                  feasibleDropoffDistances(feasibleDropoffDistances),
                  relOrdinaryPickups(relOrdinaryPickups),
                  relOrdinaryDropoffs(relOrdinaryDropoffs),
                  relPickupsBeforeNextStop(relPickupsBeforeNextStop),
                  relDropoffsBeforeNextStop(relDropoffsBeforeNextStop) {}


        void filterOrdinary(const int & bestKnownCost, const RouteStateData& routeState) {
            Timer timer;

            const int numRelStopsForPickups = filterOrdinaryPickups(bestKnownCost, routeState);
            const int numRelStopsForDropoffs = filterOrdinaryDropoffs(bestKnownCost, routeState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            ordStats.filterRelevantPDLocsTime += time;
            ordStats.numRelevantStopsForPickups += numRelStopsForPickups;
            ordStats.numRelevantStopsForDropoffs += numRelStopsForDropoffs;
        }

        int filterOrdinaryPickups(const int & bestKnownCost, const RouteStateData& routeState) {
            return filter<false, false>(feasiblePickupDistances, relOrdinaryPickups, requestState.numPickups(), bestKnownCost, routeState);
        }

        int filterOrdinaryDropoffs(const int & bestKnownCost, const RouteStateData& routeState) {
            return filter<false, true>(feasibleDropoffDistances, relOrdinaryDropoffs, requestState.numDropoffs(), bestKnownCost, routeState);
        }

        void filterBeforeNextStop(const int & bestKnownCost, const RouteStateData& routeState) {
            Timer timer;

            const int numRelStopsForPickups = filterPickupsBeforeNextStop(bestKnownCost, routeState);
            const int numRelStopsForDropoffs = filterDropoffsBeforeNextStop(bestKnownCost, routeState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            pbnsStats.filterRelevantPDLocsTime += time;
            pbnsStats.numRelevantStopsForPickups += numRelStopsForPickups;
            pbnsStats.numRelevantStopsForDropoffs += numRelStopsForDropoffs;
        }

        int filterPickupsBeforeNextStop(const int & bestKnownCost, const RouteStateData& routeState) {
            return filter<true, false>(feasiblePickupDistances, relPickupsBeforeNextStop, requestState.numPickups(), bestKnownCost, routeState);
        }

        int filterDropoffsBeforeNextStop(const int & bestKnownCost, const RouteStateData& routeState) {
            return filter<true, true>(feasibleDropoffDistances, relDropoffsBeforeNextStop, requestState.numDropoffs(), bestKnownCost, routeState);
        }

    private:


        template<bool beforeNextStop, bool isDropoff>
        int filter(const FeasibleDistancesT &feasible, RelevantPDLocs &rel, const int numPDLocs, const int& bestKnownCost, const RouteStateData& routeState) {

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

                    if ((!isDropoff || beforeNextStop) &&
                        occupancies[i] + requestState.originalRequest.numRiders > veh.capacity)
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
                            minCost = getMinCostForDropoff(fleet[vehId], i, minDistToPDLoc, minDistFromPDLoc, routeState);
                        } else {
                            minCost = getMinCostForPickup(fleet[vehId], i, minDistToPDLoc, minDistFromPDLoc, routeState);
                        }

                        if (minCost <= bestKnownCost) {

                            ++numStopsRelevant;
                            // Check each PD loc
                            const auto &distsToPDLocs = feasible.distancesToRelevantPDLocsFor(stopId);
                            const auto &distsFromPDLocs = feasible.distancesFromRelevantPDLocsToNextStopOf(stopId);
                            for (unsigned int id = 0; id < numPDLocs; ++id) {
                                const auto &distToPDLoc = distsToPDLocs[id];
                                const auto &distFromPDLoc = distsFromPDLocs[id];

                                bool isRelevant;
                                if constexpr (isDropoff) {
                                    isRelevant = isDropoffRelevant(fleet[vehId], i, id, distToPDLoc, distFromPDLoc, bestKnownCost, routeState);
                                } else {
                                    isRelevant = isPickupRelevant(fleet[vehId], i, id, distToPDLoc, distFromPDLoc, bestKnownCost, routeState);
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
                                     const int distFromPickupToNextStop,
                                     const int& bestKnownCost,
                                     const RouteStateData& routeState) {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            KASSERT(routeState.occupanciesFor(vehId)[stopIndex] + requestState.originalRequest.numRiders <=
                   veh.capacity);
            if (distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY)
                return false;

            KASSERT(distFromStopToPickup + distFromPickupToNextStop >=
                   calcLengthOfLegStartingAt(stopIndex, vehId, routeState),
                   "distFromStopToPickup = " << distFromStopToPickup << " (should be: " << recomputeDistToPDLocDirectly(vehId, stopIndex, requestState.pickups[pickupId].loc, routeState) << ")" <<
                   ", distFromPickupToNextStop = " << distFromPickupToNextStop << " (should be: " << recomputeDistFromPDLocDirectly(vehId, stopIndex + 1, requestState.pickups[pickupId].loc, routeState) << ")" <<
                   ", leg length = " << calcLengthOfLegStartingAt(stopIndex, vehId, routeState) << " (should be: " << recomputeLegLengthDirectly(vehId, stopIndex, routeState) << ")");

            const auto &p = requestState.pickups[pickupId];

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, distFromStopToPickup, p,
                                                                  requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX, depTimeAtPickup,
                                                                     distFromPickupToNextStop, requestState,
                                                                     routeState);

            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return false;


            const int curKnownCost = Calc::calcMinKnownPickupSideCost(veh, stopIndex, initialPickupDetour,
                                                                           p.walkingDist, depTimeAtPickup,
                                                                           requestState, routeState);

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            if (curKnownCost > bestKnownCost)
                return false;

            return true;
        }

        inline bool isDropoffRelevant(const Vehicle &veh, const int stopIndex, const unsigned int dropoffId,
                                      const int distFromStopToDropoff,
                                      const int distFromDropoffToNextStop,
                                      const int& bestKnownCost,
                                      const RouteStateData& routeState) const {
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
                                                                      routeState);
            assert(initialDropoffDetour >= 0);
            if (doesDropoffDetourViolateHardConstraints(veh, requestState, stopIndex, initialDropoffDetour,
                                                        routeState))
                return false;

            const int curMinCost = Calc::calcMinKnownDropoffSideCost(veh, stopIndex, initialDropoffDetour,
                                                                          d.walkingDist, requestState, routeState);

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            if (curMinCost > bestKnownCost)
                return false;

            return true;
        }

        inline int getMinCostForPickup(const Vehicle &veh, const int stopIndex, const int minDistToPickup,
                                       const int minDistFromPickup,
                                       const RouteStateData& routeState) const {
            using namespace time_utils;
            const int minVehDepTimeAtPickup =
                    getVehDepTimeAtStopForRequest(veh.vehicleId, stopIndex, requestState, routeState)
                    + minDistToPickup;
            const int minDepTimeAtPickup = std::max(requestState.originalRequest.requestTime, minVehDepTimeAtPickup);
            int minInitialPickupDetour = calcInitialPickupDetour(veh.vehicleId, stopIndex, INVALID_INDEX,
                                                                 minDepTimeAtPickup, minDistFromPickup, requestState,
                                                                 routeState);
            minInitialPickupDetour = std::max(minInitialPickupDetour, 0);
            return Calc::calcMinKnownPickupSideCost(veh, stopIndex, minInitialPickupDetour, 0, minDepTimeAtPickup,
                                                         requestState, routeState);
        }

        inline int getMinCostForDropoff(const Vehicle &veh, const int stopIndex, const int minDistToDropoff,
                                        const int minDistFromDropoff,
                                        const RouteStateData& routeState) const {
            using namespace time_utils;
            int minInitialDropoffDetour = calcInitialDropoffDetour(veh.vehicleId, stopIndex, minDistToDropoff,
                                                                   minDistFromDropoff, false, routeState);
            minInitialDropoffDetour = std::max(minInitialDropoffDetour, 0);
            return Calc::calcMinKnownDropoffSideCost(veh, stopIndex, minInitialDropoffDetour, 0, requestState, routeState);
        }

        int recomputeDistToPDLocDirectly(const int vehId, const int stopIdxBefore, const int pdLocLocation,
                                         const RouteStateData& routeState) {
            auto src = ch.rank(inputGraph.edgeHead(routeState.stopLocationsFor(vehId)[stopIdxBefore]));
            auto tar = ch.rank(inputGraph.edgeTail(pdLocLocation));
            auto offset = inputGraph.travelTime(pdLocLocation);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }

        int recomputeDistFromPDLocDirectly(const int vehId, const int stopIdxAfter, const int pdLocLocation,
                                           const RouteStateData& routeState) {
            auto src = ch.rank(inputGraph.edgeHead(pdLocLocation));
            auto tar = ch.rank(inputGraph.edgeTail(routeState.stopLocationsFor(vehId)[stopIdxAfter]));
            auto offset = inputGraph.travelTime(routeState.stopLocationsFor(vehId)[stopIdxAfter]);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }

        int recomputeLegLengthDirectly(const int vehId, const int stopIdx,
                                           const RouteStateData& routeState) {
            auto src = ch.rank(inputGraph.edgeHead(routeState.stopLocationsFor(vehId)[stopIdx]));
            auto tar = ch.rank(inputGraph.edgeTail(routeState.stopLocationsFor(vehId)[stopIdx + 1]));
            auto offset = inputGraph.travelTime(routeState.stopLocationsFor(vehId)[stopIdx + 1]);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }


        const Fleet &fleet;
        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        const RequestState &requestState;
        stats::OrdAssignmentsPerformanceStats &ordStats;
        stats::PbnsAssignmentsPerformanceStats &pbnsStats;


        const FeasibleDistancesT &feasiblePickupDistances;
        const FeasibleDistancesT &feasibleDropoffDistances;

        RelevantPDLocs &relOrdinaryPickups;
        RelevantPDLocs &relOrdinaryDropoffs;
        RelevantPDLocs &relPickupsBeforeNextStop;
        RelevantPDLocs &relDropoffsBeforeNextStop;
    };
}