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
#include "DataStructures/Containers/FastResetFlagArray.h"
#include "DataStructures/Containers/LightweightSubset.h"

namespace karri {

// Filters information about feasible distances found by elliptic BCH searches to pickups/dropoffs that are relevant
// for certain stops by considering the leeway and the current best known assignment cost.
    template<typename FeasibleDistancesT, typename InputGraphT, typename CHEnvT>
    class RelevantPDLocsFilter {

    public:

        RelevantPDLocsFilter(const Fleet &fleet, const InputGraphT &inputGraph, const CHEnvT &chEnv,
                             const RouteState &routeState)
                : fleet(fleet),
                  inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  chQuery(chEnv.template getFullCHQuery<>()),
                  calculator(routeState),
                  routeState(routeState),
                  vehiclesWithFeasibleDistances(fleet.size()) {}

        RelevantPDLocs
        filterOrdinaryPickups(const FeasibleDistancesT &feasiblePickupDistances, RequestState &requestState) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filter<false, false>(feasiblePickupDistances, requestState.numPickups(), numRelStops,
                                                  requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().ordAssignmentsStats.numRelevantStopsForPickups += numRelStops;
            requestState.stats().ordAssignmentsStats.numRelevantPickupEntries += rel.numRelevantSpots();

            return rel;
        }


        RelevantPDLocs
        filterOrdinaryDropoffs(const FeasibleDistancesT &feasibleDropoffDistances, RequestState &requestState) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filter<false, true>(feasibleDropoffDistances, requestState.numDropoffs(), numRelStops,
                                                 requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().ordAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().ordAssignmentsStats.numRelevantStopsForDropoffs += numRelStops;
            requestState.stats().ordAssignmentsStats.numRelevantDropoffEntries += rel.numRelevantSpots();

            return rel;
        }

        RelevantPDLocs
        filterPickupsBeforeNextStop(const FeasibleDistancesT &feasiblePickupDistances, RequestState &requestState) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filter<true, false>(feasiblePickupDistances, requestState.numPickups(), numRelStops,
                                                 requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().pbnsAssignmentsStats.numRelevantStopsForPickups += numRelStops;
            requestState.stats().pbnsAssignmentsStats.numRelevantPickupEntries += rel.numRelevantSpots();
            return rel;
        }

        RelevantPDLocs filterDropoffsBeforeNextStop(const FeasibleDistancesT &feasibleDropoffDistances,
                                                    RequestState &requestState) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filter<true, true>(feasibleDropoffDistances, requestState.numDropoffs(), numRelStops,
                                                requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pbnsAssignmentsStats.filterRelevantPDLocsTime += time;
            requestState.stats().pbnsAssignmentsStats.numRelevantStopsForDropoffs += numRelStops;
            requestState.stats().pbnsAssignmentsStats.numRelevantDropoffEntries += rel.numRelevantSpots();

            return rel;
        }

    private:


        template<bool beforeNextStop, bool isDropoff>
        RelevantPDLocs filter(const FeasibleDistancesT &feasible, const int numPDLocs, int &numStopsRelevant,
                              const RequestState &requestState) {

            // For each stop s, prune the pickups and dropoffs deemed relevant for an ordinary assignment after s by
            // checking them against constraints and lower bounds.
            using namespace time_utils;

            numStopsRelevant = 0;

            RelevantPDLocs rel(fleet.size());

            vehiclesWithFeasibleDistances.clear();
            for (const auto &stopId: feasible.getStopIdsWithRelevantPDLocs()) {
                const auto vehId = routeState.vehicleIdOf(stopId);
                if (vehiclesWithFeasibleDistances.contains(vehId))
                    continue;
                const auto stopPos = routeState.stopPositionOf(stopId);
                if ((!beforeNextStop && stopPos == 0) || (beforeNextStop && stopPos > 0) ||
                    (!isDropoff && stopPos == routeState.numStopsOf(vehId) - 1))
                    continue;
                vehiclesWithFeasibleDistances.insert(vehId);
            }

            for (const auto &vehId: vehiclesWithFeasibleDistances) {
                const auto &veh = fleet[vehId];
                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &stopIds = routeState.stopIdsFor(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                KASSERT(numStops > 1);

                const int totalNumRelPdLocsBefore = static_cast<int>(rel.relevantSpots.size());



                // Track relevant PD locs for each stop in the relevant PD locs data structure.
                // Entries are ordered by vehicle and by stop.
                constexpr int beginStopIdx = beforeNextStop ? 0 : 1;
                const int endStopIdx = beforeNextStop ? 1 : (isDropoff ? numStops : numStops - 1);
                for (int i = beginStopIdx; i < endStopIdx; ++i) {

                    if ((!isDropoff || beforeNextStop) &&
                        occupancies[i] + requestState.originalRequest.numRiders > veh.capacity)
                        continue;

                    const auto &stopId = stopIds[i];
                    const auto beginEntriesForStop = rel.relevantSpots.size();

                    // If we consider only the stop before the next stop, the stop is guaranteed to have relevant pd
                    // locs by construction of vehiclesWithFeasibleDistances (s.a.).
                    // If we consider the stops at and after the next stop, there may be stops without relevant PD
                    // locs. Skip them.
                    KASSERT(!beforeNextStop || feasible.hasPotentiallyRelevantPDLocs(stopId));
                    if constexpr (!beforeNextStop)
                        if (!feasible.hasPotentiallyRelevantPDLocs(stopId))
                            continue;

                    // Insert entries at this stop.

                    // If there is more than one PD loc, check with lower bounds on dist to and from PD locs whether
                    // this stop needs to be regarded before looking at every PD loc.
                    if (numPDLocs > 1) {
                        const int minDistToPDLoc = feasible.minDistToRelevantPDLocsFor(stopId);
                        const int minDistFromPDLoc = feasible.minDistFromPDLocToNextStopOf(stopId);

                        // Compute lower bound cost based on whether we are dealing with pickups or dropoffs
                        int minCost;
                        if constexpr (isDropoff) {
                            minCost = getMinCostForDropoff(veh, i, minDistToPDLoc, minDistFromPDLoc,
                                                           requestState);
                        } else {
                            minCost = getMinCostForPickup(veh, i, minDistToPDLoc, minDistFromPDLoc,
                                                          requestState);
                        }

                        if (minCost > requestState.getBestCost())
                            continue;
                    }


                    ++numStopsRelevant;
                    // Check each PD loc
                    const auto &distsToPDLocs = feasible.distancesToRelevantPDLocsFor(stopId);
                    const auto &distsFromPDLocs = feasible.distancesFromRelevantPDLocsToNextStopOf(stopId);
                    for (unsigned int id = 0; id < numPDLocs; ++id) {
                        const auto &distToPDLoc = distsToPDLocs[id];
                        const auto &distFromPDLoc = distsFromPDLocs[id];
                        bool isRelevant;
                        if constexpr (isDropoff) {
                            isRelevant = isDropoffRelevant(veh, i, id, distToPDLoc, distFromPDLoc,
                                                           requestState);
                        } else {
                            isRelevant = isPickupRelevant(veh, i, id, distToPDLoc, distFromPDLoc,
                                                          requestState);
                        }

                        if (isRelevant) {
                            rel.relevantSpots.push_back({i, id, distToPDLoc, distFromPDLoc});
                        }
                    }


                    // For ordinary pickups, the unique best pickup for a pair of stops is the one with the smallest
                    // cost induced by the pickup detour since the dropoff is entirely independent of the pickup.
                    // Remove all entries except the unique best one.
                    if constexpr (!beforeNextStop) {
                        if constexpr (isDropoff)
                            removeAllDominatedOrdinaryDropoffs(rel, vehId, i, beginEntriesForStop, requestState);
                        else
                            removeAllDominatedOrdinaryPickups(rel, vehId, i, beginEntriesForStop, requestState);
                    }


                }

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (rel.relevantSpots.size() > totalNumRelPdLocsBefore) {
                    rel.vehiclesWithRelevantSpots.push_back(vehId);
                    rel.vehicleToPdLocs[vehId] = {totalNumRelPdLocsBefore, static_cast<int>(rel.relevantSpots.size())};
                }
            }

            KASSERT(std::all_of(rel.relevantSpots.begin(), rel.relevantSpots.end(),
                                [&](const auto &h) {
                                    return h.distToPDLoc < INFTY && h.distFromPDLocToNextStop < INFTY;
                                }));

            return rel;
        }

        void removeAllDominatedOrdinaryPickups(RelevantPDLocs &rel, const int vehId, const int stopPos,
                                               const size_t firstEntryForStop, const RequestState &requestState) {
            using namespace time_utils;
            if (rel.relevantSpots.size() <= firstEntryForStop)
                return;

            int minCost = INFTY;
            size_t bestIdx = INVALID_INDEX;

            for (auto j = firstEntryForStop; j < rel.relevantSpots.size(); ++j) {
                const auto &h = rel.relevantSpots[j];

                const auto &p = requestState.pickups[h.pdId];
                const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopPos, h.distToPDLoc,
                                                                      p, requestState, routeState);
                const int detour = calcInitialPickupDetour(vehId, stopPos, INVALID_INDEX, depTimeAtPickup,
                                                           h.distFromPDLocToNextStop, requestState, routeState);
                const int cost = calculator.calcMinKnownPickupSideCost(fleet[vehId], stopPos, detour,
                                                                       p.walkingDist, depTimeAtPickup,
                                                                       h.distFromPDLocToNextStop, requestState);

                const auto breakCostTie = [&](const auto &e1, const auto &e2) -> bool {
                    const auto &pd1 = requestState.pickups[e1.pdId];
                    const auto &pd2 = requestState.pickups[e2.pdId];
                    if (pd1.walkingDist < pd2.walkingDist) return true;
                    if (pd1.walkingDist > pd2.walkingDist) return false;
                    if (pd1.id < pd2.id) return true;
                    if (pd1.id > pd2.id) return false;
                    if (e1.distToPDLoc < e2.distToPDLoc) return true;
                    if (e1.distToPDLoc > e2.distToPDLoc) return false;
                    return (e1.distFromPDLocToNextStop < e2.distFromPDLocToNextStop);
                };

                if (cost < minCost || (cost == minCost && breakCostTie(h, rel.relevantSpots[bestIdx]))) {
                    bestIdx = j;
                    minCost = cost;
                }
            }
            LIGHT_KASSERT(minCost >= 0 && minCost < INFTY);
            std::swap(rel.relevantSpots[firstEntryForStop], rel.relevantSpots[bestIdx]);
            rel.relevantSpots.erase(rel.relevantSpots.begin() + firstEntryForStop + 1, rel.relevantSpots.end());
        }

        void removeAllDominatedOrdinaryDropoffs(RelevantPDLocs &rel, const int vehId, const int stopPos,
                                                const int firstEntryForStop, const RequestState &requestState) {
            using namespace time_utils;
            if (rel.relevantSpots.size() <= firstEntryForStop)
                return;

            const auto numStops = routeState.numStopsOf(vehId);

            // Returns true iff relevant dropoff h1 dominates relevant dropoff h2 for the purposes of ordinary
            // non-paired assignments at this stop.
            const auto dominates = [&](const RelevantPDLocs::RelevantPDLoc &h1,
                                       const RelevantPDLocs::RelevantPDLoc &h2) -> bool {
                using F = CostCalculator::CostFunction;
                const auto &config = InputConfig::getInstance();
                const auto &d1 = requestState.dropoffs[h1.pdId];
                const auto &d2 = requestState.dropoffs[h2.pdId];
                const auto isAtExistingStop1 = d1.loc == routeState.stopLocationsFor(vehId)[stopPos];
                const auto isAtExistingStop2 = d2.loc == routeState.stopLocationsFor(vehId)[stopPos];
                const auto detour1 = calcInitialDropoffDetour(vehId, stopPos, h1.distToPDLoc,
                                                              h1.distFromPDLocToNextStop, isAtExistingStop1,
                                                              routeState);
                const auto detour2 = calcInitialDropoffDetour(vehId, stopPos, h2.distToPDLoc,
                                                              h2.distFromPDLocToNextStop, isAtExistingStop2,
                                                              routeState);
                const auto minTripTime1 = h1.distToPDLoc + d1.walkingDist;
                const auto minTripTime2 = h2.distToPDLoc + d2.walkingDist;
                int tripCostDiff;
                if (minTripTime1 <= minTripTime2) {
                    tripCostDiff = F::calcLowerBoundTripCostDifference(minTripTime1 - minTripTime2, requestState);
                } else {
                    tripCostDiff = F::calcUpperBoundTripCostDifference(minTripTime1 - minTripTime2, requestState);
                }

                const int walkingCostDiff = F::calcWalkingCost(d1.walkingDist, config.dropoffRadius) -
                                            F::calcWalkingCost(d2.walkingDist, config.dropoffRadius);

                const int addedTripTimeOfOthers1 = calcAddedTripTimeInInterval(vehId, stopPos, numStops - 1, detour1,
                                                                               routeState);
                const int addedTripTimeOfOthers2 = calcAddedTripTimeInInterval(vehId, stopPos, numStops - 1, detour2,
                                                                               routeState);
                const int addedTripTimeCostDiff = F::calcChangeInTripCostsOfExistingPassengers(addedTripTimeOfOthers1) -
                                                  F::calcChangeInTripCostsOfExistingPassengers(addedTripTimeOfOthers2);

                const int vehCostDiff = F::calcVehicleCost(detour1 - detour2);
                return tripCostDiff + vehCostDiff + walkingCostDiff + addedTripTimeCostDiff < 0;
            };

            int numNonDominated = 0;
            for (int j = firstEntryForStop; j < rel.relevantSpots.size(); ++j) {
                const auto &h = rel.relevantSpots[j];

                // Check if h is dominated by any previously non-dominated dropoff
                bool isDominated = false;
                for (int i = 0; i < numNonDominated; ++i) {
                    if (dominates(rel.relevantSpots[firstEntryForStop + i], h)) {
                        isDominated = true;
                        break;
                    }
                }
                if (isDominated)
                    continue;

                // Remove any previously non-dominated dropoff that is dominated by h
                for (int i = numNonDominated - 1; i >= 0; --i) {
                    if (dominates(h, rel.relevantSpots[firstEntryForStop + i])) {
                        --numNonDominated;
                        std::swap(rel.relevantSpots[firstEntryForStop + i],
                                  rel.relevantSpots[firstEntryForStop + numNonDominated]);
                    }
                }

                // Add h to non-dominated dropoffs
                std::swap(rel.relevantSpots[firstEntryForStop + numNonDominated], rel.relevantSpots[j]);
                ++numNonDominated;
            }

            // Keep only non-dominated dropoffs
            rel.relevantSpots.erase(rel.relevantSpots.begin() + firstEntryForStop + numNonDominated,
                                    rel.relevantSpots.end());
        }


        inline bool isPickupRelevant(const Vehicle &veh, const int stopIndex, const unsigned int pickupId,
                                     const int distFromStopToPickup,
                                     const int distFromPickupToNextStop,
                                     const RequestState &requestState) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            assert(routeState.occupanciesFor(vehId)[stopIndex] + requestState.originalRequest.numRiders <=
                   veh.capacity);
            if (distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY)
                return false;

            assert(distFromStopToPickup + distFromPickupToNextStop >=
                   calcLengthOfLegStartingAt(stopIndex, vehId, routeState));

            const auto &p = requestState.pickups[pickupId];

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, distFromStopToPickup, p,
                                                                  requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX, depTimeAtPickup,
                                                                     distFromPickupToNextStop, requestState,
                                                                     routeState);

            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return false;


            const int curKnownCost = calculator.calcMinKnownPickupSideCost(veh, stopIndex, initialPickupDetour,
                                                                           p.walkingDist, depTimeAtPickup,
                                                                           requestState.minDirectPDDist, requestState);

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            if (curKnownCost > requestState.getBestCost())
                return false;

            return true;
        }

        inline bool isDropoffRelevant(const Vehicle &veh, const int stopIndex, const unsigned int dropoffId,
                                      const int distFromStopToDropoff,
                                      const int distFromDropoffToNextStop,
                                      const RequestState &requestState) {
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

            const int curMinCost = calculator.calcMinKnownDropoffSideCost(veh, stopIndex, initialDropoffDetour,
                                                                          d.walkingDist, requestState.minDirectPDDist,
                                                                          requestState);

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            if (curMinCost > requestState.getBestCost())
                return false;

            return true;
        }

        inline int getMinCostForPickup(const Vehicle &veh, const int stopIndex, const int minDistToPickup,
                                       const int minDistFromPickup, const RequestState &requestState) const {
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
                                                         requestState.minDirectPDDist, requestState);
        }

        inline int getMinCostForDropoff(const Vehicle &veh, const int stopIndex, const int minDistToDropoff,
                                        const int minDistFromDropoff, const RequestState &requestState) const {
            using namespace time_utils;
            int minInitialDropoffDetour = calcInitialDropoffDetour(veh.vehicleId, stopIndex, minDistToDropoff,
                                                                   minDistFromDropoff, false, routeState);
            minInitialDropoffDetour = std::max(minInitialDropoffDetour, 0);
            return calculator.calcMinKnownDropoffSideCost(veh, stopIndex, minInitialDropoffDetour, 0,
                                                          requestState.minDirectPDDist, requestState);
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
        CostCalculator calculator;
        const RouteState &routeState;

        LightweightSubset vehiclesWithFeasibleDistances;
    };
}