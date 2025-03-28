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
                        removeAllDominatedOrdinaryPdLocs<isDropoff>(rel, vehId, i, beginEntriesForStop, requestState);
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

        bool pickupDominates(const RelevantPDLocs::RelevantPDLoc &h1, const RelevantPDLocs::RelevantPDLoc &h2,
                             const int vehId, const int stopPos, const RequestState &requestState) const {
            using F = CostCalculator::CostFunction;
            using namespace time_utils;
            static const auto &config = InputConfig::getInstance();
            const auto numStops = routeState.numStopsOf(vehId);
            const auto &p1 = requestState.pickups[h1.pdId];
            const auto &p2 = requestState.pickups[h2.pdId];
            const auto depTimeAtPickup1 = getActualDepTimeAtPickup(vehId, stopPos, h1.distToPDLoc,
                                                                   p1, requestState, routeState);
            const auto depTimeAtPickup2 = getActualDepTimeAtPickup(vehId, stopPos, h2.distToPDLoc,
                                                                   p2, requestState, routeState);

            const auto arrTimeAtNext1 = depTimeAtPickup1 + h1.distFromPDLocToNextStop;
            const auto arrTimeAtNext2 = depTimeAtPickup2 + h2.distFromPDLocToNextStop;

            int tripCostDiff;
            if (arrTimeAtNext1 <= arrTimeAtNext2) {
                // Vehicle wait times along route (before dropoff) can eat up a trip time advantage of p1 over p2.
                // Thus, we subtract the sum of vehicle wait times until the last stop (the maximum possible) from the
                // trip time difference. However, vehicle wait times cannot reduce the trip time difference below 0.
                const int minTripTimeDiff = -std::max(arrTimeAtNext2 - arrTimeAtNext1 - getTotalVehWaitTimeInInterval(vehId, stopPos, numStops - 1, routeState), 0);
                tripCostDiff = F::calcLowerBoundTripCostDifference(minTripTimeDiff, requestState);
            } else {
                // Here, we consider the smallest possible vehicle wait time for an assignment where the pickup is
                // ordinary. This is the wait time at the next stop.
                const int maxTripTimeDiff = std::max(arrTimeAtNext1 - arrTimeAtNext2 - getTotalVehWaitTimeInInterval(vehId, stopPos, stopPos + 1, routeState), 0);
                tripCostDiff = F::calcUpperBoundTripCostDifference(maxTripTimeDiff, requestState);
            }

            const int walkingCostDiff = F::calcWalkingCost(p1.walkingDist, config.pickupRadius) -
                                        F::calcWalkingCost(p2.walkingDist, config.pickupRadius);

            const int detour1 = calcInitialPickupDetour(vehId, stopPos, INVALID_INDEX, depTimeAtPickup1,
                                                        h1.distFromPDLocToNextStop, requestState, routeState);
            const int detour2 = calcInitialPickupDetour(vehId, stopPos, INVALID_INDEX, depTimeAtPickup2,
                                                        h2.distFromPDLocToNextStop, requestState, routeState);
            const int addedTripTime1 = calcAddedTripTimeInInterval(vehId, stopPos, numStops - 1, detour1, routeState);
            const int addedTripTime2 = calcAddedTripTimeInInterval(vehId, stopPos, numStops - 1, detour2, routeState);
            const int addedTripTimeCostDiff = F::calcChangeInTripCostsOfExistingPassengers(addedTripTime1) -
                                              F::calcChangeInTripCostsOfExistingPassengers(addedTripTime2);
            const int waitVioCostDiff = F::calcWaitViolationCost(depTimeAtPickup1, requestState) -
                                        F::calcWaitViolationCost(depTimeAtPickup2, requestState);

            const auto residualDetourAtEnd1 = calcResidualPickupDetour(vehId, stopPos, numStops - 1, detour1,
                                                                       routeState);
            const auto residualDetourAtEnd2 = calcResidualPickupDetour(vehId, stopPos, numStops - 1, detour2,
                                                                       routeState);
            KASSERT(!isServiceTimeConstraintViolated(fleet[vehId], requestState, residualDetourAtEnd1, routeState));
            KASSERT(!isServiceTimeConstraintViolated(fleet[vehId], requestState, residualDetourAtEnd2, routeState));
            const int vehCostDiff = F::calcVehicleCost(residualDetourAtEnd1 - residualDetourAtEnd2);
            return vehCostDiff + tripCostDiff + walkingCostDiff + addedTripTimeCostDiff + waitVioCostDiff < 0;
        }

        bool dropoffDominates(const RelevantPDLocs::RelevantPDLoc &h1, const RelevantPDLocs::RelevantPDLoc &h2,
                              const int vehId, const int stopPos, const RequestState &requestState) const {
            using F = CostCalculator::CostFunction;
            using namespace time_utils;
            static const auto &config = InputConfig::getInstance();
            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLoc = routeState.stopLocationsFor(vehId)[stopPos];
            const auto minStopDuration = std::min(config.stopTime, routeState.schedDepTimesFor(vehId)[stopPos] -
                                                                   routeState.schedArrTimesFor(vehId)[stopPos]);
            const auto maxStopDuration = std::max(config.stopTime, routeState.schedDepTimesFor(vehId)[stopPos] -
                                                                   routeState.schedArrTimesFor(vehId)[stopPos]);
            const auto &d1 = requestState.dropoffs[h1.pdId];
            const auto &d2 = requestState.dropoffs[h2.pdId];
            const auto isAtExistingStop1 = d1.loc == stopLoc;
            const auto isAtExistingStop2 = d2.loc == stopLoc;
            const auto detour1 = calcInitialDropoffDetour(vehId, stopPos, h1.distToPDLoc, h1.distFromPDLocToNextStop,
                                                          isAtExistingStop1, routeState);
            const auto detour2 = calcInitialDropoffDetour(vehId, stopPos, h2.distToPDLoc, h2.distFromPDLocToNextStop,
                                                          isAtExistingStop2, routeState);

            // If detour1 is greater than detour2, then h1 may lead to a violation of a hard constraint while h2
            // does not. Thus, h1 cannot safely dominate h2.
            if (detour1 > detour2)
                return false;

            const auto minTripTime1 = ((!isAtExistingStop1) * maxStopDuration) + h1.distToPDLoc + d1.walkingDist;
            const auto minTripTime2 = ((!isAtExistingStop2) * minStopDuration) + h2.distToPDLoc + d2.walkingDist;
            int tripCostDiff;
            if (minTripTime1 <= minTripTime2) {
                tripCostDiff = F::calcLowerBoundTripCostDifference(minTripTime1 - minTripTime2, requestState);
            } else {
                tripCostDiff = F::calcUpperBoundTripCostDifference(minTripTime1 - minTripTime2, requestState);
            }

            const int walkingCostDiff = F::calcWalkingCost(d1.walkingDist, config.dropoffRadius) -
                                        F::calcWalkingCost(d2.walkingDist, config.dropoffRadius);

            const int addedTripTime1 = calcAddedTripTimeInInterval(vehId, stopPos, numStops - 1, detour1, routeState);
            const int addedTripTime2 = calcAddedTripTimeInInterval(vehId, stopPos, numStops - 1, detour2, routeState);
            const int addedTripTimeCostDiff = F::calcChangeInTripCostsOfExistingPassengers(addedTripTime1) -
                                              F::calcChangeInTripCostsOfExistingPassengers(addedTripTime2);

            const int vehCostDiff = F::calcVehicleCost(detour1 - detour2);
            return tripCostDiff + vehCostDiff + walkingCostDiff + addedTripTimeCostDiff < 0;
        }

        template<bool isDropoff>
        void removeAllDominatedOrdinaryPdLocs(RelevantPDLocs &rel, const int vehId, const int stopPos,
                                              const int firstEntryForStop, const RequestState &requestState) {
            using namespace time_utils;
            if (rel.relevantSpots.size() <= firstEntryForStop)
                return;

            int numNonDominated = 0;
            for (int j = firstEntryForStop; j < rel.relevantSpots.size(); ++j) {
                const auto &h = rel.relevantSpots[j];

                // Check if h is dominated by any previously non-dominated PD loc
                bool isDominated = false;
                for (int i = 0; i < numNonDominated; ++i) {
                    if ((isDropoff &&
                         dropoffDominates(rel.relevantSpots[firstEntryForStop + i], h, vehId, stopPos, requestState)) ||
                        (!isDropoff &&
                         pickupDominates(rel.relevantSpots[firstEntryForStop + i], h, vehId, stopPos, requestState))) {
                        isDominated = true;
                        break;
                    }
                }
                if (isDominated)
                    continue;

                // Remove any previously non-dominated PD loc that is dominated by h
                for (int i = numNonDominated - 1; i >= 0; --i) {
                    if ((isDropoff &&
                         dropoffDominates(h, rel.relevantSpots[firstEntryForStop + i], vehId, stopPos, requestState)) ||
                        (!isDropoff &&
                         pickupDominates(h, rel.relevantSpots[firstEntryForStop + i], vehId, stopPos, requestState))) {
                        --numNonDominated;
                        std::swap(rel.relevantSpots[firstEntryForStop + i],
                                  rel.relevantSpots[firstEntryForStop + numNonDominated]);
                    }
                }

                // Add h to non-dominated PD locs
                std::swap(rel.relevantSpots[firstEntryForStop + numNonDominated], rel.relevantSpots[j]);
                ++numNonDominated;
            }

            // Keep only non-dominated PD locs
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