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
#include "Algorithms/KaRRi/EllipticBCH/FeasiblePDLocs.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "DataStructures/Pareto/ParetoBag.h"

namespace karri {

// Filters the feasible PD locs found by the elliptic BCH searches to the pickups/dropoffs that are relevant for
// certain stops and assignment types.
//
// Filtering is based on two observations:
// 1. Cost: A PD loc is irrelevant at a stop if a lower bound on the cost of any assignment using the PD loc at the
//    stop is worse than the cost of the best known assignment. Thus, filtering should happen as late as possible to
//    obtain a tight upper bound (best known cost).
// 2. Pareto-optimality: For non-paired ordinary assignments (pickup and dropoff not inserted between the same pair
//    of stops) and the PD loc that is inserted after an ordinary stop (i.e. after stop 0), a pickup p can only be
//    part of the best assignment for a given pickup stop if no other pickup p' at the same stop has both a smaller
//    fixed cost (walking and wait time violation cost) and a smaller detour (i.e. an earlier arrival at the next stop)
//    than p. Similarly, a dropoff d can only be part of the best assignment if no other dropoff d' at the same stop
//    has both a smaller fixed cost (walking and trip time cost from the stop to the destination) and a smaller detour.
//    Since every other part of the cost of an assignment is monotonous in the respective detour, the dominated PD loc
//    cannot lead to a better assignment for any choice of the remaining assignment parameters.
//    Pareto-optimality does not apply to paired assignments or to pickups before the next stop (as the exact
//    departure time at the pickup depends on the current location of the vehicle). Paired assignments use the
//    unfiltered FeasiblePDLocs instead.
    template<typename InputGraphT, typename CHEnvT>
    class RelevantPDLocsFilter {

        using F = CostCalculator::CostFunction;

    public:

        RelevantPDLocsFilter(const Fleet &fleet, const InputGraphT &inputGraph, const CHEnvT &,
                             const RouteState &routeState)
                : fleet(fleet),
                  inputGraph(inputGraph),
                  calculator(routeState),
                  routeState(routeState),
                  vehiclesWithFeasibleDistances(fleet.size()) {}


        RelevantPDLocs
        filterOrdinaryPickups(const FeasiblePDLocs &feasiblePickups, const RequestState &requestState,
                              const PDLocs &pdLocs,
                              stats::OrdAssignmentsPerformanceStats &stats) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filterOrdinary<false>(feasiblePickups, pdLocs.pickups, numRelStops, requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterRelevantPDLocsTime += time;
            stats.numRelevantStopsForPickups += numRelStops;

            return rel;
        }

        RelevantPDLocs
        filterOrdinaryDropoffs(const FeasiblePDLocs &feasibleDropoffs, const RequestState &requestState,
                               const PDLocs &pdLocs,
                               stats::OrdAssignmentsPerformanceStats &stats) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filterOrdinary<true>(feasibleDropoffs, pdLocs.dropoffs, numRelStops, requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterRelevantPDLocsTime += time;
            stats.numRelevantStopsForDropoffs += numRelStops;

            return rel;
        }

        RelevantPDLocs
        filterPickupsBeforeNextStop(const FeasiblePDLocs &feasiblePickups, const RequestState &requestState,
                                    const PDLocs &pdLocs,
                                    stats::PbnsAssignmentsPerformanceStats &stats) {
            Timer timer;

            int numRelStops = 0;
            const auto rel = filterPickupsBeforeNextStop(feasiblePickups, pdLocs.pickups, numRelStops, requestState);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterRelevantPDLocsTime += time;
            stats.numRelevantStopsForPickups += numRelStops;
            return rel;
        }

    private:

        // Relevant PD loc with the values of the Pareto criteria.
        struct RelevantPDLocWithCriteria {
            RelevantPDLocs::RelevantPDLoc relPdLoc;

            // For pickups: fixed cost of pickup (walking cost and wait time violation cost);
            // for dropoffs: fixed cost of dropoff (walking cost and linear part of trip cost from stop to destination)
            int fixedCost;

            // For dropoffs: if trip cost is not linear (trip time violation is penalized), time from departure at
            // stop to arrival at destination; otherwise 0.
            int tripTimeFromStop;

            // Initial detour caused by the PD loc (for pickups, this is equivalent to the arrival time at the next stop)
            int detour;

            // Dropoffs at an existing stop do not incur an additional stop time. To be safe, we never let a dropoff
            // that is not at the existing stop dominate one that is.
            bool atExistingStop;

            static bool dominates(const RelevantPDLocWithCriteria &a, const RelevantPDLocWithCriteria &b) {
                if (b.atExistingStop && !a.atExistingStop)
                    return false;
                return a.fixedCost <= b.fixedCost && a.tripTimeFromStop <= b.tripTimeFromStop &&
                       a.detour <= b.detour &&
                       (a.fixedCost < b.fixedCost || a.tripTimeFromStop < b.tripTimeFromStop || a.detour < b.detour);
            }
        };

        // Filters feasible pickups or dropoffs for non-paired ordinary assignments using cost lower bounds and
        // Pareto-optimality (see above).
        template<bool isDropoff>
        RelevantPDLocs filterOrdinary(const FeasiblePDLocs &feasible, const std::vector<PDLoc> &pdLocs,
                                      int &numStopsRelevant, const RequestState &requestState) {
            using namespace time_utils;

            numStopsRelevant = 0;
            const int numPDLocs = static_cast<int>(pdLocs.size());

            RelevantPDLocs rel(fleet.size());

            vehiclesWithFeasibleDistances.clear();
            for (const auto &stopId: feasible.getStopIdsWithFeasiblePDLocs()) {
                const auto vehId = routeState.vehicleIdOf(stopId);
                if (vehiclesWithFeasibleDistances.contains(vehId))
                    continue;
                const auto stopPos = routeState.stopPositionOf(stopId);
                if (stopPos == 0 || (!isDropoff && stopPos == routeState.numStopsOf(vehId) - 1))
                    continue;
                vehiclesWithFeasibleDistances.insert(vehId);
            }

            for (const auto &vehId: vehiclesWithFeasibleDistances) {
                const auto &veh = fleet[vehId];
                const auto numStops = routeState.numStopsOf(vehId);
                const auto stopIds = routeState.stopIdsFor(vehId);
                const auto occupancies = routeState.occupanciesFor(vehId);
                KASSERT(numStops > 1);

                const int totalNumRelPdLocsBefore = static_cast<int>(rel.relevantSpots.size());

                // Track relevant PD locs for each stop in the relevant PD locs data structure.
                // Entries are ordered by vehicle and by stop.
                const int endStopIdx = isDropoff ? numStops : numStops - 1;
                for (int i = 1; i < endStopIdx; ++i) {

                    if (!isDropoff && occupancies[i] + requestState.originalRequest.numRiders > veh.capacity)
                        continue;

                    const auto &stopId = stopIds[i];
                    if (!feasible.hasFeasiblePDLocs(stopId))
                        continue;

                    // If there is more than one PD loc, check with lower bounds on dist to and from PD locs whether
                    // this stop needs to be regarded before looking at every PD loc.
                    if (numPDLocs > 1) {
                        const int minDistToPDLoc = feasible.minDistToPDLocFor(stopId);
                        const int minDistFromPDLoc = feasible.minDistFromPDLocToNextStopOf(stopId);
                        int minCost;
                        if constexpr (isDropoff) {
                            minCost = getMinCostForDropoff(veh, i, minDistToPDLoc, minDistFromPDLoc, requestState);
                        } else {
                            minCost = getMinCostForPickup(veh, i, minDistToPDLoc, minDistFromPDLoc, requestState);
                        }
                        if (minCost > requestState.getBestCost())
                            continue;
                    }

                    ++numStopsRelevant;

                    // Check each feasible PD loc for relevance and add relevant PD locs to Pareto bag for this stop.
                    paretoBag.clear();
                    for (const auto &[id, distToPDLoc, distFromPDLoc]: feasible.feasiblePdLocsFor(stopId)) {
                        RelevantPDLocWithCriteria candidate{{i, static_cast<unsigned int>(id), distToPDLoc, distFromPDLoc},
                                                            INFTY, INFTY, INFTY, false};
                        bool isRelevant;
                        if constexpr (isDropoff) {
                            isRelevant = computeCriteriaForRelevantDropoff(veh, i, pdLocs[id], distToPDLoc,
                                                                           distFromPDLoc, requestState, candidate);
                        } else {
                            isRelevant = computeCriteriaForRelevantPickup(veh, i, pdLocs[id], distToPDLoc,
                                                                          distFromPDLoc, requestState, candidate);
                        }
                        if (isRelevant)
                            paretoBag.addCandidate(candidate);
                    }

                    for (const auto &spotWithCriteria: paretoBag)
                        rel.relevantSpots.push_back(spotWithCriteria.relPdLoc);
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

        // Filters feasible pickups for assignments where the pickup is inserted before the next stop of a vehicle.
        // Pareto-optimality does not apply here since the exact distance from the current location of the vehicle
        // to a pickup is not known yet.
        RelevantPDLocs filterPickupsBeforeNextStop(const FeasiblePDLocs &feasible, const std::vector<PDLoc> &pickups,
                                                   int &numStopsRelevant, const RequestState &requestState) {
            using namespace time_utils;

            numStopsRelevant = 0;
            const int numPickups = static_cast<int>(pickups.size());

            RelevantPDLocs rel(fleet.size());

            for (const auto &stopId: feasible.getStopIdsWithFeasiblePDLocs()) {
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopPos = routeState.stopPositionOf(stopId);
                if (stopPos > 0 || routeState.numStopsOf(vehId) == 1)
                    continue;
                // Skip vehicles that are already on edge that represents next stop since they can't have relevant
                // PD locs before next stop
                if (routeState.schedArrTimesFor(vehId)[1] -
                    inputGraph.travelTime(routeState.stopLocationsFor(vehId)[1]) <= requestState.dispatchingTime)
                    continue;

                const auto &veh = fleet[vehId];
                if (routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders > veh.capacity)
                    continue;

                // If there is more than one PD loc, check with lower bounds on dist to and from PD locs whether
                // this stop needs to be regarded before looking at every PD loc.
                if (numPickups > 1) {
                    const int minCost = getMinCostForPickup(veh, 0, feasible.minDistToPDLocFor(stopId),
                                                            feasible.minDistFromPDLocToNextStopOf(stopId),
                                                            requestState);
                    if (minCost > requestState.getBestCost())
                        continue;
                }

                ++numStopsRelevant;
                const int totalNumRelPdLocsBefore = static_cast<int>(rel.relevantSpots.size());
                for (const auto &[id, distToPDLoc, distFromPDLoc]: feasible.feasiblePdLocsFor(stopId)) {
                    if (isPickupRelevant(veh, 0, pickups[id], distToPDLoc, distFromPDLoc, requestState))
                        rel.relevantSpots.push_back({0, static_cast<unsigned int>(id), distToPDLoc, distFromPDLoc});
                }

                // If vehicle has at least one relevant PD loc, add the vehicle
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

        // Checks whether a pickup is relevant at a stop for an assignment where the pickup is inserted between
        // the stop and the next stop.
        inline bool isPickupRelevant(const Vehicle &veh, const int stopIndex, const PDLoc &pickup,
                                     const int distFromStopToPickup,
                                     const int distFromPickupToNextStop,
                                     const RequestState &requestState) const {
            int depTimeAtPickup, initialPickupDetour;
            return isPickupRelevant(veh, stopIndex, pickup, distFromStopToPickup, distFromPickupToNextStop,
                                    requestState, depTimeAtPickup, initialPickupDetour);
        }

        // Checks whether a pickup is relevant at a stop for an assignment where the pickup is inserted between
        // the stop and the next stop. Writes departure time at pickup and initial pickup detour to out parameters.
        inline bool isPickupRelevant(const Vehicle &veh, const int stopIndex, const PDLoc &pickup,
                                     const int distFromStopToPickup,
                                     const int distFromPickupToNextStop,
                                     const RequestState &requestState,
                                     int &depTimeAtPickup, int &initialPickupDetour) const {
            KASSERT(routeState.occupanciesFor(veh.vehicleId)[stopIndex] + requestState.originalRequest.numRiders <=
                    veh.capacity);
            KASSERT(distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY ||
                    distFromStopToPickup + distFromPickupToNextStop >=
                    time_utils::calcLengthOfLegStartingAt(stopIndex, veh.vehicleId, routeState));

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            const int minCost = calculator.calcMinCostForPickupBetweenStops(veh, stopIndex, pickup,
                                                                            distFromStopToPickup,
                                                                            distFromPickupToNextStop, requestState,
                                                                            depTimeAtPickup, initialPickupDetour);
            return minCost < INFTY && minCost <= requestState.getBestCost();
        }

        // Checks whether the pickup is relevant for a non-paired ordinary assignment at the given stop. If so,
        // computes the Pareto criteria for the pickup.
        inline bool computeCriteriaForRelevantPickup(const Vehicle &veh, const int stopIndex, const PDLoc &pickup,
                                                     const int distFromStopToPickup,
                                                     const int distFromPickupToNextStop,
                                                     const RequestState &requestState,
                                                     RelevantPDLocWithCriteria &criteria) const {
            KASSERT(stopIndex < routeState.numStopsOf(veh.vehicleId) - 1);

            // If the pickup coincides with the location of the next stop, a pickup at the next stop is at least as
            // good for non-paired assignments. It is considered at the next stop.
            if (pickup.loc == routeState.stopLocationsFor(veh.vehicleId)[stopIndex + 1])
                return false;

            int depTimeAtPickup, initialPickupDetour;
            if (!isPickupRelevant(veh, stopIndex, pickup, distFromStopToPickup, distFromPickupToNextStop,
                                  requestState, depTimeAtPickup, initialPickupDetour))
                return false;

            // The detour is equivalent to the arrival time at the next stop.
            criteria.fixedCost = F::calcWalkingCost(pickup.walkingDist) +
                                 F::calcWaitViolationCost(depTimeAtPickup, requestState);
            criteria.tripTimeFromStop = 0;
            criteria.detour = initialPickupDetour;
            criteria.atExistingStop = false;
            return true;
        }

        // Checks whether the dropoff is relevant for a non-paired ordinary assignment at the given stop. If so,
        // computes the Pareto criteria for the dropoff.
        inline bool computeCriteriaForRelevantDropoff(const Vehicle &veh, const int stopIndex, const PDLoc &dropoff,
                                                      const int distFromStopToDropoff,
                                                      const int distFromDropoffToNextStop,
                                                      const RequestState &requestState,
                                                      RelevantPDLocWithCriteria &criteria) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            // If this is the last stop in the route, we only consider this dropoff for ordinary assignments if it is at the
            // last stop. Similarly, if the vehicle is full after this stop, we can't perform the dropoff here unless the
            // dropoff coincides with the stop. A dropoff at an existing stop causes no detour, so it is always relevant.
            const auto numStops = routeState.numStopsOf(vehId);
            const auto occupancy = routeState.occupanciesFor(vehId)[stopIndex];
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            const bool isDropoffAtExistingStop = dropoff.loc == stopLocations[stopIndex];
            KASSERT(!isDropoffAtExistingStop || distFromStopToDropoff == 0);
            if (stopIndex == numStops - 1 || occupancy + requestState.originalRequest.numRiders > veh.capacity) {
                if (!isDropoffAtExistingStop)
                    return false;
                setDropoffCriteria(dropoff, 0, 0, true, criteria);
                return true;
            }

            if (stopLocations[stopIndex + 1] == dropoff.loc)
                return false;

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            int initialDropoffDetour;
            const int minCost = calculator.calcMinCostForDropoffBetweenStops(veh, stopIndex, dropoff,
                                                                             distFromStopToDropoff,
                                                                             distFromDropoffToNextStop,
                                                                             isDropoffAtExistingStop, requestState,
                                                                             initialDropoffDetour);
            if (minCost >= INFTY || minCost > requestState.getBestCost())
                return false;

            setDropoffCriteria(dropoff, distFromStopToDropoff, initialDropoffDetour, isDropoffAtExistingStop,
                               criteria);
            return true;
        }

        static inline void setDropoffCriteria(const PDLoc &dropoff, const int distFromStopToDropoff,
                                              const int initialDropoffDetour, const bool isDropoffAtExistingStop,
                                              RelevantPDLocWithCriteria &criteria) {
            // The trip time of the rider is the time until the vehicle departs from the stop before the dropoff
            // (which depends on the pickup) plus the time from the stop to the destination. The cost for the latter
            // part is independent of the pickup if the trip cost is linear in the trip time. Otherwise, we need to
            // additionally consider the trip time from the stop as a Pareto criterion.
            const int tripTimeFromStop = distFromStopToDropoff + dropoff.walkingDist;
            criteria.fixedCost = F::calcWalkingCost(dropoff.walkingDist) + F::PSG_WEIGHT * tripTimeFromStop;
            criteria.tripTimeFromStop = F::TRIP_VIO_WEIGHT == 0 ? 0 : tripTimeFromStop;
            criteria.detour = initialDropoffDetour;
            criteria.atExistingStop = isDropoffAtExistingStop;
        }

        inline int getMinCostForPickup(const Vehicle &veh, const int stopIndex, const int minDistToPickup,
                                       const int minDistFromPickup, const RequestState &requestState) const {
            using namespace time_utils;
            const int minVehDepTimeAtPickup =
                    getVehDepTimeAtStopForRequest(veh.vehicleId, stopIndex, requestState, routeState)
                    + minDistToPickup;
            const int minDepTimeAtPickup = std::max(requestState.dispatchingTime, minVehDepTimeAtPickup);
            int minInitialPickupDetour = calcInitialPickupDetour(veh.vehicleId, stopIndex, INVALID_INDEX,
                                                                 minDepTimeAtPickup, minDistFromPickup, requestState,
                                                                 routeState);
            minInitialPickupDetour = std::max(minInitialPickupDetour, 0);
            return calculator.calcMinKnownPickupSideCost(veh, stopIndex, minInitialPickupDetour, 0, minDepTimeAtPickup,
                                                         requestState);
        }

        inline int getMinCostForDropoff(const Vehicle &veh, const int stopIndex, const int minDistToDropoff,
                                        const int minDistFromDropoff, const RequestState &requestState) const {
            using namespace time_utils;
            int minInitialDropoffDetour = calcInitialDropoffDetour(veh.vehicleId, stopIndex, minDistToDropoff,
                                                                   minDistFromDropoff, false, routeState);
            minInitialDropoffDetour = std::max(minInitialDropoffDetour, 0);
            return calculator.calcMinKnownDropoffSideCost(veh, stopIndex, minInitialDropoffDetour, 0, requestState);
        }

        const Fleet &fleet;
        const InputGraphT &inputGraph;
        CostCalculator calculator;
        const RouteState &routeState;

        LightweightSubset vehiclesWithFeasibleDistances;
        ParetoBag<RelevantPDLocWithCriteria> paretoBag;
    };
}
