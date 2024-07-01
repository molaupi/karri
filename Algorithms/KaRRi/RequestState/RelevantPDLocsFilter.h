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
                             const FeasibleDistancesT &feasiblePickupDistances,
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

                    if ((!isDropoff || beforeNextStop) &&
                        occupancies[i] + requestState.originalRequest.numRiders > veh.capacity)
                        continue;

                    const auto &stopId = stopIds[i];

                    // Insert entries at this stop
                    if (!feasible.hasPotentiallyRelevantPDLocs(stopId))
                        continue;

                    KASSERT(vehiclesWithFeasibleDistances.contains(vehId));

                    // Check with lower bounds on travel times to and from PD loc whether this stop needs to be regarded
                    const int minTravelTimeToPDLoc = feasible.minTravelTimeToRelevantPDLocsFor(stopId);
                    const int minTravelTimeFromPDLoc = feasible.minTravelTimeFromPDLocToNextStopOf(stopId);

                    if constexpr (isDropoff) {
                        auto minDropoffDetour = calcInitialDropoffDetour(vehId, i, minTravelTimeToPDLoc,
                                                                         minTravelTimeFromPDLoc, false, routeState);
                        minDropoffDetour = std::max(minDropoffDetour, 0);
                        if (doesDropoffDetourViolateHardConstraints(veh, requestState, i, minDropoffDetour, routeState))
                            continue;
                    } else {
                        const int minVehDepTimeAtPickup =
                                getVehDepTimeAtStopForRequest(vehId, i, requestState, routeState) +
                                minTravelTimeToPDLoc;
                        const int minDepTimeAtPickup = std::max(requestState.originalRequest.requestTime,
                                                                minVehDepTimeAtPickup);
                        int minPickupDetour = calcInitialPickupDetour(veh.vehicleId, i, INVALID_INDEX,
                                                                      minDepTimeAtPickup, minTravelTimeFromPDLoc,
                                                                      requestState, routeState);
                        minPickupDetour = std::max(minPickupDetour, 0);
                        if (doesPickupDetourViolateHardConstraints(veh, requestState, i, minPickupDetour, routeState))
                            continue;
                    }

                    // Compute lower bound cost and compare to best known cost
                    const int minCostToPDLoc = feasible.minCostToRelevantPDLocsFor(stopId);
                    const int minCostFromPDLoc = feasible.minCostFromPDLocToNextStopOf(stopId);
                    const auto minCost = minCostToPDLoc + minCostFromPDLoc - routeState.legCostsFor(vehId)[i];
                    if (minCost > requestState.getBestCost())
                        continue;

                    ++numStopsRelevant;
                    // Check each PD loc
                    const auto &costsToPDLocs = feasible.costsToRelevantPDLocsFor(stopId);
                    const auto &costsFromPDLocs = feasible.costsFromRelevantPDLocsToNextStopOf(stopId);
                    const auto &travelTimesToPDLocs = feasible.travelTimesToRelevantPDLocsFor(stopId);
                    const auto &travelTimesFromPDLocs = feasible.travelTimesFromRelevantPDLocsToNextStopOf(stopId);
                    for (unsigned int id = 0; id < numPDLocs; ++id) {

                        bool isRelevant;
                        if constexpr (isDropoff) {
                            isRelevant = isDropoffRelevant(fleet[vehId], i, id, costsToPDLocs[id], costsFromPDLocs[id],
                                                           travelTimesToPDLocs[id], travelTimesFromPDLocs[id]);
                        } else {
                            isRelevant = isPickupRelevant(fleet[vehId], i, id, costsToPDLocs[id], costsFromPDLocs[id],
                                                          travelTimesToPDLocs[id], travelTimesFromPDLocs[id]);
                        }

                        if (isRelevant) {
                            rel.relevantSpots.push_back({i, id, costsToPDLocs[id], costsFromPDLocs[id],
                                                         travelTimesToPDLocs[id], travelTimesFromPDLocs[id]});
                        }
                    }


                }

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (rel.relevantSpots.size() > rel.startOfRelevantPDLocs[vehId])
                    rel.vehiclesWithRelevantSpots.insert(vehId);
            }

            rel.startOfRelevantPDLocs[fleet.size()] = rel.relevantSpots.size();

            KASSERT(std::all_of(rel.relevantSpots.begin(), rel.relevantSpots.end(),
                                [&](const auto &h) {
                                    return h.costToPDLoc < INFTY && h.costFromPDLocToNextStop < INFTY;
                                }));

            return numStopsRelevant;
        }

        inline bool isPickupRelevant(const Vehicle &veh, const int stopIndex, const unsigned int pickupId,
                                     const int costFromStopToPickup,
                                     const int costFromPickupToNextStop,
                                     const int travelTimeFromStopToPickup,
                                     const int travelTimeFromPickupToStop) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            KASSERT(routeState.occupanciesFor(vehId)[stopIndex] + requestState.originalRequest.numRiders <=
                    veh.capacity);
            if (costFromStopToPickup >= INFTY || costFromPickupToNextStop >= INFTY)
                return false;

            KASSERT(costFromStopToPickup + costFromPickupToNextStop >= routeState.legCostsFor(vehId)[stopIndex]);

            const auto &p = requestState.pickups[pickupId];

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, travelTimeFromStopToPickup, p,
                                                                  requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX, depTimeAtPickup,
                                                                     travelTimeFromPickupToStop, requestState,
                                                                     routeState);
            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return false;


            const int curKnownCost =
                    costFromStopToPickup + costFromPickupToNextStop - routeState.legCostsFor(vehId)[stopIndex];

            // If cost for only pickup side is already worse than best known cost for a whole assignment, then
            // this pickup is not relevant at this stop.
            return curKnownCost <= requestState.getBestCost();
        }

        inline bool isDropoffRelevant(const Vehicle &veh, const int stopIndex, const unsigned int dropoffId,
                                      const int costFromStopToDropoff,
                                      const int costFromDropoffToNextStop,
                                      const int travelTimeFromStopToDropoff,
                                      const int travelTimeFromDropoffToNextStop) {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;
            const auto &d = requestState.dropoffs[dropoffId];

            // If this is the last stop in the route, we only consider this dropoff for ordinary assignments if it is at the
            // last stop. Similarly, if the vehicle is full after this stop, we can't perform the dropoff here unless the
            // dropoff coincides with the stop. A dropoff at an existing stop causes no detour, so it is always relevant.
            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &occupancy = routeState.occupanciesFor(vehId)[stopIndex];
            const auto &stopLocations = routeState.stopLocationsFor(vehId);
            KASSERT(d.loc != stopLocations[stopIndex] || costFromStopToDropoff == 0);
            if (stopIndex == numStops - 1 || occupancy + requestState.originalRequest.numRiders > veh.capacity)
                return d.loc == stopLocations[stopIndex];

            if (stopLocations[stopIndex + 1] == d.loc)
                return false;

            if (costFromStopToDropoff >= INFTY || costFromDropoffToNextStop >= INFTY)
                return false;

            const bool isDropoffAtExistingStop = d.loc == stopLocations[stopIndex];
            const int initialDropoffDetour = calcInitialDropoffDetour(vehId, stopIndex, travelTimeFromStopToDropoff,
                                                                      travelTimeFromDropoffToNextStop,
                                                                      isDropoffAtExistingStop, routeState);
            KASSERT(initialDropoffDetour >= 0);
            if (doesDropoffDetourViolateHardConstraints(veh, requestState, stopIndex, initialDropoffDetour, routeState))
                return false;

            const int curMinCost =
                    costFromStopToDropoff + costFromDropoffToNextStop - routeState.legCostsFor(vehId)[stopIndex];

            // If cost for only dropoff side is already worse than best known cost for a whole assignment, then
            // this dropoff is not relevant at this stop.
            return curMinCost <= requestState.getBestCost();
        }

        int recomputeCostToPDLocDirectly(const int vehId, const int stopIdxBefore, const int pdLocLocation) {
            auto src = ch.rank(inputGraph.edgeHead(routeState.stopLocationsFor(vehId)[stopIdxBefore]));
            auto tar = ch.rank(inputGraph.edgeTail(pdLocLocation));
            auto offset = inputGraph.travelTime(pdLocLocation);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }

        int recomputeCostFromPDLocDirectly(const int vehId, const int stopIdxAfter, const int pdLocLocation) {
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

        const FeasibleDistancesT &feasiblePickupDistances;
        const FeasibleDistancesT &feasibleDropoffDistances;

        RelevantPDLocs &relOrdinaryPickups;
        RelevantPDLocs &relOrdinaryDropoffs;
        RelevantPDLocs &relPickupsBeforeNextStop;
        RelevantPDLocs &relDropoffsBeforeNextStop;
    };
}