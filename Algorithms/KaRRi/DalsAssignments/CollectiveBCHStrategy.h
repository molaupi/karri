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

#include "MinCostDropoffAfterLastStopQuery.h"
#include "Algorithms/KaRRi/LastStopSearches/LabelBucketContainer.h"
#include "Algorithms/KaRRi/LastStopSearches/ClosestPDLocToLastStopBCHQuery.h"

namespace karri::DropoffAfterLastStopStrategies {

    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename CurVehLocToPickupSearchesT,
            typename RouteStateT,
            typename CostCalculatorT,
            typename FallBackCHLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    class CollectiveBCHStrategy {


        // Checks whether a DALS assignment is possible for this vehicle.
        // Call operator returns false if the vehicle has no relevant pickups along its route before the last stop.
        struct IsVehEligibleForDropoffAfterLastStop {

            IsVehEligibleForDropoffAfterLastStop(const CollectiveBCHStrategy &strat) : strat(strat) {}

            bool operator()(const int &vehId) const {
                return strat.routeState.numStopsOf(vehId) > 1 &&
                       (strat.relevantPickupsBeforeNextStop.hasRelevantSpotsFor(vehId) ||
                        strat.relevantOrdinaryPickups.hasRelevantSpotsFor(vehId));
            }

        private:
            const CollectiveBCHStrategy &strat;
        };


        using MinCostLabelSearch = MinCostDropoffAfterLastStopQuery<InputGraphT, CHEnvT, LastStopBucketsEnvT, IsVehEligibleForDropoffAfterLastStop>;
        using ClosestDropoffToLastStopQuery = ClosestPDLocToLastStopBCHQueryWithStallOnDemand<InputGraphT, CHEnvT, typename LastStopBucketsEnvT::BucketContainer>;

    public:

        CollectiveBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const RouteStateT &routeState,
                              const CHEnvT &chEnv,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const CostCalculatorT &calculator,
                              CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                              RequestState<CostCalculatorT> &requestState,
                              const RelevantPDLocs &relevantOrdinaryPickups,
                              const RelevantPDLocs &relevantPickupsBeforeNextStop,
                              const InputConfig &inputConfig)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  routeState(routeState),
                  calculator(calculator),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  closestDropoffSearch(inputGraph, fleet.size(), chEnv, lastStopBucketsEnv.getBuckets(),
                                       {chEnv.getCH().upwardGraph()}),
                  ch(chEnv.getCH()),
                  requestState(requestState),
                  relevantOrdinaryPickups(relevantOrdinaryPickups),
                  relevantPickupsBeforeNextStop(relevantPickupsBeforeNextStop),
                  inputConfig(inputConfig),
                  isVehEligibleForDropoffAfterLastStop(*this),
                  minCostSearch(inputGraph, fleet, chEnv, calculator, lastStopBucketsEnv,
                                isVehEligibleForDropoffAfterLastStop, routeState, requestState,
                                inputConfig),
                  distsFromLastStopToDropoffs(0, INFTY),
                  checkPBNSForVehicle(fleet.size()),
                  fullCHQuery(chEnv.template getFullCHQuery<FallBackCHLabelSet>()) {}

        void tryDropoffAfterLastStop() {
            runCollectiveSearch();
            enumerateAssignments();
        }


    private:

        void runCollectiveSearch() {
            Timer timer;

            minCostSearch.run();

            auto& stats = requestState.stats().dalsAssignmentsStats;
            stats.searchTime +=  minCostSearch.getRunTime();
            stats.numEdgeRelaxationsInSearchGraph += minCostSearch.getNumEdgeRelaxations();
            stats.numVerticesOrLabelsSettled += minCostSearch.getNumLabelsRelaxed();
            stats.numEntriesOrLastStopsScanned += minCostSearch.getNumEntriesScanned();
            stats.numCandidateVehicles += minCostSearch.getVehiclesSeen().size();
            stats.collective_initializationTime += minCostSearch.getInitializationTime();
            stats.collective_numDominationRelationTests += minCostSearch.getNumDominationRelationTests();
        }

        // For each vehicle, combine the dropoff with minimal cost with all relevant pickups along the route and check
        // whether the resulting assignment is better than the current best and whether it violates the service time
        // constraint. If the assignment is worse than the current best (irrespective of whether it violates the
        // constraint), inserting this pickup at this stop of the vehicle with any dropoff is worse than the current
        // best. If the assignment is better than the current best, we check whether it satisfies the service time
        // constraint. If yes, it is the new best assignment and no other dropoff will be better for this combination of
        // vehicle, pickup and stop index. If no, we have to check if other dropoffs exist that fulfil the service time
        // constraint and are still better than the current best known assignment. We mark those assignments as constraint
        // breakers, and filter them again using cost lower bounds after trying all feasible assignments with pareto best
        // dropoffs. Then, if any constraint breakers remain for a vehicle veh, we compute the distances from the last
        // stop of veh to all dropoffs and try every assignment explicitly.
        void enumerateAssignments() {
            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
            int numAssignmentsTried = 0;
            int numParetoBestLabels = 0;
            int numFallBackChSearches = 0;
            bool ranClosestDropoffSearch = false;

            Timer timer;
            distsFromLastStopToDropoffs.resize(requestState.numDropoffs());
            constraintBreakers.clear();

            enumerateAssignmentsWithOrdinaryPickup(numAssignmentsTried, numParetoBestLabels, numFallBackChSearches,
                                                   ranClosestDropoffSearch);
            enumerateAssignmentsWithPBNS(numAssignmentsTried, numParetoBestLabels, numFallBackChSearches,
                                         ranClosestDropoffSearch);

            // Time spent to locate vehicles and compute distances from current vehicle locations to pickups is counted
            // into PBNS time so subtract it here.
            const int64_t pbnsTime = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                     curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() -
                                     pbnsTimeBefore;

            auto &stats = requestState.stats().dalsAssignmentsStats;
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>() - pbnsTime;
            stats.tryAssignmentsTime = time;
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.numCandidateDropoffsAcrossAllVehicles += numParetoBestLabels;
            stats.collective_ranClosestDropoffSearch = ranClosestDropoffSearch;
            stats.collective_numDirectCHSearches += numFallBackChSearches;
            if (ranClosestDropoffSearch) {
                stats.numEdgeRelaxationsInSearchGraph += closestDropoffSearch.getNumEdgeRelaxations();
                stats.numVerticesOrLabelsSettled += closestDropoffSearch.getNumVerticesSettled();
                stats.numEntriesOrLastStopsScanned += closestDropoffSearch.getNumEntriesScanned();
            }
        }

        void enumerateAssignmentsWithOrdinaryPickup(int &numAssignmentsTried, int &numParetoBestLabels,
                                                    int &numFallBackChSearches, bool &ranClosestDropoffSearch) {
            using namespace time_utils;
            Assignment asgn;

            checkPBNSForVehicle.reset();
            for (const auto &vehId: minCostSearch.getVehiclesSeen()) {
                numParetoBestLabels += minCostSearch.getParetoBestDropoffLabelsFor(vehId).size();

                if (!relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    // vehicle may still have relevant assignment with pickup before next stop
                    checkPBNSForVehicle.set(vehId);
                    continue;
                }


                for (const auto &label: minCostSearch.getParetoBestDropoffLabelsFor(vehId)) {
                    asgn.dropoff = &requestState.dropoffs[label.dropoffId];
                    const auto &distFromLastStopToDropoff = label.distToDropoff;

                    // If a different assignment that has already been checked is better than the cost of only the dropoff
                    // side of this assignment, then skip this assignment. The pareto best labels for the same vehicle are
                    // additionally sorted by increasing dropoff side cost, so if one is worse than the best known assignment
                    // all other labels for this vehicle can also be skipped.
                    if (calculator.calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(
                            distFromLastStopToDropoff, *asgn.dropoff, requestState) > requestState.getBestCost())
                        break;  // no need to check pickup before next stop

                    const auto &numStops = routeState.numStopsOf(vehId);
                    const auto &occupancies = routeState.occupanciesFor(vehId);

                    asgn.vehicle = &fleet[vehId];
                    asgn.distToDropoff = distFromLastStopToDropoff;
                    assert(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);
                    asgn.dropoffStopIdx = numStops - 1;

                    const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(
                            vehId);

                    int curPickupIndex = numStops - 1;
                    auto pickupIt = relevantPickupsInRevOrder.begin();
                    for (; pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                        const auto &entry = *pickupIt;

                        if (entry.stopIndex < curPickupIndex) {
                            // New smaller pickup index reached: Check if seating capacity and cost lower bound admit
                            // any valid assignments at this or earlier indices.
                            if (occupancies[entry.stopIndex] >= asgn.vehicle->capacity)
                                break;

                            assert(entry.stopIndex < numStops - 1);
                            const auto minTripTimeToLastStop = routeState.schedDepTimesFor(vehId)[numStops - 1] -
                                                               routeState.schedArrTimesFor(vehId)[entry.stopIndex + 1];

                            const auto minCostFromHere = calculator.calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(
                                    asgn.dropoff->walkingDist, distFromLastStopToDropoff, minTripTimeToLastStop,
                                    requestState);
                            if (minCostFromHere > requestState.getBestCost())
                                break;

                            curPickupIndex = entry.stopIndex;
                        }


                        asgn.pickup = &requestState.pickups[entry.pdId];
                        if (asgn.pickup->loc == asgn.dropoff->loc)
                            continue;

                        asgn.pickupStopIdx = entry.stopIndex;
                        asgn.distToPickup = entry.distToPDLoc;
                        asgn.distFromPickup = entry.distFromPDLocToNextStop;

                        const int initialPickupDetour = calcInitialPickupDetour(asgn, requestState, routeState,
                                                                                inputConfig);
                        const int residualDetourAtEnd = calcResidualPickupDetour(vehId, asgn.pickupStopIdx, numStops - 1,
                                                                                 initialPickupDetour, routeState);
                        if (!isServiceTimeConstraintViolated(fleet[vehId], requestState, residualDetourAtEnd,
                                                             routeState)) {
                            ++numAssignmentsTried;
                            requestState.tryAssignment(asgn);
                        } else {
                            // In the unlikely case that the assignment breaks the service time constraint of this
                            // vehicle, mark this combination of pareto best dropoff label and ordinary pickup label as
                            // a constraint breaker. Later, constraint breakers are filtered again and if any remain
                            // for this vehicle, the distance from this vehicle's last stop to every dropoff needs to
                            // be computed explicitly.
                            constraintBreakers.push_back(asgn);
                        }
                    }

                    if (pickupIt == relevantPickupsInRevOrder.end()) {
                        // If the reverse scan of the vehicle route did not break early at a later stop, then we also
                        // need to consider the pickup before next stop case.
                        checkPBNSForVehicle.set(vehId);
                    }
                }
            }

            filterConstraintBreakersBasedOnCost();
            if (!constraintBreakers.empty()) {
                closestDropoffSearch.run(requestState.dropoffs);
                ranClosestDropoffSearch = true;
                filterConstraintBreakersBasedOnDetour();

                // For every remaining constraint breaker, we know that the cost ignoring the constraint is better than
                // the best known cost and that there is a dropoff for which the constraint is held.
                // Therefore, we have to try all dropoffs to see if there is one for which both of those aspects are true.
                evaluateConstraintBreakersWithAllDropoffs(numAssignmentsTried, numFallBackChSearches);
            }
        }

        void
        enumerateAssignmentsWithPBNS(int &numAssignmentsTried, int &, int &numFallBackChSearches,
                                     bool &ranClosestDropoffSearch) {
            using namespace time_utils;

            Assignment asgn;
            asgn.pickupStopIdx = 0;

            // Try pickups before next assignments for all vehicles where the reverse scan for pickups did not end at a
            // later vertex.
            struct PairWithPickupBeforeNextStopLeftToCheck {
                int pickupId;
                int dropoffId;
                int distFromPickup;
                int distToDropoff;
            };
            std::vector<PairWithPickupBeforeNextStopLeftToCheck> leftToCheck;

            for (const auto &vehId: minCostSearch.getVehiclesSeen()) {

                if (!checkPBNSForVehicle.isSet(vehId))
                    continue;

                if (!relevantPickupsBeforeNextStop.hasRelevantSpotsFor(vehId)
                    || routeState.occupanciesFor(vehId)[0] >= fleet[vehId].capacity) {
                    continue;
                }

                leftToCheck.clear();
                const auto &numStops = routeState.numStopsOf(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;

                // First, find out for which pickups we need the exact distance from the vehicle's
                // current location to the pickup. Filter using lower bound distance to pickup.
                for (const auto &label: minCostSearch.getParetoBestDropoffLabelsFor(vehId)) {
                    asgn.dropoff = &requestState.dropoffs[label.dropoffId];
                    const auto &distFromLastStopToDropoff = label.distToDropoff;
                    asgn.distToDropoff = distFromLastStopToDropoff;


                    // Labels are ordered by their dropoff side cost so if dropoff side cost is worse than best known
                    // cost for this vehicle, then cost of any PBNS assignment will be worse for this and all remaining
                    // labels.
                    if (calculator.calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(
                            distFromLastStopToDropoff, *asgn.dropoff, requestState) > requestState.getBestCost())
                        break;  // no need to check pickup before next stop


                    // Compute a lower bound for the cost of any assignment with pickup before next stop
                    // based on the trip time starting at stop 1 and the detour starting at the last stop.
                    // If this is already worse than the best known cost, we don't have to consider the PBNS case for
                    // this label.
                    assert(numStops > 1);
                    const auto minTripTimeToLastStop = routeState.schedDepTimesFor(vehId)[numStops - 1] -
                                                       routeState.schedArrTimesFor(vehId)[1];
                    const auto minCostFromHere = calculator.calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(
                            asgn.dropoff->walkingDist, distFromLastStopToDropoff, minTripTimeToLastStop, requestState);

                    if (minCostFromHere > requestState.getBestCost()) {
                        continue;
                    }


                    // If necessary, add all the pickups relevant before the next stop as uncertain assignments.
                    for (const auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {
                        asgn.pickup = &requestState.pickups[entry.pdId];
                        if (asgn.pickup->loc == asgn.dropoff->loc)
                            continue;

                        if (!curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup->id)) {
                            // If we do not know the exact distance from the vehicle's current location to the pickup, use
                            // the known lower bound distance first to compute a cost lower bound.
                            asgn.distFromPickup = entry.distFromPDLocToNextStop;
                            asgn.distToPickup = entry.distToPDLoc;
                            const auto lowerBoundCost = calculator.calcWithoutHardConstraints(asgn, requestState);
                            // If the cost lower bound is worse than the best known cost, this pickup/dropoff
                            // combination is not relevant.
                            if (lowerBoundCost > requestState.getBestCost())
                                continue;

                            // Otherwise, calculate the exact distance from the vehicle's location to the pickup.
                            // (computation of exact distances bundled for vehicle later)
                            curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup->id, entry.distToPDLoc);
                        }
                        leftToCheck.push_back(
                                {asgn.pickup->id, asgn.dropoff->id, entry.distFromPDLocToNextStop,
                                 distFromLastStopToDropoff});
                    }
                }

                // Bundled computation of required exact distances from vehicle to pickups.
                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId]);

                // Check pairs of pickups for which we now know exact distance and dropoffs. If a pair holds the service
                // time constraint, try the assignment. Otherwise, mark the pair as a constraint breaker.
                constraintBreakers.clear();
                for (const auto &pair: leftToCheck) {
                    asgn.pickup = &requestState.pickups[pair.pickupId];
                    asgn.dropoff = &requestState.dropoffs[pair.dropoffId];
                    assert(curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup->id));
                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup->id);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    asgn.distFromPickup = pair.distFromPickup;
                    asgn.distToDropoff = pair.distToDropoff;

                    const int initialPickupDetour = calcInitialPickupDetour(asgn, requestState, routeState,
                                                                            inputConfig);
                    const int residualDetourAtEnd = calcResidualPickupDetour(vehId, asgn.pickupStopIdx, numStops - 1,
                                                                             initialPickupDetour, routeState);
                    if (!isServiceTimeConstraintViolated(fleet[vehId], requestState, residualDetourAtEnd,
                                                         routeState)) {
                        ++numAssignmentsTried;
                        requestState.tryAssignment(asgn);
                    } else {
                        constraintBreakers.push_back(asgn);
                    }
                }
            }

            filterConstraintBreakersBasedOnCost();
            if (!constraintBreakers.empty()) {
                if (!ranClosestDropoffSearch) {
                    closestDropoffSearch.run(requestState.dropoffs);
                    ranClosestDropoffSearch = true;
                }
                filterConstraintBreakersBasedOnDetour();

                // For every remaining constraint breaker, we know that the cost ignoring the constraint is better than
                // the best known cost and that there is a dropoff for which the constraint is held.
                // Therefore, we have to try all dropoffs to see if there is one for which both of those aspects are true.
                evaluateConstraintBreakersWithAllDropoffs(numAssignmentsTried, numFallBackChSearches);
            }
        }


        // Filter constraint breakers based on cost. Current best known cost has been updated to the best possible cost
        // including the assignments with pareto best dropoffs after last stops that do not break the service time
        // constraint. We now filter out the constraint breakers that lead to cost worse than that.
        // Constraint breakers have to be given ordered by vehicles.
        // We maintain the order of constraint breakers.
        void filterConstraintBreakersBasedOnCost() {
            int cur = 0;
            int nextGoodOffset = 0;
            while (cur + nextGoodOffset < constraintBreakers.size()) {
                // Keep bad entries at tail of the entries already considered
                std::swap(constraintBreakers[cur], constraintBreakers[cur + nextGoodOffset]);

                const auto &constraintBreaker = constraintBreakers[cur];
                const auto lowerBoundCost = calculator.calcWithoutHardConstraints(constraintBreaker, requestState);
                if (lowerBoundCost > requestState.getBestCost()) {
                    // Lower bound is worse than best known => actual cost is worse than best known => remove
                    ++nextGoodOffset;
                } else {
                    ++cur;
                }
            }
            constraintBreakers.resize(constraintBreakers.size() - nextGoodOffset);
        }

        // Filter constraint breakers based on minimum detour. For each last stop, find out the minimum distance from
        // the last stop to any dropoff. For every remaining constraint breaker, check if the detour of its pickup
        // label plus the minimum detour with any dropoff can hold the service time constraint. If not, we can filter
        // it out.
        // Constraint breakers have to be given ordered by vehicles.
        // We maintain the order of constraint breakers.
        void filterConstraintBreakersBasedOnDetour() {
            using namespace time_utils;
            int cur = 0;
            int nextGoodOffset = 0;
            while (cur + nextGoodOffset < constraintBreakers.size()) {
                // Keep bad entries at tail of the entries already considered
                std::swap(constraintBreakers[cur], constraintBreakers[cur + nextGoodOffset]);

                const auto &constraintBreaker = constraintBreakers[cur];
                const auto &vehId = constraintBreaker.vehicle->vehicleId;

                const auto lengthOfPickupLeg = calcLengthOfLegStartingAt(constraintBreaker.pickupStopIdx,
                                                                         vehId, routeState);
                const auto totalDetour =
                        constraintBreaker.distToPickup + inputConfig.stopTime + constraintBreaker.distFromPickup -
                        lengthOfPickupLeg + closestDropoffSearch.getDistToClosestPDLocFromVeh(vehId) +
                        inputConfig.stopTime;

                if (isServiceTimeConstraintViolated(fleet[vehId], requestState, totalDetour, routeState)) {
                    // Constraint cannot be held even with minimal distance to any dropoff, so breaker is not relevant
                    ++nextGoodOffset;
                } else {
                    ++cur;
                }
            }
            constraintBreakers.resize(constraintBreakers.size() - nextGoodOffset);
        }

        // Evaluates constraint breakers, i.e. combinations of vehicle, pickup and pickup assignment point for which a cost
        // lower bound with some pareto best dropoff is better than the best known cost but that break the service time
        // constraint with that dropoff.
        // For those combinations, we have to check whether an assignment with a different dropoff exists that
        // is better than the best known assignment and holds the service time constraint.
        // Computes distances from last stop of each affected vehicle to each dropoff and tries the assignments of each
        // constraint breaker with each dropoff.
        // Constraint breakers have to be given ordered by vehicles.
        void evaluateConstraintBreakersWithAllDropoffs(int &numAssignmentsTried, int &numFallbackChSearchesRun) {

            int lastVehId = constraintBreakers.empty() ? INVALID_ID : constraintBreakers[0].vehicle->vehicleId;
            int startOfLastVehId = 0;
            for (int i = 0; i < constraintBreakers.size(); ++i) {
                if (constraintBreakers[i].vehicle->vehicleId != lastVehId) {
                    distsFromLastStopToDropoffs.clear();

                    // We already know some distances: The distances to pareto best dropoffs and the distance to the
                    // closest dropoff.
                    for (const auto &label: minCostSearch.getParetoBestDropoffLabelsFor(lastVehId)) {
                        distsFromLastStopToDropoffs[label.dropoffId] = label.distToDropoff;
                    }
                    distsFromLastStopToDropoffs[closestDropoffSearch.getIdOfSpotClosestToVeh(
                            lastVehId)] = closestDropoffSearch.getDistToClosestPDLocFromVeh(lastVehId);

                    // Calculate the rest of the distances.
                    numFallbackChSearchesRun += computeDistancesFromLastStopToAllDropoffs(lastVehId,
                                                                                          distsFromLastStopToDropoffs);

                    // Explicitly evaluate all assignments using a constraint breaker of this vehicle and any dropoff.
                    for (int j = startOfLastVehId; j < i; ++j) {
                        auto asgn = constraintBreakers[j];
                        assert(asgn.vehicle->vehicleId == lastVehId);
                        assert(asgn.dropoffStopIdx == routeState.numStopsOf(lastVehId) - 1);

                        for (const auto &dropoff: requestState.dropoffs) {
                            asgn.dropoff = &dropoff;
                            if (asgn.pickup->loc == asgn.dropoff->loc) continue;

                            asgn.distToDropoff = distsFromLastStopToDropoffs[asgn.dropoff->id];
                            ++numAssignmentsTried;
                            requestState.tryAssignment(asgn);
                        }
                    }

                    startOfLastVehId = i;
                    lastVehId = constraintBreakers[i].vehicle->vehicleId;
                }
            }
        }

        // Computes distances from last stop of given vehicle to each dropoff and stores the results in distances.
        // Skips dropoffs for which the given distances vector already has a valid entry.
        // Returns number of CH searches run.
        int computeDistancesFromLastStopToAllDropoffs(const int vehId, TimestampedVector<int> &distances) {
            assert(distances.size() == requestState.numDropoffs());

            int numSearchesRun = 0;

            std::array<int, FallBackCHLabelSet::K> sources;
            std::array<int, FallBackCHLabelSet::K> targets;
            std::array<int, FallBackCHLabelSet::K> dropoffIds;
            int nextFreeDirectSearch = 0;

            const auto numStops = routeState.numStopsOf(vehId);
            const auto lastStopLoc = routeState.stopLocationsFor(vehId)[numStops - 1];
            sources.fill(ch.rank(inputGraph.edgeHead(lastStopLoc)));

            // Compute the distance from the last stop to all other dropoffs
            for (const auto &dropoff: requestState.dropoffs) {
                if (distances.hasValidValue(dropoff.id))
                    continue;

                targets[nextFreeDirectSearch] = ch.rank(inputGraph.edgeTail(dropoff.loc));
                dropoffIds[nextFreeDirectSearch] = dropoff.id;

                if (++nextFreeDirectSearch == FallBackCHLabelSet::K) {
                    // Run K direct searches
                    fullCHQuery.run(sources, targets);
                    ++numSearchesRun;
                    for (int i = 0; i < FallBackCHLabelSet::K; ++i) {
                        const auto targetOffset = inputGraph.travelTime(requestState.dropoffs[dropoffIds[i]].loc);
                        distances[dropoffIds[i]] = fullCHQuery.getDistance(i) + targetOffset;
                    }
                    nextFreeDirectSearch = 0;
                }
            }

            if (nextFreeDirectSearch != 0) {
                // Run leftover direct searches
                const auto numSearchesLeft = nextFreeDirectSearch;
                for (; nextFreeDirectSearch < FallBackCHLabelSet::K; ++nextFreeDirectSearch) {
                    // Fill up targets with duplicates of first target
                    targets[nextFreeDirectSearch] = targets[0];
                }
                fullCHQuery.run(sources, targets);
                ++numSearchesRun;
                for (int i = 0; i < numSearchesLeft; ++i) {
                    const auto targetOffset = inputGraph.travelTime(
                            requestState.dropoffs[dropoffIds[i]].loc);
                    distances[dropoffIds[i]] = fullCHQuery.getDistance(i) + targetOffset;
                }
            }

            return numSearchesRun;
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const RouteStateT &routeState;
        const CostCalculatorT &calculator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        ClosestDropoffToLastStopQuery closestDropoffSearch;
        const CH &ch;
        RequestState<CostCalculatorT> &requestState;
        const RelevantPDLocs &relevantOrdinaryPickups;
        const RelevantPDLocs &relevantPickupsBeforeNextStop;

        const InputConfig &inputConfig;
        IsVehEligibleForDropoffAfterLastStop isVehEligibleForDropoffAfterLastStop;
        MinCostLabelSearch minCostSearch;
        TimestampedVector<int> distsFromLastStopToDropoffs;

        // DALS assignments that break service time constraint and may lead to having to compute distances from last
        // stop to all dropoffs.
        std::vector<Assignment> constraintBreakers;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle
        FastResetFlagArray<> checkPBNSForVehicle;

        typename CHEnvT::template FullCHQuery<FallBackCHLabelSet> fullCHQuery;

    };

}