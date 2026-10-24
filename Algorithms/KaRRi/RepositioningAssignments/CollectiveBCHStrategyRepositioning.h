/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <algorithm>
#include <vector>

#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Tools/Timer.h"

namespace karri {

    // Finds assignments using vehicles that are currently repositioning. Such a vehicle has no stops other than its
    // stop 0 (at which it started repositioning). If it is assigned a request, it is rerouted at its current location
    // to the pickup and then to the dropoff.
    //
    // Candidate vehicles are found using a collective BCH search from all pickups against the repositioning buckets,
    // which yields lower bounds on the distance from the current location of every repositioning vehicle to any
    // pickup. For candidate vehicles, the current location and the exact distances to the pickups are computed.
    template<typename InputGraphT, typename CHEnvT, typename RepositioningBucketContainerT, typename VehicleLocatorT,
            typename CurVehLocToPickupSearchesT, typename PDDistancesT>
    class CollectiveBCHStrategyRepositioning {

        using LabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;

        // Scans the repositioning bucket of a vertex settled by the reverse search from the pickups.
        struct ScanBucket {
            explicit ScanBucket(CollectiveBCHStrategyRepositioning &parent) : parent(parent) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContT &) {
                // distFromV[0] is the distance from v to the closest pickup.
                for (const auto &entry: parent.repositioningBuckets.getBucketOf(v)) {
                    ++parent.numEntriesScanned;
                    const int lbDistToPickup = entry.distToTarget + distFromV[0];
                    const int lowerBoundCost = parent.calculator.calcCostLowerBoundForRepositioningAssignment(
                            lbDistToPickup, parent.curMinDirectDist, *parent.curReqState);
                    // Entries are sorted by distance, so all further entries will be worse too.
                    if (lowerBoundCost > parent.curReqState->getBestCost())
                        break;
                    if (parent.candidateVehicles.insert(entry.targetId)) {
                        parent.minLowerBoundDistToPickup[entry.targetId] = lbDistToPickup;
                    } else {
                        auto &minDist = parent.minLowerBoundDistToPickup[entry.targetId];
                        minDist = std::min(minDist, lbDistToPickup);
                    }
                }
                return false;
            }

            CollectiveBCHStrategyRepositioning &parent;
        };

        // Stops the reverse search once the lower bound cost for any vehicle found from here on is worse than the
        // best known cost.
        struct StopWhenLowerBoundExceedsBestCost {
            explicit StopWhenLowerBoundExceedsBestCost(CollectiveBCHStrategyRepositioning &parent) : parent(parent) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distFromV, const DistLabelContT &) {
                const bool stop = parent.calculator.calcCostLowerBoundForRepositioningAssignment(
                        distFromV[0], parent.curMinDirectDist, *parent.curReqState) > parent.curReqState->getBestCost();
                if (stop)
                    parent.reverseSearchStopDist = distFromV[0];
                return stop;
            }

            CollectiveBCHStrategyRepositioning &parent;
        };

        using ReverseSearch = typename CHEnvT::template UpwardSearch<ScanBucket, StopWhenLowerBoundExceedsBestCost, LabelSet>;

        // Forward CH search from the current location of a vehicle that meets the reverse search from the pickups
        // (that has been run for the BCH search) to obtain a lower bound on the distance to any pickup.
        struct MeetReverseSearch {
            explicit MeetReverseSearch(CollectiveBCHStrategyRepositioning &parent) : parent(parent) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                const int distFromV = parent.reverseSearch.getDistance(v);
                if (distFromV < INFTY)
                    parent.minMeetingDist = std::min(parent.minMeetingDist, distToV[0] + distFromV);
                return false;
            }

            CollectiveBCHStrategyRepositioning &parent;
        };

        struct StopWhenMeetingDistCannotImprove {
            explicit StopWhenMeetingDistCannotImprove(CollectiveBCHStrategyRepositioning &parent) : parent(parent) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                return distToV[0] >= parent.minMeetingDist;
            }

            CollectiveBCHStrategyRepositioning &parent;
        };

        using ForwardSearch = typename CHEnvT::template UpwardSearch<MeetReverseSearch, StopWhenMeetingDistCannotImprove, LabelSet>;

    public:

        CollectiveBCHStrategyRepositioning(const InputGraphT &inputGraph,
                                           const Fleet &fleet,
                                           const CHEnvT &chEnv,
                                           const RepositioningBucketContainerT &repositioningBuckets,
                                           const RouteState &routeState,
                                           VehicleLocatorT &vehicleLocator,
                                           CurVehLocToPickupSearchesT &curVehLocToPickupSearches)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  ch(chEnv.getCH()),
                  calculator(routeState),
                  repositioningBuckets(repositioningBuckets),
                  routeState(routeState),
                  vehicleLocator(vehicleLocator),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  candidateVehicles(static_cast<int>(fleet.size())),
                  minLowerBoundDistToPickup(fleet.size(), INFTY),
                  curReqState(nullptr),
                  curMinDirectDist(INFTY),
                  numEntriesScanned(0),
                  reverseSearchStopDist(INFTY),
                  reverseSearch(chEnv.template getReverseSearch<ScanBucket, StopWhenLowerBoundExceedsBestCost, LabelSet>(
                          ScanBucket(*this), StopWhenLowerBoundExceedsBestCost(*this))),
                  minMeetingDist(INFTY),
                  forwardSearch(chEnv.template getForwardSearch<MeetReverseSearch, StopWhenMeetingDistCannotImprove, LabelSet>(
                          MeetReverseSearch(*this), StopWhenMeetingDistCannotImprove(*this))) {}

        CollectiveBCHStrategyRepositioning(const CollectiveBCHStrategyRepositioning &) = delete;

        CollectiveBCHStrategyRepositioning(CollectiveBCHStrategyRepositioning &&) = delete;

        void init(const RequestState &, const PDLocs &pdLocs, stats::RepositioningAssignmentsPerformanceStats &stats) {
            Timer timer;
            pickupRanks.resize(pdLocs.numPickups());
            pickupOffsets.resize(pdLocs.numPickups());
            for (int pId = 0; pId < pdLocs.numPickups(); ++pId) {
                const auto &p = pdLocs.pickups[pId];
                pickupRanks[pId] = ch.rank(inputGraph.edgeTail(p.loc));
                pickupOffsets[pId] = inputGraph.travelTime(p.loc);
            }
            stats.initializationTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void tryRepositioningAssignments(RequestState &requestState, const PDDistancesT &pdDistances,
                                         const PDLocs &pdLocs,
                                         stats::RepositioningAssignmentsPerformanceStats &stats) {
            // If no vehicle is repositioning, there is nothing to do.
            if (routeState.numVehiclesRepositioning() == 0)
                return;

            Timer timer;
            curReqState = &requestState;
            curMinDirectDist = pdDistances.getMinDirectDistance();
            candidateVehicles.clear();
            numEntriesScanned = 0;

            // Run collective BCH search from all pickups to find candidate vehicles for repositioning assignments.
            reverseSearchStopDist = INFTY;
            reverseSearch.runWithMultipleRoots(pickupRanks, pickupOffsets);

            stats.searchTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.numEntriesScanned += numEntriesScanned;
            stats.numVerticesOrLabelsSettled += reverseSearch.getNumVerticesSettled();
            stats.numEdgeRelaxationsInSearchGraph += reverseSearch.getNumEdgeRelaxations();
            stats.numRepositioningVehiclesTotal += routeState.numVehiclesRepositioning();
            stats.numVehiclesAfterPathBCH += candidateVehicles.size();

            // For each candidate vehicle, compute exact location and exact distances to pickups, then try assignments.
            tryAssignmentsForCandidateVehicles(requestState, pdDistances, pdLocs, stats);

            curReqState = nullptr;
        }

    private:

        void tryAssignmentsForCandidateVehicles(RequestState &requestState, const PDDistancesT &pdDistances,
                                                const PDLocs &pdLocs,
                                                stats::RepositioningAssignmentsPerformanceStats &stats) {
            const int64_t chSearchTimeBefore = curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
            const int64_t numChSearchesBefore = curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest();
            int64_t locatingTime = 0;
            int64_t lowerBoundQueryTime = 0;

            Timer totalTimer;
            Timer timer;
            Assignment asgn;
            asgn.pickupStopIdx = 0;
            asgn.dropoffStopIdx = 0;
            asgn.distFromPickup = 0;
            asgn.distFromDropoff = 0;
            const int now = requestState.dispatchingTime;
            int numAssignmentsTried = 0;

            // Process candidate vehicles in order of the lower bound distance to any pickup found by the BCH search.
            // Once the lower bound cost for a vehicle is worse than the best known cost, the same holds for all
            // remaining vehicles.
            sortedCandidates.clear();
            for (const auto vehId: candidateVehicles)
                sortedCandidates.push_back({minLowerBoundDistToPickup[vehId], vehId});
            std::sort(sortedCandidates.begin(), sortedCandidates.end());

            for (const auto &[lbDistToPickup, vehId]: sortedCandidates) {
                KASSERT(routeState.isRepositioning(vehId));
                if (calculator.calcCostLowerBoundForRepositioningAssignment(
                        lbDistToPickup, pdDistances.getMinDirectDistance(), requestState) > requestState.getBestCost())
                    break;

                const auto &veh = fleet[vehId];
                asgn.vehicle = &veh;

                // Compute current vehicle location
                timer.restart();
                const auto vehLocation = vehicleLocator.computeCurrentLocation(veh, now);
                locatingTime += timer.elapsed<std::chrono::nanoseconds>();
                KASSERT(vehLocation.depTimeAtHead >= now);
                const int distToHeadOfCurEdge = vehLocation.depTimeAtHead - now;

                // Compute lower bound cost for this vehicle using minimum distance from vehicle location to any
                // pickup obtained with a one-to-any CH query. The reverse part of the query from all pickups is the
                // reverse search run for the BCH search. If that search has been stopped early, vertices that it has
                // not settled have a distance of at least the distance at which it was stopped.
                timer.restart();
                int minDistToPickup;
                if (std::any_of(pdLocs.pickups.begin(), pdLocs.pickups.end(),
                                [&](const PDLoc &p) { return p.loc == vehLocation.location; })) {
                    minDistToPickup = distToHeadOfCurEdge;
                } else {
                    minMeetingDist = reverseSearchStopDist;
                    forwardSearch.run(ch.rank(inputGraph.edgeHead(vehLocation.location)));
                    minDistToPickup = minMeetingDist >= INFTY ? INFTY : distToHeadOfCurEdge + minMeetingDist;
                }
                lowerBoundQueryTime += timer.elapsed<std::chrono::nanoseconds>();
                const int minCost = calculator.calcCostLowerBoundForRepositioningAssignment(
                        minDistToPickup, pdDistances.getMinDirectDistance(), requestState);
                if (minCost > requestState.getBestCost())
                    continue;

                ++stats.numVehiclesAfterOneToAny;

                // Compute exact distances from current vehicle location to all pickups
                for (const auto &p: pdLocs.pickups)
                    curVehLocToPickupSearches.addPickupForProcessing(p.id, INFTY);
                curVehLocToPickupSearches.computeExactDistancesForRepositioningVehicle(veh, vehLocation, pdLocs);

                // Try all assignments for this vehicle and all pickup-dropoff pairs
                for (const auto &p: pdLocs.pickups) {
                    asgn.pickup = p;
                    KASSERT(curVehLocToPickupSearches.knowsDistance(vehId, p.id));
                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, p.id);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    const int minCostForP = calculator.calcCostLowerBoundForRepositioningAssignment(
                            asgn.distToPickup, pdDistances.getMinDirectDistanceForPickup(p.id), requestState);
                    if (minCostForP > requestState.getBestCost())
                        continue;

                    for (const auto &d: pdLocs.dropoffs) {
                        asgn.dropoff = d;
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                        requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        ++numAssignmentsTried;
                    }
                }
            }

            const int64_t chSearchTime =
                    curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() - chSearchTimeBefore;
            const int64_t totalTime = totalTimer.elapsed<std::chrono::nanoseconds>();
            stats.locatingVehiclesTime += locatingTime;
            stats.directCHSearchTime += chSearchTime + lowerBoundQueryTime;
            stats.numCHSearches += curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest() -
                                   numChSearchesBefore;
            stats.tryAssignmentsTime += totalTime - locatingTime - chSearchTime - lowerBoundQueryTime;
            stats.numAssignmentsTried += numAssignmentsTried;
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CH &ch;
        CostCalculator calculator;
        const RepositioningBucketContainerT &repositioningBuckets;
        const RouteState &routeState;
        VehicleLocatorT &vehicleLocator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;

        LightweightSubset candidateVehicles;
        std::vector<int> minLowerBoundDistToPickup;
        std::vector<std::pair<int, int>> sortedCandidates;

        // Context of current request (accessed by callbacks of reverse search)
        const RequestState *curReqState;
        int curMinDirectDist;
        int64_t numEntriesScanned;

        std::vector<int> pickupRanks;
        std::vector<int> pickupOffsets;

        int reverseSearchStopDist;
        ReverseSearch reverseSearch;

        int minMeetingDist;
        ForwardSearch forwardSearch;
    };
}
