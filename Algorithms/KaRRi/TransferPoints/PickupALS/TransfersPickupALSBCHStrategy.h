/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h"
#include "Tools/Timer.h"

namespace karri::Transfers {

    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsEnvT, typename LabelSetT>
    class TransfersPickupALSBCHStrategy {

        static constexpr int K = LabelSetT::K;
        using LabelMask = typename LabelSetT::LabelMask;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        struct PickupAfterLastStopPruner {

            static constexpr bool INCLUDE_IDLE_VEHICLES = true;

            PickupAfterLastStopPruner(TransfersPickupALSBCHStrategy &strat, const CostCalculator &calc)
                    : strat(strat), calc(calc) {}

            // Returns whether a given distance from a vehicle's last stop to the pickup cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop distance greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesDistanceNotAdmitBestAsgn(const DistanceLabel &distancesToPickups,
                                                   const bool considerPickupWalkingDists) const {
                KASSERT(strat.requestState.minDirectPDDist < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToPickups[i] >= INFTY or
                    // minDirectDistances[i] >= INFTY are worse than the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const auto &walkingDists = considerPickupWalkingDists ? strat.currentPickupWalkingDists : 0;

                const auto detourTillDepAtPickup =
                        distancesToPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                auto tripTimeTillDepAtPickup = detourTillDepAtPickup;
                tripTimeTillDepAtPickup.max(walkingDists);
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedPickupAndTransferAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, strat.requestState.minDirectPDDist,
                        walkingDists, strat.requestState);

                costLowerBound.setIf(DistanceLabel(INFTY), ~(distancesToPickups < INFTY));

                return strat.upperBoundCost < costLowerBound;
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the pickup cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToPickups needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtPickups,
                                                  const DistanceLabel &minDistancesToPickups) const {
                KASSERT(strat.requestState.minDirectPDDist < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with arrTimesAtPickups[i] >= INFTY or
                    // minDistancesToPickups[i] >= INFTY are worse than the current best.
                    return ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY));
                }

                const auto detourTillDepAtPickup =
                        minDistancesToPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                auto depTimeAtPickup = arrTimesAtPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                const auto reqTime = DistanceLabel(strat.requestState.originalRequest.requestTime);
                depTimeAtPickup.max(reqTime);
                const auto tripTimeTillDepAtPickup = depTimeAtPickup - reqTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedPickupAndTransferAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, strat.requestState.minDirectPDDist,
                        strat.currentPickupWalkingDists, strat.requestState);

                costLowerBound.setIf(DistanceLabel(INFTY),
                                     ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY)));
                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask
            isWorseThanBestKnownVehicleDependent(const int vehId, const DistanceLabel &distancesToPickups) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const auto detourTillDepAtPickup = distancesToPickups + InputConfig::getInstance().stopTime;
                const auto &stopIdx = strat.routeState.numStopsOf(vehId) - 1;
                const int vehDepTimeAtLastStop = time_utils::getVehDepTimeAtStopForRequest(vehId, stopIdx,
                                                                                           strat.requestState,
                                                                                           strat.routeState);
                auto depTimeAtPickups = vehDepTimeAtLastStop + distancesToPickups + InputConfig::getInstance().stopTime;
                depTimeAtPickups.max(strat.curPassengerArrTimesAtPickups);
                const auto tripTimeTillDepAtPickup = depTimeAtPickups - strat.requestState.originalRequest.requestTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedPickupAndTransferAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, strat.requestState.minDirectPDDist,
                        strat.currentPickupWalkingDists,
                        strat.requestState);

                costLowerBound.setIf(INFTY, ~(distancesToPickups < INFTY));
                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int /* vehId */, const DistanceLabel & /*distancesToPickups*/) {
                return; // No update possible since actual transfer point is unknown
            }

            bool isVehicleEligible(const int &) const {
                return true; // All vehicles can perform PALS assignments
            }

        private:
            TransfersPickupALSBCHStrategy &strat;
            const CostCalculator &calc;
        };

        using PickupBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, PickupAfterLastStopPruner, LabelSetT>;

    public:

        TransfersPickupALSBCHStrategy(const InputGraphT &inputGraph,
                                      const Fleet &fleet,
                                      const CHEnvT &chEnv,
                                      const CostCalculator &calculator,
                                      const LastStopBucketsEnvT &lastStopBucketsEnv,
                                      const RouteState &routeState,
                                      RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  lastStopBucketsEnv(lastStopBucketsEnv),
                  routeState(routeState),
                  requestState(requestState),
                  distances(fleet.size()),
                  search(lastStopBucketsEnv, distances, chEnv, routeState, vehiclesSeenForPickups,
                         PickupAfterLastStopPruner(*this, calculator)),
                  vehiclesSeenForPickups(fleet.size()) {}

        int getDistanceToPickup(const int vehId, const unsigned int pickupId) {
            const int distance = distances.getDistance(vehId, pickupId);
            KASSERT(distance >= 0);
            return distance;
        }

        const LightweightSubset& findPickupsAfterLastStop(const PDDistances &pdDistances) {
            runBchSearches(pdDistances);
            filterVehiclesBasedOnParetoDominance();
            return vehiclesSeenForPickups;
        }

    private:

        // Run BCH searches that find distances from last stops to pickups
        void runBchSearches(const PDDistances &pdDistances) {
            // Timer timer;

            initPickupSearches();
            for (int i = 0; i < requestState.numPickups(); i += K)
                runSearchesForPickupBatch(i, pdDistances);
        }


        void initPickupSearches() {
            upperBoundCost = requestState.getBestCost();
            vehiclesSeenForPickups.clear();
            const int numPickupBatches = requestState.numPickups() / K + (requestState.numPickups() % K != 0);
            distances.init(numPickupBatches);
        }

        void runSearchesForPickupBatch(const int firstPickupId, const PDDistances &pdDistances) {
            KASSERT(firstPickupId % K == 0 && firstPickupId < requestState.numPickups());

            distances.setCurBatchIdx(firstPickupId / K);

            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < requestState.numPickups() ? requestState.pickups[firstPickupId + i]
                                                                      : requestState.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
                currentPickupWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup.id);
                curDistancesToDest[i] = pdDistances.getDirectDistance(pickup.id, 0);

                // Initialize distance to 0 for every last stop coinciding with the pickup
                const int head = inputGraph.edgeHead(pickup.loc);
                for (const auto &vehId: lastStopBucketsEnv.vehiclesWithLastStopAt(head)) {
                    const auto numStops = routeState.numStopsOf(vehId);
                    if (routeState.stopLocationsFor(vehId)[numStops - 1] != pickup.loc)
                        continue;
                    LabelMask pickupMask = false;
                    pickupMask.set(i, true);
                    distances.setDistancesForCurBatchIf(vehId, 0, pickupMask);
                    vehiclesSeenForPickups.insert(vehId);
                }
            }

            search.run(pickupTails, travelTimes);
        }


        bool dominates(const int vehId1, const int vehId2) {

            const int stopIdx1 = routeState.numStopsOf(vehId1) - 1;
            const int stopIdx2 = routeState.numStopsOf(vehId2) - 1;

            using namespace time_utils;
            using F = CostCalculator::CostFunction;
            for (const auto &pickup: requestState.pickups) {
                const auto distToPickup1 = getDistanceToPickup(vehId1, pickup.id);
                const auto distToPickup2 = getDistanceToPickup(vehId2, pickup.id);
                if (distToPickup1 == INFTY && distToPickup2 == INFTY)
                    continue; // Both vehicles cannot reach the pickup, so they are equal
                const auto depTimeAtPickup1 = getActualDepTimeAtPickup(vehId1, stopIdx1, distToPickup1, pickup,
                                                                       requestState, routeState);
                const auto depTimeAtPickup2 = getActualDepTimeAtPickup(vehId2, stopIdx2, distToPickup2, pickup,
                                                                       requestState, routeState);
                const auto vehTimeTillDepAtPickup1 =
                        depTimeAtPickup1 - getVehDepTimeAtStopForRequest(vehId1, stopIdx1, requestState, routeState);
                const auto vehTimeTillDepAtPickup2 =
                        depTimeAtPickup2 - getVehDepTimeAtStopForRequest(vehId2, stopIdx2, requestState, routeState);
                const auto psgTimeTillDepAtPickup1 = depTimeAtPickup1 - requestState.originalRequest.requestTime;
                const auto psgTimeTillDepAtPickup2 = depTimeAtPickup2 - requestState.originalRequest.requestTime;

                const auto detourDiff = vehTimeTillDepAtPickup1 - vehTimeTillDepAtPickup2;
                const auto waitVioDiff = F::calcWaitViolationCost(depTimeAtPickup1, requestState) -
                                         F::calcWaitViolationCost(depTimeAtPickup2, requestState);

                // Upper bound for how much smaller the trip time of vehId2 can be compared to vehId1.
                // We need to max with 0 since the dropoff vehicle may arrive at the transfer later than the pickup
                // vehicle, which may dominate the trip time, negating a possible advantage of vehId1.
                const auto maxTripTimeDiff = std::max(psgTimeTillDepAtPickup1 - psgTimeTillDepAtPickup2, 0);
                const auto maxTripVioDiff = F::TRIP_VIO_WEIGHT * maxTripTimeDiff;

                const auto maxCostDiff = F::VEH_WEIGHT * detourDiff +
                                         F::PSG_WEIGHT * maxTripTimeDiff +
                                         waitVioDiff + maxTripVioDiff;
                if (maxCostDiff >= 0)
                    return false; // vehId1 does not dominate vehId2 for this pickup
            }

            return true;
        }

        void filterVehiclesBasedOnParetoDominance() {
            if (vehiclesSeenForPickups.empty())
                return;

            std::vector<int> indicesToKeep;
            for (int i = 0; i < vehiclesSeenForPickups.size(); ++i) {
                const int vehId1 = *(vehiclesSeenForPickups.begin() + i);

                bool isDominated = false;
                for (const int idxOfVeh2: indicesToKeep) {
                    const int vehId2 = *(vehiclesSeenForPickups.begin() + idxOfVeh2);
                    if (dominates(vehId2, vehId1)) {
                        isDominated = true;
                        break; // vehId1 is dominated by vehId2
                    }
                }
                if (isDominated)
                    continue;

                // vehId1 is not dominated by any of the vehicles in indicesToKeep, so we keep it. Check if vehId1
                // dominates any of the vehicles in indicesToKeep and if so remove them
                indicesToKeep.push_back(i);
                for (int j = 0; j < indicesToKeep.size();) {
                    const int vehId2 = *(vehiclesSeenForPickups.begin() + indicesToKeep[j]);
                    if (dominates(vehId1, vehId2)) {
                        std::swap(indicesToKeep[j], indicesToKeep.back());
                        indicesToKeep.pop_back();
                        continue;
                    }
                    ++j;
                }
            }

            // Remove all vehicles that are not in indicesToKeep from vehiclesSeenForPickups
            for (auto &idx: indicesToKeep) {
                idx = *(vehiclesSeenForPickups.begin() + idx);
            }
            vehiclesSeenForPickups.clear();
            for (const auto &idx: indicesToKeep) {
                vehiclesSeenForPickups.insert(idx);
            }

        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const LastStopBucketsEnvT &lastStopBucketsEnv;
        const RouteState &routeState;
        RequestState &requestState;

        int upperBoundCost;

        TentativeLastStopDistances <LabelSetT> distances;
        PickupBCHQuery search;

        // Vehicles seen by any last stop pickup search
        LightweightSubset vehiclesSeenForPickups;
        DistanceLabel currentPickupWalkingDists;
        DistanceLabel curPassengerArrTimesAtPickups;
        DistanceLabel curDistancesToDest;

    };

}
