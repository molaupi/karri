/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <stack>

#include "Algorithms/KaRRi/RouteState/RouteStateData.h"

#include "Tools/Constants.h"
#include "Tools/custom_assertion_levels.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Containers/BitVector.h"

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "StopIdManager.h"

namespace karri {

// Represents the state of all vehicle routes including the stop locations and schedules.
    class RouteStateUpdater {

    public:
        RouteStateUpdater() {}

        template<typename RequestStateT>
        std::pair<int, int>
        insertAssignment(RouteStateData &data, const Assignment &asgn, const RequestStateT &requestState) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &pickup = *asgn.pickup;
            const auto &dropoff = *asgn.dropoff;
            const int now = requestState.originalRequest.requestTime;
            const int numRiders = requestState.originalRequest.numRiders;
            const auto &start = data.pos[vehId].start;
            const auto &end = data.pos[vehId].end;
            auto pickupIndex = asgn.pickupStopIdx;
            auto dropoffIndex = asgn.dropoffStopIdx;

            KASSERT(pickupIndex >= 0);
            KASSERT(pickupIndex < end - start);
            KASSERT(dropoffIndex >= 0);
            KASSERT(dropoffIndex < end - start);

            bool pickupInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;

            if ((pickupIndex > 0 || data.schedDepTimes[start] > now) &&
                pickup.loc == data.stopLocations[start + pickupIndex]) {
                KASSERT(start + pickupIndex == end - 1 || pickupIndex == dropoffIndex ||
                        asgn.distFromPickup ==
                        data.schedArrTimes[start + pickupIndex + 1] - data.schedDepTimes[start + pickupIndex]);

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                data.schedDepTimes[start + pickupIndex] = std::max(data.schedDepTimes[start + pickupIndex],
                                                                   requestState.getPassengerArrAtPickup(pickup.id));

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime =
                        std::max(requestState.getMaxDepTimeAtPickup(), requestState.getPassengerArrAtPickup(pickup.id));
                data.maxArrTimes[start + pickupIndex] = std::min(data.maxArrTimes[start + pickupIndex],
                                                                 psgMaxDepTime - data.stopTime);
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                data.schedDepTimes[end - 1] = std::max(data.schedDepTimes[end - 1],
                                                       requestState.originalRequest.requestTime);
                data.schedArrTimes[end - 1] = data.schedDepTimes[end - 1] - data.stopTime;
                ++pickupIndex;
                ++dropoffIndex;
                const int unusedStopId = StopIdManager::getUnusedStopId();
                stableInsertion(vehId, pickupIndex, unusedStopId, data.pos, data.stopIds,
                                data.stopLocations,
                                data.schedArrTimes,
                                data.schedDepTimes,
                                data.vehWaitTimesPrefixSum,
                                data.maxArrTimes,
                                data.occupancies,
                                data.numDropoffsPrefixSum,
                                data.vehWaitTimesUntilDropoffsPrefixSum);
                data.stopLocations[start + pickupIndex] = pickup.loc;
                data.schedArrTimes[start + pickupIndex] =
                        data.schedDepTimes[start + pickupIndex - 1] + asgn.distToPickup;
                data.schedDepTimes[start + pickupIndex] = std::max(
                        data.schedArrTimes[start + pickupIndex] + data.stopTime,
                        requestState.getPassengerArrAtPickup(pickup.id));
                data.maxArrTimes[start + pickupIndex] = requestState.getMaxDepTimeAtPickup() - data.stopTime;
                data.occupancies[start + pickupIndex] = data.occupancies[start + pickupIndex - 1];
                data.numDropoffsPrefixSum[start + pickupIndex] = data.numDropoffsPrefixSum[start + pickupIndex - 1];
                pickupInsertedAsNewStop = true;
            }

            if (pickupIndex != dropoffIndex) {
                // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
                KASSERT(asgn.distFromPickup > 0);
                data.propagateSchedArrAndDepForward(start + pickupIndex + 1, start + dropoffIndex, asgn.distFromPickup);
            }

            if (pickup.loc != dropoff.loc && dropoff.loc == data.stopLocations[start + dropoffIndex]) {
                data.maxArrTimes[start + dropoffIndex] = std::min(data.maxArrTimes[start + dropoffIndex],
                                                                  requestState.getMaxArrTimeAtDropoff(pickup.id,
                                                                                                      dropoff.id));
            } else {
                ++dropoffIndex;
                const int unusedStopId = StopIdManager::getUnusedStopId();
                stableInsertion(vehId, dropoffIndex, unusedStopId, data.pos, data.stopIds,
                                data.stopLocations,
                                data.schedArrTimes,
                                data.schedDepTimes,
                                data.vehWaitTimesPrefixSum,
                                data.maxArrTimes,
                                data.occupancies,
                                data.numDropoffsPrefixSum,
                                data.vehWaitTimesUntilDropoffsPrefixSum);
                data.stopLocations[start + dropoffIndex] = dropoff.loc;
                data.schedArrTimes[start + dropoffIndex] =
                        data.schedDepTimes[start + dropoffIndex - 1] + asgn.distToDropoff;
                data.schedDepTimes[start + dropoffIndex] = data.schedArrTimes[start + dropoffIndex] + data.stopTime;
                // compare maxVehArrTime to next stop later
                data.maxArrTimes[start + dropoffIndex] = requestState.getMaxArrTimeAtDropoff(pickup.id, dropoff.id);
                data.occupancies[start + dropoffIndex] = data.occupancies[start + dropoffIndex - 1];
                data.numDropoffsPrefixSum[start + dropoffIndex] = data.numDropoffsPrefixSum[start + dropoffIndex - 1];
                dropoffInsertedAsNewStop = true;
            }

            // Propagate updated scheduled arrival and departure times as well as latest permissible arrival times.
            if (start + dropoffIndex < end - 1) {
                // At this point minDepTimes[start + dropoffIndex] is correct. If dropoff has been inserted not as the last
                // stop, propagate the changes to minDep and minArr times forward until the last stop.
                data.propagateSchedArrAndDepForward(start + dropoffIndex + 1, end - 1, asgn.distFromDropoff);

                // If there are stops after the dropoff, consider them for propagating changes to the maxArrTimes
                data.propagateMaxArrTimeBackward(start + dropoffIndex, start + pickupIndex);
            } else {
                // If there are no stops after the dropoff, propagate maxArrTimes backwards not including dropoff
                data.propagateMaxArrTimeBackward(start + dropoffIndex - 1, start + pickupIndex);
            }
            data.propagateMaxArrTimeBackward(start + pickupIndex - 1, start);

            // Update occupancies and prefix sums
            for (int idx = start + pickupIndex; idx < start + dropoffIndex; ++idx) {
                data.occupancies[idx] += numRiders;
                KASSERT(data.occupancies[idx] <= asgn.vehicle->capacity);
            }

            for (int idx = start + dropoffIndex; idx < end; ++idx) {
                ++data.numDropoffsPrefixSum[idx];
            }

            const int lastUnchangedPrefixSum =
                    pickupIndex > 0 ? data.vehWaitTimesPrefixSum[start + pickupIndex - 1] : 0;
            data.recalculateVehWaitTimesPrefixSum(start + pickupIndex, end - 1, lastUnchangedPrefixSum);
            const int lastUnchangedAtDropoffsPrefixSum =
                    pickupIndex > 0 ? data.vehWaitTimesUntilDropoffsPrefixSum[start + pickupIndex - 1] : 0;
            data.recalculateVehWaitTimesAtDropoffsPrefixSum(start + pickupIndex, end - 1,
                                                            lastUnchangedAtDropoffsPrefixSum);

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(data.stopIds[start + pickupIndex], data.stopIds[start + dropoffIndex]) + 1;
            if (data.stopIdToIdOfPrevStop.size() < newMinSize) {
                data.stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                data.stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                data.stopIdToLeeway.resize(newMinSize, 0);
                data.stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                data.rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                data.rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            KASSERT(start == data.pos[vehId].start && end == data.pos[vehId].end);
            if (pickupInsertedAsNewStop) {
                KASSERT(pickupIndex >= 1 && start + pickupIndex < end - 1);
                data.stopIdToVehicleId[data.stopIds[start + pickupIndex]] = vehId;
                data.stopIdToIdOfPrevStop[data.stopIds[start + pickupIndex]] = data.stopIds[start + pickupIndex - 1];
                data.stopIdToIdOfPrevStop[data.stopIds[start + pickupIndex + 1]] = data.stopIds[start + pickupIndex];
            }
            if (dropoffInsertedAsNewStop) {
                KASSERT(dropoffIndex > pickupIndex && start + dropoffIndex < end);
                data.stopIdToVehicleId[data.stopIds[start + dropoffIndex]] = vehId;
                data.stopIdToIdOfPrevStop[data.stopIds[start + dropoffIndex]] = data.stopIds[start + dropoffIndex - 1];
                if (start + dropoffIndex != end - 1)
                    data.stopIdToIdOfPrevStop[data.stopIds[start + dropoffIndex + 1]] = data.stopIds[start +
                                                                                                     dropoffIndex];
            }

            if (pickupInsertedAsNewStop || dropoffInsertedAsNewStop) {
                for (int i = start + pickupIndex; i < end; ++i) {
                    data.stopIdToPosition[data.stopIds[i]] = i - start;
                }
            }

            data.updateLeeways(vehId);
            data.updateMaxLegLength(vehId, pickupIndex, dropoffIndex);


            // Remember that request is picked up and dropped of at respective stops:
            insertion(data.stopIds[start + pickupIndex], requestState.originalRequest.requestId,
                      data.rangeOfRequestsPickedUpAtStop, data.requestsPickedUpAtStop);
            insertion(data.stopIds[start + dropoffIndex], requestState.originalRequest.requestId,
                      data.rangeOfRequestsDroppedOffAtStop, data.requestsDroppedOffAtStop);

            return {pickupIndex, dropoffIndex};
        }

        void removeStartOfCurrentLeg(RouteStateData &data, const int vehId) {
            KASSERT(vehId >= 0);
            KASSERT(vehId < data.pos.size());
            const auto &start = data.pos[vehId].start;
            KASSERT(data.pos[vehId].end - start > 0);
            const bool haveToRecomputeMaxLeeway = data.stopIds[start] == data.stopIdOfMaxLeeway;
            data.stopIdToVehicleId[data.stopIds[start]] = INVALID_ID;
            data.stopIdToLeeway[data.stopIds[start]] = 0;
            data.stopIdToPosition[data.stopIds[start]] = INVALID_INDEX;
            removalOfAllCols(data.stopIds[start], data.rangeOfRequestsPickedUpAtStop, data.requestsPickedUpAtStop);
            removalOfAllCols(data.stopIds[start], data.rangeOfRequestsDroppedOffAtStop, data.requestsDroppedOffAtStop);
            StopIdManager::markIdUnused(data.stopIds[start]);
            KASSERT(data.stopIdToIdOfPrevStop[data.stopIds[start]] == INVALID_ID);
            if (data.numStopsOf(vehId) > 1) {
                data.stopIdToIdOfPrevStop[data.stopIds[start + 1]] = INVALID_ID;
            }

            const auto numDropoffsAtStart = data.numDropoffsPrefixSum[start];
            stableRemoval(vehId, 0, data.pos, data.stopIds,
                          data.stopLocations,
                          data.schedArrTimes,
                          data.schedDepTimes,
                          data.vehWaitTimesPrefixSum,
                          data.maxArrTimes,
                          data.occupancies,
                          data.numDropoffsPrefixSum,
                          data.vehWaitTimesUntilDropoffsPrefixSum);

            const auto &startAfterRemoval = data.pos[vehId].start;
            const auto &endAfterRemoval = data.pos[vehId].end;
            for (int i = startAfterRemoval; i < endAfterRemoval; ++i) {
                data.numDropoffsPrefixSum[i] -= numDropoffsAtStart;
                --data.stopIdToPosition[data.stopIds[i]];
                KASSERT(data.stopIdToPosition[data.stopIds[i]] == i - startAfterRemoval);
            }

            if (haveToRecomputeMaxLeeway)
                data.recomputeMaxLeeway();
        }

        // Creates an intermediate stop between stop 0 and stop 1 for a vehicle reroute at the given location.
        void createIntermediateStopForReroute(RouteStateData &data, const int vehId, const int location, const int now,
                                              const int depTime) {
            KASSERT(vehId >= 0);
            KASSERT(vehId < data.pos.size());
            KASSERT(data.pos[vehId].end - data.pos[vehId].start > 0);
            KASSERT(depTime >= now);
            const int unusedStopId = StopIdManager::getUnusedStopId();
            stableInsertion(vehId, 1, unusedStopId, data.pos, data.stopIds,
                            data.stopLocations,
                            data.schedArrTimes,
                            data.schedDepTimes,
                            data.vehWaitTimesPrefixSum,
                            data.maxArrTimes,
                            data.occupancies,
                            data.numDropoffsPrefixSum,
                            data.vehWaitTimesUntilDropoffsPrefixSum);
            const auto start = data.pos[vehId].start;
            const auto end = data.pos[vehId].end;
            data.stopLocations[start + 1] = location;
            data.schedArrTimes[start + 1] = now;
            data.schedDepTimes[start + 1] = depTime;
            data.maxArrTimes[start + 1] = now;
            data.occupancies[start + 1] = data.occupancies[start];
            data.numDropoffsPrefixSum[start + 1] = data.numDropoffsPrefixSum[start];
            data.vehWaitTimesPrefixSum[start + 1] = data.vehWaitTimesPrefixSum[start];
            data.vehWaitTimesUntilDropoffsPrefixSum[start + 1] = data.vehWaitTimesUntilDropoffsPrefixSum[start];

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const int newStopId = data.stopIds[start + 1];
            const auto newMinSize = newStopId + 1;
            if (data.stopIdToIdOfPrevStop.size() < newMinSize) {
                data.stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                data.stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                data.stopIdToLeeway.resize(newMinSize, 0);
                data.stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                data.rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                data.rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            KASSERT(start == data.pos[vehId].start && end == data.pos[vehId].end);
            data.stopIdToVehicleId[newStopId] = vehId;
            data.stopIdToIdOfPrevStop[newStopId] = data.stopIds[start];
            data.stopIdToIdOfPrevStop[data.stopIds[start + 2]] = newStopId;
            for (int i = start; i < end; ++i) {
                data.stopIdToPosition[data.stopIds[i]] = i - start;
            }

            const auto leeway =
                    std::max(data.maxArrTimes[start + 2], data.schedDepTimes[start + 2]) -
                    data.schedDepTimes[start + 1] - data.stopTime;
            KASSERT(leeway >= 0);
            data.stopIdToLeeway[newStopId] = leeway;

            data.updateMaxLegLength(vehId, 1, 1);
        }

//        void moveStartOfCurrentLeg(RouteStateData &data, const int vehId,
//                                   const int newLocation, const int arrTime, const int depTime) {
//            KASSERT(vehId >= 0);
//            KASSERT(vehId < data.pos.size());
//            KASSERT(data.pos[vehId].end - data.pos[vehId].start > 0);
//            KASSERT(depTime >= arrTime);
//
//
//        }

    };
}