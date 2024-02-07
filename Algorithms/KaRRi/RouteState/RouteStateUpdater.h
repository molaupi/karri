//
// Created by tim on 07.12.23.
//

#pragma once

#include "RouteStateData.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"

namespace karri {

    class RouteStateUpdater {
    public:
        RouteStateUpdater(RouteStateData &data, const int stopTime)
        : data(data),
        stopTime(stopTime) {}

        template<typename RequestStateT>
        std::pair<int, int>
        insert(const Assignment &asgn, const RequestStateT &requestState) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &pickup = *asgn.pickup;
            const auto &dropoff = *asgn.dropoff;
            const int now = requestState.originalRequest.requestTime;
            auto pickupIndex = asgn.pickupStopIdx;
            auto dropoffIndex = asgn.dropoffStopIdx;

            auto stopIds = data.stopIdsFor(vehId);
            auto schedDepTimes = data.schedDepTimesFor(vehId);
            auto schedArrTimes = data.schedArrTimesFor(vehId);
            auto maxArrTimes = data.maxArrTimesFor(vehId);
            auto stopLocations = data.stopLocationsFor(vehId);
            auto occupancies = data.occupanciesFor(vehId);
            auto numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            auto vehWaitTimesPrefixSum = data.vehWaitTimesPrefixSumFor(vehId);
            auto vehWaitTimesUntilDropoffsPrefixSum = data.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId);

            int size = schedDepTimes.size();

            assert(pickupIndex >= 0);
            assert(pickupIndex < schedDepTimes.size());
            assert(dropoffIndex >= 0);
            assert(dropoffIndex < schedDepTimes.size());

            bool pickupInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;


            if ((pickupIndex > 0 || schedDepTimes[0] > now) && pickup.loc == stopLocations[pickupIndex]) {
                assert(pickupIndex == size - 1 || pickupIndex == dropoffIndex ||
                       asgn.distFromPickup ==
                       schedArrTimes[pickupIndex + 1] - schedDepTimes[pickupIndex]);

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                data.updateSchedDepTimesFor(vehId, pickupIndex, std::max(schedDepTimes[pickupIndex],
                                                                         requestState.getPassengerArrAtPickup(pickup.id)));

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime =
                        std::max(requestState.getMaxDepTimeAtPickup(), requestState.getPassengerArrAtPickup(pickup.id));
                data.updateMaxArrTimesFor(vehId, pickupIndex, std::min(maxArrTimes[pickupIndex], psgMaxDepTime - stopTime));
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                data.updateSchedDepTimesFor(vehId, size - 1, std::max(schedDepTimes[size - 1], requestState.originalRequest.requestTime));
                data.updateSchedArrTimesFor(vehId, size - 1, schedDepTimes[size - 1] - stopTime);
                ++pickupIndex;
                ++dropoffIndex;
                ++size;

                data.addNewStopFor(vehId, pickupIndex);

                rereshRouteDataDatastructures(vehId, stopIds, schedDepTimes,schedArrTimes, maxArrTimes, stopLocations,
                                              occupancies, numDropoffsPrefixSum, vehWaitTimesPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

                data.updateStopLocationFor(vehId, pickupIndex, pickup.loc);
                data.updateSchedArrTimesFor(vehId, pickupIndex, schedDepTimes[pickupIndex - 1] + asgn.distToPickup);
                data.updateSchedDepTimesFor(vehId, pickupIndex, std::max(schedArrTimes[pickupIndex] + stopTime,
                                                                         requestState.getPassengerArrAtPickup(pickup.id)));
                data.updateMaxArrTimesFor(vehId, pickupIndex, requestState.getMaxDepTimeAtPickup() - stopTime);
                data.updateOccupanciesFor(vehId, pickupIndex, occupancies[pickupIndex - 1]);
                data.updateNumDropoffsPrefixSumFor(vehId, pickupIndex, numDropoffsPrefixSum[pickupIndex - 1]);
                pickupInsertedAsNewStop = true;
            }

            if (pickupIndex != dropoffIndex) {
                // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
                assert(asgn.distFromPickup > 0);
                propagateSchedArrAndDepForward(vehId, pickupIndex + 1, dropoffIndex, asgn.distFromPickup, schedArrTimes, schedDepTimes);
            }

            if (pickup.loc != dropoff.loc && dropoff.loc == stopLocations[dropoffIndex]) {
                data.updateMaxArrTimesFor(vehId, dropoffIndex, std::min(maxArrTimes[dropoffIndex],
                                                                        requestState.getMaxArrTimeAtDropoff(pickup.id, dropoff.id)));
            } else {
                ++dropoffIndex;
                ++size;
                data.addNewStopFor(vehId, dropoffIndex);

                rereshRouteDataDatastructures(vehId, stopIds, schedDepTimes,schedArrTimes, maxArrTimes, stopLocations,
                                              occupancies, numDropoffsPrefixSum, vehWaitTimesPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

                data.updateStopLocationFor(vehId, dropoffIndex, dropoff.loc);
                data.updateSchedArrTimesFor(vehId, dropoffIndex, schedDepTimes[dropoffIndex - 1] + asgn.distToDropoff);
                data.updateSchedDepTimesFor(vehId, dropoffIndex, schedArrTimes[dropoffIndex] + stopTime);
                data.updateMaxArrTimesFor(vehId, dropoffIndex, requestState.getMaxArrTimeAtDropoff(pickup.id, dropoff.id));
                data.updateOccupanciesFor(vehId, dropoffIndex, occupancies[dropoffIndex - 1]);
                data.updateNumDropoffsPrefixSumFor(vehId, dropoffIndex, numDropoffsPrefixSum[dropoffIndex - 1]);
                dropoffInsertedAsNewStop = true;
            }

            // Propagate updated scheduled arrival and departure times as well as latest permissible arrival times.
            if (dropoffIndex < size - 1) {
                // At this point minDepTimes[start + dropoffIndex] is correct. If dropoff has been inserted not as the last
                // stop, propagate the changes to minDep and minArr times forward until the last stop.
                propagateSchedArrAndDepForward(vehId, dropoffIndex + 1, size - 1, asgn.distFromDropoff, schedArrTimes, schedDepTimes);

                // If there are stops after the dropoff, consider them for propagating changes to the maxArrTimes
                propagateMaxArrTimeBackward(vehId, dropoffIndex, pickupIndex, schedArrTimes, schedDepTimes, maxArrTimes);
            } else {
                // If there are no stops after the dropoff, propagate maxArrTimes backwards not including dropoff
                propagateMaxArrTimeBackward(vehId, dropoffIndex - 1, pickupIndex, schedArrTimes, schedDepTimes, maxArrTimes);
            }
            propagateMaxArrTimeBackward(vehId, pickupIndex - 1, 0, schedArrTimes, schedDepTimes, maxArrTimes);

            // Update occupancies and prefix sums
            for (int idx = pickupIndex; idx < dropoffIndex; ++idx) {
                data.updateOccupanciesFor(vehId, idx, occupancies[idx] + 1);
                assert(occupancies[idx] <= asgn.vehicle->capacity);
            }

            for (int idx = dropoffIndex; idx < size; ++idx) {
                data.updateNumDropoffsPrefixSumFor(vehId, idx, numDropoffsPrefixSum[idx] + 1);
            }


            const int lastUnchangedPrefixSum = pickupIndex > 0 ? vehWaitTimesPrefixSum[pickupIndex - 1] : 0;
            recalculateVehWaitTimesPrefixSum(vehId, pickupIndex, size - 1, lastUnchangedPrefixSum, schedArrTimes, schedDepTimes);
            const int lastUnchangedAtDropoffsPrefixSum = pickupIndex > 0 ? vehWaitTimesUntilDropoffsPrefixSum[pickupIndex - 1] : 0;
            recalculateVehWaitTimesAtDropoffsPrefixSum(vehId, pickupIndex, size - 1, lastUnchangedAtDropoffsPrefixSum, numDropoffsPrefixSum, vehWaitTimesPrefixSum);

            if (pickupInsertedAsNewStop) {
                assert(pickupIndex >= 1 && pickupIndex < size - 1);
                data.updateVehicleIdOf(stopIds[pickupIndex], vehId);
                data.updateIdOfPreviousStopOf(stopIds[pickupIndex], stopIds[pickupIndex - 1]);
                data.updateIdOfPreviousStopOf(stopIds[pickupIndex + 1], stopIds[pickupIndex]);
            }
            if (dropoffInsertedAsNewStop) {
                assert(dropoffIndex > pickupIndex && dropoffIndex < size);
                data.updateVehicleIdOf(stopIds[dropoffIndex], vehId);
                data.updateIdOfPreviousStopOf(stopIds[dropoffIndex], stopIds[dropoffIndex - 1]);
                if (dropoffIndex != size - 1)
                    data.updateIdOfPreviousStopOf(stopIds[dropoffIndex + 1], stopIds[dropoffIndex]);
            }

            if (pickupInsertedAsNewStop || dropoffInsertedAsNewStop) {
                for (int i = pickupIndex; i < size; ++i) {
                    data.updateStopPositionOf(stopIds[i], i);
                }
            }

            updateLeeways(vehId, stopIds, schedDepTimes, maxArrTimes);


            // Remember that request is picked up and dropped of at respective stops:
            data.addPickedUpRequest(stopIds[pickupIndex], requestState.originalRequest.requestId);
            data.addDroppedOffRequest(stopIds[dropoffIndex], requestState.originalRequest.requestId);

            return {pickupIndex, dropoffIndex};
        }


        void removeStartOfCurrentLeg(const int vehId) {
            assert(vehId >= 0);
            auto stopIds = data.stopIdsFor(vehId);
            auto numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            int size = stopIds.size();
            assert(size > 0);
            const bool haveToRecomputeMaxLeeway = stopIds[0] == data.getStopIdOfMaxLeeway();
            data.updateVehicleIdOf(stopIds[0], INVALID_ID);
            data.updateLeewayOfLegStartingAt(stopIds[0], 0);
            data.updateStopPositionOf(stopIds[0], INVALID_INDEX);
            data.removePickedUpRequests(stopIds[0]);
            data.removeDroppedOffRequests(stopIds[0]);
            assert(data.idOfPreviousStopOf(stopIds[0]) == INVALID_ID);
            if (data.numStopsOf(vehId) > 1) {
                data.updateIdOfPreviousStopOf(stopIds[1], INVALID_ID);
            }

            const auto numDropoffsAtStart = numDropoffsPrefixSum[0];
            data.removeStopFor(vehId, stopIds[0], 0);
            size--;
            stopIds = data.stopIdsFor(vehId);
            numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            for (int i = 0; i < size; ++i) {
                data.updateNumDropoffsPrefixSumFor(vehId, i, numDropoffsPrefixSum[i] - numDropoffsAtStart);
                data.updateStopPositionOf(stopIds[i], i);
            }

            if (haveToRecomputeMaxLeeway)
                recomputeMaxLeeway();
        }



        void updateStartOfCurrentLeg(const int vehId, const int location, const int depTime) {
            assert(vehId >= 0);
            assert(data.numStopsOf(vehId) > 0);
            auto schedDepTimes = data.schedDepTimesFor(vehId);
            auto schedArrTimes = data.schedArrTimesFor(vehId);
            auto numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            auto vehWaitTimesPrefixSum = data.vehWaitTimesPrefixSumFor(vehId);

            data.updateStopLocationFor(vehId, 0, location);
            data.updateSchedDepTimesFor(vehId, 0, depTime);
            data.updateSchedArrTimesFor(vehId, 0, depTime - stopTime);

            recalculateVehWaitTimesPrefixSum(vehId, 0, data.numStopsOf(vehId) - 1, 0, schedArrTimes, schedDepTimes);
            recalculateVehWaitTimesAtDropoffsPrefixSum(vehId, 0, data.numStopsOf(vehId) - 1, 0, numDropoffsPrefixSum, vehWaitTimesPrefixSum);
        }

        std::vector<int> exchangeRouteFor(const int vehId, const RouteStateData &otherData) {
            assert(data.numStopsOf(vehId) >= otherData.numStopsOf(vehId));
            std::vector<int> result;
            const auto otherStopIds = otherData.stopIdsFor(vehId);
            auto stopIds = data.stopIdsFor(vehId);
            int index = 0;
            bool maxLeewayChanged = false;

            while (index < data.numStopsOf(vehId)) {
                if (index >= otherData.numStopsOf(vehId) || stopIds[index] != otherStopIds[index]) {
                    if (data.getStopIdOfMaxLeeway() == stopIds[index]) {
                        maxLeewayChanged = true;
                    }
                    result.push_back(stopIds[index]);
                    deleteRouteDataFor(vehId, stopIds[index], index);
                    stopIds = data.stopIdsFor(vehId);
                    continue;
                }
                data.updateSchedArrTimesFor(vehId, index, otherData.schedArrTimesFor(vehId)[index]);
                data.updateSchedDepTimesFor(vehId, index, otherData.schedDepTimesFor(vehId)[index]);
                data.updateMaxArrTimesFor(vehId, index, otherData.maxArrTimesFor(vehId)[index]);
                data.updateOccupanciesFor(vehId, index, otherData.occupanciesFor(vehId)[index]);
                data.updateVehWaitTimesPrefixSumFor(vehId, index, otherData.vehWaitTimesPrefixSumFor(vehId)[index]);
                data.updateVehWaitTimesUntilDropoffsPrefixSumFor(vehId, index, otherData.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId)[index]);
                data.updateNumDropoffsPrefixSumFor(vehId, index, otherData.numDropoffsPrefixSumFor(vehId)[index]);
                data.updateIdOfPreviousStopOf(stopIds[index], otherData.idOfPreviousStopOf(stopIds[index]));
                data.updateStopPositionOf(stopIds[index], index);

                const int leeway = otherData.leewayOfLegStartingAt(stopIds[index]);
                data.updateLeewayOfLegStartingAt(stopIds[index], leeway);
                if (data.getMaxLeeway() < leeway) {
                    data.updateMaxLeeway(stopIds[index], leeway);
                } else if (data.getStopIdOfMaxLeeway() == stopIds[index]) {
                    maxLeewayChanged = true;
                }

                data.removeDroppedOffRequests(stopIds[index]);
                data.removePickedUpRequests(stopIds[index]);
                for (const int reqId: otherData.getRequestsPickedUpAt(stopIds[index])) {
                    data.addPickedUpRequest(stopIds[index], reqId);
                }
                for (const int reqId: otherData.getRequestsDroppedOffAt(stopIds[index])) {
                    data.addDroppedOffRequest(stopIds[index], reqId);
                }

                index++;
            }

            if (maxLeewayChanged) {
                recomputeMaxLeeway();
            }
            assert(index == otherData.numStopsOf(vehId) && index == data.numStopsOf(vehId));
            return result;
        }


    private:

        // This method only removes and invalidates the data for one stopId.
        // There are value arrays inside RouteData that store codependent data which need to be updated separately.
        void deleteRouteDataFor(const int vehId, const int stopId, const int index) {
            data.removePickedUpRequests(stopId);
            data.removeDroppedOffRequests(stopId);
            data.removeStopFor(vehId, stopId, index);
            data.updateIdOfPreviousStopOf(stopId, INVALID_ID);
            data.updateStopPositionOf(stopId, INVALID_INDEX);
            data.updateLeewayOfLegStartingAt(stopId, 0);
            data.updateVehicleIdOf(stopId, INVALID_ID);
        }

        void rereshRouteDataDatastructures(const int vehId, ConstantVectorRange<int> &stopIds, ConstantVectorRange<int> &schedDepTimes,
                ConstantVectorRange<int> &schedArrTimes, ConstantVectorRange<int> &maxArrTimes, ConstantVectorRange<int> &stopLocations,
                ConstantVectorRange<int> &occupancies, ConstantVectorRange<int> &numDropoffsPrefixSum, ConstantVectorRange<int> &vehWaitTimesPrefixSum,
                ConstantVectorRange<int> &vehWaitTimesUntilDropoffsPrefixSum) {
            stopIds = data.stopIdsFor(vehId);
            schedDepTimes = data.schedDepTimesFor(vehId);
            schedArrTimes = data.schedArrTimesFor(vehId);
            maxArrTimes = data.maxArrTimesFor(vehId);
            stopLocations = data.stopLocationsFor(vehId);
            occupancies = data.occupanciesFor(vehId);
            numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            vehWaitTimesPrefixSum = data.vehWaitTimesPrefixSumFor(vehId);
            vehWaitTimesUntilDropoffsPrefixSum = data.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId);
        }

        // Standard forward propagation of changes to minVehArrTime and minVehDepTime from fromIdx to toIdx (both inclusive)
        // caused by inserting a pickup stop. Needs distance from stop at fromIdx - 1 to stop at fromIdx because that
        // distance cannot be inferred. Indices are direct indices in the 2D arrays.
        void propagateSchedArrAndDepForward(const int vehId, const int fromIdx, const int toIdx, const int distFromPrevOfFromIdx,
                                            ConstantVectorRange<int> &schedArrTimes, ConstantVectorRange<int> &schedDepTimes) {
            assert(distFromPrevOfFromIdx > 0);
            int distPrevToCurrent = distFromPrevOfFromIdx;
            for (int l = fromIdx; l <= toIdx; ++l) {
                data.updateSchedArrTimesFor(vehId, l, schedDepTimes[l - 1] + distPrevToCurrent);

                // If the planned departure time is already later than the new arrival time demands, then the planned
                // departure time remains unaffected and subsequent arrival/departure times will not change either.
                if (schedDepTimes[l] >= schedArrTimes[l] + stopTime) {
                    break;
                }

                const auto oldMinDepTime = schedDepTimes[l];
                data.updateSchedDepTimesFor(vehId, l, schedArrTimes[l] + stopTime); // = max(schedDepTimes[l], schedArrTimes[l] + stopTime);
                if (l < toIdx) distPrevToCurrent = schedArrTimes[l + 1] - oldMinDepTime;
            }
        }

        // Backwards propagation of changes to maxArrTimes from fromIdx down to toIdx
        void propagateMaxArrTimeBackward(const int vehId, const int fromIdx, const int toIdx, ConstantVectorRange<int> &schedArrTimes,
                                         ConstantVectorRange<int> &schedDepTimes, ConstantVectorRange<int> &maxArrTimes) {
            for (int l = fromIdx; l >= toIdx; --l) {
                const auto distToNext = schedArrTimes[l + 1] - schedDepTimes[l];
                const auto propagatedMaxArrTime = maxArrTimes[l + 1] - distToNext - stopTime;
                if (maxArrTimes[l] <= propagatedMaxArrTime)
                    break; // Stop propagating if known maxArrTime at l is stricter already
                data.updateMaxArrTimesFor(vehId, l, propagatedMaxArrTime);
            }
        }

        void updateLeeways(const int vehId, ConstantVectorRange<int> &stopIds, ConstantVectorRange<int> &schedDepTimes,
                           ConstantVectorRange<int> &maxArrTimes) {
            for (int idx = 0; idx < data.numStopsOf(vehId) - 1; ++idx) {
                const auto &stopId = stopIds[idx];

                // Set leeway of stop, possibly update max leeway
                const auto leeway =
                        std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1]) - schedDepTimes[idx] - stopTime;
                assert(leeway >= 0);
                data.updateLeewayOfLegStartingAt(stopId, leeway);

                if (leeway > data.getMaxLeeway()) {
                    data.updateMaxLeeway(stopId, leeway);
                }
            }

            if (data.leewayOfLegStartingAt(data.getStopIdOfMaxLeeway()) < data.getMaxLeeway()) {
                // Leeway of stop that previously had the max leeway has decreased s.t. it is no longer the stop with
                // the largest leeway, so we recompute the largest leeway from scratch.
                recomputeMaxLeeway();
            }
        }

        // Recalculate the prefix sum of vehicle wait times from fromIdx up to toIdx (both inclusive) based on current
        // minVehArrTime and minVehDepTime values. Takes a sum for the element before fromIdx as baseline.
        void recalculateVehWaitTimesPrefixSum(const int vehId, const int fromIdx, const int toIdx, const int baseline, ConstantVectorRange<int> &schedArrTimes,
                                              ConstantVectorRange<int> &schedDepTimes) {
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                const auto stopLength = schedDepTimes[l] - stopTime - schedArrTimes[l];
                const int value = prevSum + stopLength;
                assert(stopLength >= 0);
                data.updateVehWaitTimesPrefixSumFor(vehId, l, value);
                prevSum = value;
            }
        }

        void recalculateVehWaitTimesAtDropoffsPrefixSum(const int vehId, const int fromIdx, const int toIdx, const int baseline,
                                                        ConstantVectorRange<int> &numDropoffsPrefixSum, ConstantVectorRange<int> &vehWaitTimesPrefixSum) {
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                const auto numDropoffs = numDropoffsPrefixSum[l] - (l == 0 ? 0 : numDropoffsPrefixSum[l - 1]);
                const auto waitPrefixSum = l == 0 ? 0 : vehWaitTimesPrefixSum[l - 1];
                const int value = prevSum + numDropoffs * waitPrefixSum;
                data.updateVehWaitTimesUntilDropoffsPrefixSumFor(vehId, l, value);
                prevSum = value;
            }
        }

        void recomputeMaxLeeway() {
            data.updateMaxLeeway(INVALID_ID, 0);
            for (unsigned long vehId = 0; vehId < data.getFleetSize(); vehId++) {
                for (int idx = 0; idx < data.numStopsOf(vehId) - 1; ++idx) {
                    auto maxArrTimes = data.maxArrTimesFor(vehId);
                    auto schedDepTimes = data.schedDepTimesFor(vehId);
                    auto stopIds = data.stopIdsFor(vehId);
                    const auto leeway = std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1])
                                        - schedDepTimes[idx] - stopTime;
                    if (leeway > data.getMaxLeeway()) {
                        data.updateMaxLeeway(stopIds[idx], leeway);
                    }
                }
            }
        }

        RouteStateData &data;
        const int stopTime;
    };
}