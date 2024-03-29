//
// Created by tim on 10.12.23.
//

#pragma once

#include "RouteStateData.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/StopInfo.h"

namespace karri {

    template<typename VehCHEnvT, typename VehInputGraphT>
    class FixedRouteStateUpdater {
    public:
        FixedRouteStateUpdater(RouteStateData &data, const int stopTime, VehCHEnvT &vehChEnv, VehInputGraphT &vehInputGraph)
        : data(data),
        stopTime(stopTime),
        vehInputGraph(vehInputGraph),
        vehCh(vehChEnv.getCH()),
        vehChQuery(vehChEnv.template getFullCHQuery<>()) {}


        void addNewPickup(const StopInfo &info, const int stopId) {
            assert(info.insertIndex == 0 || info.insertIndex == 1);
            bool insertedPickupAsNewStop = false;
            const int vehId = info.vehicleId;

            auto stopIds = data.stopIdsFor(vehId);
            auto schedDepTimes = data.schedDepTimesFor(vehId);
            auto schedArrTimes = data.schedArrTimesFor(vehId);
            auto maxArrTimes = data.maxArrTimesFor(vehId);
            auto stopLocations = data.stopLocationsFor(vehId);
            auto occupancies = data.occupanciesFor(vehId);
            auto numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            auto vehWaitTimesPrefixSum = data.vehWaitTimesPrefixSumFor(vehId);
            auto vehWaitTimesUntilDropoffsPrefixSum = data.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId);

            int size = stopIds.size();

            if (info.insertIndex == 0 || (data.hasNextScheduledStop(vehId) && info.location == stopLocations[info.insertIndex])) {
                //The pickup is the next fixed stop or the current stop (last minute insert)
                if (schedDepTimes[info.insertIndex] != info.schedDepTime) {
                    data.updateSchedDepTimesFor(vehId, info.insertIndex, info.schedDepTime);
                    if (size - 1 >= info.insertIndex + 1)
                        propagateSchedArrAndDepForward(vehId, info.insertIndex + 1, size - 1,
                                                       calcDistance(info.location, stopLocations[info.insertIndex + 1]),
                                                       schedArrTimes, schedDepTimes);
                }
                data.updateSchedArrTimesFor(vehId, info.insertIndex, info.schedArrTime);
                data.updateMaxArrTimesFor(vehId, info.insertIndex, info.maxArrTime);
            } else {
                //The pickup location currently does not exist as fixed stop
                assert(info.insertIndex == 1);
                data.addNewStopFor(vehId, info.insertIndex, stopId);
                size++;

                rereshRouteDataDatastructures(vehId, stopIds, schedDepTimes,schedArrTimes, maxArrTimes, stopLocations,
                                              occupancies, numDropoffsPrefixSum, vehWaitTimesPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

                data.updateStopLocationFor(vehId, info.insertIndex, info.location);
                data.updateSchedArrTimesFor(vehId, info.insertIndex, info.schedArrTime);
                data.updateSchedDepTimesFor(vehId, info.insertIndex, info.schedDepTime);
                data.updateMaxArrTimesFor(vehId, info.insertIndex, info.maxArrTime);

                data.updateVehWaitTimesPrefixSumFor(vehId, info.insertIndex, 0);
                data.updateNumDropoffsPrefixSumFor(vehId, info.insertIndex, numDropoffsPrefixSum[info.insertIndex - 1]);
                data.updateOccupanciesFor(vehId, info.insertIndex, occupancies[0]);

                if (data.numStopsOf(vehId) > 2) {
                    //Newly inserted stop changed Arr and departure times for upcoming stops
                    int distance = calcDistance(info.location, stopLocations[info.insertIndex + 1]);
                    propagateSchedArrAndDepForward(vehId, info.insertIndex + 1, size - 1, distance,
                                                   schedArrTimes, schedDepTimes);
                }
                insertedPickupAsNewStop = true;
            }

            propgateNumOfOccupanciesForward(vehId, info.insertIndex, info.pickedupReqAndDropoff.size(), occupancies);


            // Updating all the other vectors

            if (insertedPickupAsNewStop) {
                assert(info.insertIndex == 1);
                data.updateVehicleIdOf(stopIds[info.insertIndex], vehId);
                data.updateIdOfPreviousStopOf(stopIds[info.insertIndex], stopIds[0]);

                if (data.numStopsOf(vehId) > 2) {
                    data.updateIdOfPreviousStopOf(stopIds[info.insertIndex + 1], stopIds[info.insertIndex]);
                }
                for (int i = info.insertIndex; i < size; ++i) {
                    data.updateStopPositionOf(stopIds[i], i);
                }
            }

            for (const auto& tuple : info.pickedupReqAndDropoff) {
                data.addPickedUpRequest(stopIds[info.insertIndex], tuple.first);
            }
        }


        void addNewDropoff(const StopInfo &info, const int requestId, const int stopId) {
            bool insertedDropoffAsNewStop = false;
            const int vehId = info.vehicleId;

            auto stopIds = data.stopIdsFor(vehId);
            auto schedDepTimes = data.schedDepTimesFor(vehId);
            auto schedArrTimes = data.schedArrTimesFor(vehId);
            auto maxArrTimes = data.maxArrTimesFor(vehId);
            auto stopLocations = data.stopLocationsFor(vehId);
            auto occupancies = data.occupanciesFor(vehId);
            auto numDropoffsPrefixSum = data.numDropoffsPrefixSumFor(vehId);
            auto vehWaitTimesPrefixSum = data.vehWaitTimesPrefixSumFor(vehId);
            auto vehWaitTimesUntilDropoffsPrefixSum = data.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId);

            int size = stopIds.size();

            if (data.numStopsOf(vehId) == info.insertIndex) {
                // Dropoff is a new stop and inserted as the last stop
                data.addNewStopFor(vehId, info.insertIndex, stopId);
                size++;

                rereshRouteDataDatastructures(vehId, stopIds, schedDepTimes,schedArrTimes, maxArrTimes, stopLocations,
                                              occupancies, numDropoffsPrefixSum, vehWaitTimesPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

                data.updateStopLocationFor(vehId, info.insertIndex, info.location);
                data.updateSchedArrTimesFor(vehId, info.insertIndex, schedDepTimes[info.insertIndex - 1] + calcDistance(stopLocations[info.insertIndex - 1], info.location));
                data.updateSchedDepTimesFor(vehId, info.insertIndex, schedArrTimes[info.insertIndex] + stopTime);
                data.updateMaxArrTimesFor(vehId, info.insertIndex, info.maxArrTimeAtDropoff);
                data.updateOccupanciesFor(vehId, info.insertIndex, occupancies[info.insertIndex - 1]);
                data.updateNumDropoffsPrefixSumFor(vehId, info.insertIndex, numDropoffsPrefixSum[info.insertIndex - 1]);

                propagateMaxArrTimeBackward(vehId, info.insertIndex - 1, 0, schedArrTimes, schedDepTimes, maxArrTimes);
                insertedDropoffAsNewStop = true;
            } else if(stopLocations[info.insertIndex] == info.location) {
                // Dropoff stop already fixed
                data.updateMaxArrTimesFor(vehId, info.insertIndex, std::min(maxArrTimes[info.insertIndex], info.maxArrTimeAtDropoff));
                const int fromIdx = (info.insertIndex == size - 1) ? info.insertIndex - 1 : info.insertIndex;
                propagateMaxArrTimeBackward(vehId, fromIdx, 0, schedArrTimes, schedDepTimes, maxArrTimes);
            } else {
                // Dropoff is a new stop but not the last one
                data.addNewStopFor(vehId, info.insertIndex, stopId);
                size++;

                rereshRouteDataDatastructures(vehId, stopIds, schedDepTimes,schedArrTimes, maxArrTimes, stopLocations,
                                              occupancies, numDropoffsPrefixSum, vehWaitTimesPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

                data.updateStopLocationFor(vehId, info.insertIndex, info.location);
                data.updateSchedArrTimesFor(vehId, info.insertIndex, schedDepTimes[info.insertIndex - 1] + calcDistance(stopLocations[info.insertIndex - 1], info.location));
                data.updateSchedDepTimesFor(vehId, info.insertIndex, schedArrTimes[info.insertIndex] + stopTime);
                data.updateMaxArrTimesFor(vehId, info.insertIndex, info.maxArrTimeAtDropoff);
                data.updateOccupanciesFor(vehId, info.insertIndex, occupancies[info.insertIndex - 1]);
                data.updateNumDropoffsPrefixSumFor(vehId, info.insertIndex, numDropoffsPrefixSum[info.insertIndex - 1]);

                propagateSchedArrAndDepForward(vehId, info.insertIndex + 1, size - 1,
                                               calcDistance(info.location, stopLocations[info.insertIndex + 1]), schedArrTimes, schedDepTimes);
                propagateMaxArrTimeBackward(vehId, info.insertIndex, 0, schedArrTimes, schedDepTimes, maxArrTimes);
                insertedDropoffAsNewStop = true;
            }

            propgateNumOfOccupanciesForward(vehId, info.insertIndex, -1, occupancies);
            propagateNumOfDropoffsForward(vehId, info.insertIndex, 1, numDropoffsPrefixSum);
            recalculateVehWaitTimesPrefixSum(vehId, 1, size - 1, vehWaitTimesPrefixSum[0], schedArrTimes, schedDepTimes);
            recalculateVehWaitTimesAtDropoffsPrefixSum(vehId, 1, size - 1, vehWaitTimesUntilDropoffsPrefixSum[0],
                                                       numDropoffsPrefixSum, vehWaitTimesPrefixSum);

            if (insertedDropoffAsNewStop) {

                data.updateVehicleIdOf(stopIds[info.insertIndex], vehId);
                data.updateIdOfPreviousStopOf(stopIds[info.insertIndex], stopIds[info.insertIndex - 1]);
                if (info.insertIndex != size - 1)
                    data.updateIdOfPreviousStopOf(stopIds[info.insertIndex + 1], stopIds[info.insertIndex]);
            }

            if (insertedDropoffAsNewStop) {
                for (int i = 1; i < size; ++i) {
                    data.updateStopPositionOf(stopIds[i], i);
                }
            }

            updateLeeways(info.vehicleId, stopIds, schedDepTimes, maxArrTimes);

            data.addDroppedOffRequest(stopIds[info.insertIndex], requestId);
        }


        void removeStartOfCurrentLeg(const int vehId) {
            assert(vehId >= 0);
            //testNumberOfOccupancies(vehId);
            //assert(testNumDropoffsPrefixSum(vehId));
            //assert(testVehicleWaitTimes(vehId));
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

            //assert(testArrAndDepTimes(vehId));
            //assert(testMappingVectors(vehId));
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


        //Calculates distance between two edges
        int calcDistance(int from, int to) {
            const auto source = vehCh.rank(vehInputGraph.edgeHead(from));
            const auto target = vehCh.rank(vehInputGraph.edgeTail(to));
            vehChQuery.run(source, target);
            return vehChQuery.getDistance() + vehInputGraph.travelTime(to);
        }


    private:


        void propgateNumOfOccupanciesForward(const int vehId, const int index, const int change, ConstantVectorRange<int> &occupancies) {
            for (int i = index; i < occupancies.size(); i++) {
                data.updateOccupanciesFor(vehId, i, occupancies[i] + change);
            }
        }

        void propagateNumOfDropoffsForward(const int vehId, const int index, const int change, ConstantVectorRange<int> &numDropoffsPrefixSum) {
            for (int idx = index; idx < numDropoffsPrefixSum.size(); ++idx) {
                data.updateNumDropoffsPrefixSumFor(vehId, idx, numDropoffsPrefixSum[idx] + change);
            }
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



        /**
        void testNumberOfOccupancies(const int vehId) {
            const auto &start = data.pos[vehId].start;
            const auto &end = data.pos[vehId].end;
            int currNumOfPassengers = data.occupancies[start];
            for (int i = start + 1; i < end; i++) {
                const int currStopId = data.stopIds[i];
                const int numPeoplePickedup = data.rangeOfRequestsPickedUpAtStop[currStopId].end - data.rangeOfRequestsPickedUpAtStop[currStopId].start;
                const int numPeopleDroppedoff = data.rangeOfRequestsDroppedOffAtStop[currStopId].end - data.rangeOfRequestsDroppedOffAtStop[currStopId].start;
                currNumOfPassengers += numPeoplePickedup - numPeopleDroppedoff;
                assert(data.occupancies[i] >= 0 && data.occupancies[i] == currNumOfPassengers);
            }
            assert(data.occupancies[end - 1] == 0);
        }

        bool testArrAndDepTimes(const int vehId) {
            bool result = true;
            const auto &start = data.pos[vehId].start;
            const auto &end = data.pos[vehId].end;

            for (int i = start; i < end - 1; i++) {
                result = result && (data.schedDepTimes[i] - data.schedArrTimes[i] >= stopTime) &&
                         (data.schedArrTimes[i + 1] - data.schedDepTimes[i] == calcDistance(data.stopLocations[i], data.stopLocations[i + 1]));
            }
            return result;
        }

        bool testNumDropoffsPrefixSum(const int vehId) {
            bool result = true;
            const auto &start = data.pos[vehId].start;
            const auto &end  = data.pos[vehId].end;
            int currNumDropoffs = 0;
            for (int i = start; i < end; i++) {
                currNumDropoffs += data.rangeOfRequestsDroppedOffAtStop[data.stopIds[i]].end - data.rangeOfRequestsDroppedOffAtStop[data.stopIds[i]].start;
                result = result && data.numDropoffsPrefixSum[i] == currNumDropoffs;
            }
            return result;
        }

        bool testMappingVectors(const int vehId) {
            bool result = true;
            const auto &start = data.pos[vehId].start;
            const auto &end  = data.pos[vehId].end;

            int prevStopId = INVALID_ID;
            for (int i = start; i < end; i++) {
                const auto stopId = data.stopIds[i];
                result = result && (data.stopIdToVehicleId[stopId] == vehId) &&
                         (data.stopIdToPosition[stopId] == i - start) &&
                         (data.stopIdToIdOfPrevStop[stopId] == prevStopId);
                prevStopId = stopId;
            }
            return result;
        }
        bool testVehicleWaitTimes(const int vehId) {
            bool result = true;
            const auto &start = data.pos[vehId].start;
            const auto &end  = data.pos[vehId].end;
            int prevSum = data.vehWaitTimesPrefixSum[start];
            for (int i = start + 1; i < end; i++) {
                const auto stopLength = data.schedDepTimes[i] - stopTime - data.schedArrTimes[i];
                result = result && (data.vehWaitTimesPrefixSum[i] == prevSum + stopLength);
                prevSum = data.vehWaitTimesPrefixSum[i];
            }
            return result;
        }
         */

        RouteStateData &data;
        const int stopTime;

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;
        const VehInputGraphT &vehInputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;
    };
}
