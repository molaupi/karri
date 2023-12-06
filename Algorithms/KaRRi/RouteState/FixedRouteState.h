#pragma once

#include <stack>

#include "Tools/Constants.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Containers/BitVector.h"

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h"
#include "Algorithms/KaRRi/BaseObjects/StopInfo.h"
#include "Algorithms/CH/CH.h"
#include "ScheduledStop.h"

namespace karri {

    template<typename VehCHEnvT, typename VehInputGraphT>
    class FixedRouteState {
    private:
        // Index Array:

        // For a vehicle with ID vehId, the according entries in each value array lie in the index interval
        // [pos[vehId].start, pos[vehId].end).
        std::vector<ValueBlockPosition> pos;

        // Value Arrays:

        // Unique ID for each stop (IDs can be reused after stops are finished, just unique for any point in time)
        std::vector<int> stopIds;

        // Locations of stops (edges in vehicle road network)
        std::vector<int> stopLocations;

        // Scheduled arrival time of vehicle for each stop
        std::vector<int> schedArrTimes;

        // Scheduled departure time of vehicle for each stop
        std::vector<int> schedDepTimes;

        // Latest permissible arrival time of vehicle at each stop in order to adhere to hard constraints of existing
        // passengers.
        std::vector<int> maxArrTimes;

        // Occupancies in route leg immediately following each stop
        std::vector<int> occupancies;

        // Index-shifted prefix sum of vehicle wait times not including the minimum stop length stopTime for any stop.
        // vehWaitTimesPrefixSum[pos[vehId].start + i] is the sum of all wait times up till and including stop i for the
        // vehicle with ID vehId.
        std::vector<int> vehWaitTimesPrefixSum;

        // For any vehicle and its i-th stop, this value is the sum of the prefix sums of vehicle wait times up to dropoff
        // d for each dropoff d before the i-th stop.
        // Let N_d(l) be the number of dropoffs at stop l.
        // Then vehWaitTimesUntilDropoffsPrefixSum[i] = \sum_{z = 0}^{i} N_d(z) * vehWaitTimesPrefixSum[z - 1]
        std::vector<int> vehWaitTimesUntilDropoffsPrefixSum;

        // Prefix sum of the number of dropoffs scheduled for the vehicle up to a stop.
        // numDropoffsPrefixSum[pos[vehId].start + i] is the number of dropoffs before stop i plus the number of dropoffs
        // at stop i for the vehicle with ID vehId.
        std::vector<int> numDropoffsPrefixSum;



        // Mappings of stop ids to other aspects of the respective vehicle route:

        // Maps each stop id to the id of the stop before it in the route of its vehicle.
        std::vector<int> stopIdToIdOfPrevStop;

        // Maps each stop id to its position in the route of its vehicle.
        std::vector<int> stopIdToPosition;

        // stopIdToLeeway[id] is the current leeway in the leg starting at the stop with stopId id.
        std::vector<int> stopIdToLeeway;

        // stopIdToVehicleId[stopId] is the id of the vehicle that the stop with stopId is currently part of the route of.
        std::vector<int> stopIdToVehicleId;

        // Pickups and dropoffs per request as dynamic ragged 2D-arrays.
        // The range requestsPickedUpAtStop[rangeOfRequestsPickedUpAtStop[stopId].start ... rangeOfRequestsPickedUpAtStop[stopId].end]
        // stores the IDs of all requests that are picked up at stop with ID stopId. (Analogous for dropoffs.)
        std::vector<ValueBlockPosition> rangeOfRequestsPickedUpAtStop;
        std::vector<int> requestsPickedUpAtStop;
        std::vector<ValueBlockPosition> rangeOfRequestsDroppedOffAtStop;
        std::vector<int> requestsDroppedOffAtStop;


        // Other data:

        int maxLeeway;
        int stopIdOfMaxLeeway;

        int maxLegLength;
        int stopIdOfMaxLegLength;

        std::stack<int, std::vector<int>> unusedStopIds;
        int nextUnusedStopId;
        int maxStopId;

        const int stopTime;

        //Distance query stuff
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const VehInputGraphT &vehInputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;


    public:
        FixedRouteState(const Fleet &fleet, const int stopTime, VehCHEnvT &vehChEnv, VehInputGraphT &vehInputGraph)
                : pos(fleet.size()),
                  stopIds(fleet.size()),
                  stopLocations(fleet.size()),
                  schedArrTimes(fleet.size()),
                  schedDepTimes(fleet.size()),
                  maxArrTimes(fleet.size()),
                  occupancies(fleet.size()),
                  vehWaitTimesPrefixSum(fleet.size()),
                  vehWaitTimesUntilDropoffsPrefixSum(fleet.size()),
                  numDropoffsPrefixSum(fleet.size()),
                  stopIdToIdOfPrevStop(fleet.size(), INVALID_ID),
                  stopIdToPosition(fleet.size(), 0),
                  stopIdToLeeway(fleet.size(), 0),
                  stopIdToVehicleId(fleet.size(), INVALID_ID),
                  rangeOfRequestsPickedUpAtStop(fleet.size()),
                  requestsPickedUpAtStop(),
                  rangeOfRequestsDroppedOffAtStop(fleet.size()),
                  requestsDroppedOffAtStop(),
                  maxLeeway(0),
                  stopIdOfMaxLeeway(INVALID_ID),
                  maxLegLength(0),
                  stopIdOfMaxLegLength(INVALID_ID),
                  unusedStopIds(),
                  nextUnusedStopId(fleet.size()),
                  maxStopId(fleet.size() - 1),
                  stopTime(stopTime),
                  vehInputGraph(vehInputGraph),
                  vehCh(vehChEnv.getCH()),
                  vehChQuery(vehChEnv.template getFullCHQuery<>()) {
            for (auto i = 0; i < fleet.size(); ++i) {
                pos[i].start = i;
                pos[i].end = i + 1;
                stopIds[i] = i;
                stopIdToVehicleId[i] = i;
                stopLocations[i] = fleet[i].initialLocation;
                schedArrTimes[i] = fleet[i].startOfServiceTime;
                schedDepTimes[i] = fleet[i].startOfServiceTime;
                vehWaitTimesPrefixSum[i] = 0;
                occupancies[i] = 0;
                numDropoffsPrefixSum[i] = 0;
                vehWaitTimesUntilDropoffsPrefixSum[i] = 0;
                maxArrTimes[i] = INFTY;
            }
        }

        const int &getMaxStopId() const {
            return maxStopId;
        }

        int numStopsOf(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            return pos[vehId].end - pos[vehId].start;
        }

        // Range containing the ids of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopIdsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopIds.begin() + start, stopIds.begin() + end};
        }

        // Range containing the locations (= edges) of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopLocationsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopLocations.begin() + start, stopLocations.begin() + end};
        }

        // Range containing the scheduled arrival times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedArrTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedArrTimes.begin() + start, schedArrTimes.begin() + end};
        }

        // Range containing the scheduled departure times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedDepTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedDepTimes.begin() + start, schedDepTimes.begin() + end};
        }

        // Range containing the latest possible arrival times of vehicle with given ID at its stops s.t. all hard
        // constraints of the vehicle and its passengers are still satisfied.
        ConstantVectorRange<int> maxArrTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {maxArrTimes.begin() + start, maxArrTimes.begin() + end};
        }

        // Range containing a prefix sum over the wait times of the vehicle with given ID at its stops. A vehicle
        // wait time means any amount of time that a vehicle has to wait for a passenger to arrive at a stop (excluding
        // the minimum duration stopTime of each stop).
        ConstantVectorRange<int> vehWaitTimesPrefixSumFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {vehWaitTimesPrefixSum.begin() + start, vehWaitTimesPrefixSum.begin() + end};
        }

        // Range containing the occupancies of each leg of the currently scheduled route of the vehicle with given ID.
        // occupanciesFor(vehId)[i] means the number of passengers that travel in the vehicle between stops i and i+1.
        ConstantVectorRange<int> occupanciesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {occupancies.begin() + start, occupancies.begin() + end};
        }

        // Range containing a prefix sum over the number of dropoffs that the vehicle with given ID is scheduled to
        // make at each of its stops.
        ConstantVectorRange<int> numDropoffsPrefixSumFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {numDropoffsPrefixSum.begin() + start, numDropoffsPrefixSum.begin() + end};
        }

        // For any vehicle and its i-th stop, this value is the sum of the prefix sums of vehicle wait times up to dropoff
        // d for each dropoff d before the i-th stop.
        // Let N_d(l) be the number of dropoffs at stop l.
        // Then vehWaitTimesUntilDropoffsPrefixSumsFor(vehId)[i] = \sum_{z = 0}^{i} N_d(z) * vehWaitTimesPrefixSumFor(vehId)[z - 1]
        ConstantVectorRange<int> vehWaitTimesUntilDropoffsPrefixSumsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {vehWaitTimesUntilDropoffsPrefixSum.begin() + start,
                    vehWaitTimesUntilDropoffsPrefixSum.begin() + end};
        }

        // Returns the id of the vehicle whose route the stop with the given ID is currently part of.
        int vehicleIdOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToVehicleId.size());
            return stopIdToVehicleId[stopId];
        }

        // Returns the id of the stop that comes before the stop with the given ID in the route of its vehicle.
        int idOfPreviousStopOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToIdOfPrevStop.size());
            return stopIdToIdOfPrevStop[stopId];
        }

        int stopPositionOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToPosition[stopId];
        }

        int leewayOfLegStartingAt(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToLeeway.size());
            return stopIdToLeeway[stopId];
        }

        const int &getMaxLeeway() const {
            return maxLeeway;
        }

        const int &getMaxLegLength() const {
            return maxLegLength;
        }


        void addNewPickup(StopInfo &info) {
            assert(info.insertIndex == 0 || info.insertIndex == 1);
            const auto &start = pos[info.vehicleId].start;
            const auto &end = pos[info.vehicleId].end;
            bool insertedPickupAsNewStop = false;

            if (info.insertIndex == 0 || (hasNextScheduledStop(info.vehicleId) && info.location == stopLocations[start + info.insertIndex])) {
                //The pickup is the next fixed stop or the current stop (last minute insert)
                if (schedDepTimes[start + info.insertIndex] != info.schedDepTime) {
                    schedDepTimes[start + info.insertIndex] = info.schedDepTime;
                    if (end - 1 >= start + info.insertIndex + 1)
                        propagateSchedArrAndDepForward(start + info.insertIndex + 1, end - 1, calcDistance(info.location, stopLocations[start + info.insertIndex + 1]));
                }
                schedArrTimes[start + info.insertIndex] = info.schedArrTime;
                maxArrTimes[start + info.insertIndex] = info.maxArrTime;
            } else {
                //The pickup location currently does not exist as fixed stop
                assert(info.insertIndex == 1);
                stableInsertion(info.vehicleId, info.insertIndex, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + info.insertIndex] = info.location;
                schedArrTimes[start + info.insertIndex] = info.schedArrTime;
                schedDepTimes[start + info.insertIndex] = info.schedDepTime;
                maxArrTimes[start + info.insertIndex] = info.maxArrTime;

                vehWaitTimesPrefixSum[start + info.insertIndex] = 0;
                numDropoffsPrefixSum[start + info.insertIndex] = numDropoffsPrefixSum[start + info.insertIndex - 1];; // No dropoffs otherwise it would've been a fixed stop
                occupancies[start + info.insertIndex] = occupancies[start];

                if (numStopsOf(info.vehicleId) > 2) {
                    //Newly inserted stop changed Arr and departure times for upcoming stops
                    int distance = calcDistance(info.location, stopLocations[start + info.insertIndex + 1]);
                    propagateSchedArrAndDepForward(start + info.insertIndex + 1, end - 1, distance);
                }
                insertedPickupAsNewStop = true;
                info.fixedStopId = stopIds[start + info.insertIndex];
            }

            propgateNumOfOccupanciesForward(info.vehicleId, info.insertIndex, info.pickedupReqAndDropoff.size());


            // Updating all the other vectors

            const auto size = stopIds[start + info.insertIndex] + 1;
            if (stopIdToIdOfPrevStop.size() < size) {
                stopIdToIdOfPrevStop.resize(size, INVALID_ID);
                stopIdToPosition.resize(size, INVALID_INDEX);
                stopIdToLeeway.resize(size, 0);
                stopIdToVehicleId.resize(size, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(size);
                rangeOfRequestsDroppedOffAtStop.resize(size);
            }

            if (insertedPickupAsNewStop) {
                assert(info.insertIndex == 1);
                stopIdToVehicleId[stopIds[start + info.insertIndex]] = info.vehicleId;
                stopIdToIdOfPrevStop[stopIds[start + info.insertIndex]] = stopIds[start];
                if (numStopsOf(info.vehicleId) > 2) {
                    stopIdToIdOfPrevStop[stopIds[start + info.insertIndex + 1]] = stopIds[start + info.insertIndex];
                }
                for (int i = start + info.insertIndex; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            for (const auto& tuple : info.pickedupReqAndDropoff) {
                insertion(stopIds[start + info.insertIndex], tuple.first,rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            }
        }

        void addNewDropoff(StopInfo &info, const int requestId) {
            const auto &start = pos[info.vehicleId].start;
            const auto &end = pos[info.vehicleId].end;
            bool insertedDropoffAsNewStop = false;

            if (numStopsOf(info.vehicleId) == info.insertIndex) {
                // Dropoff is a new stop and inserted as the last stop
                stableInsertion(info.vehicleId, info.insertIndex, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + info.insertIndex] = info.location;
                schedArrTimes[start + info.insertIndex] = schedDepTimes[start + info.insertIndex - 1] + calcDistance(stopLocations[start + info.insertIndex - 1], info.location);
                schedDepTimes[start + info.insertIndex] = schedArrTimes[start + info.insertIndex] + stopTime;
                maxArrTimes[start + info.insertIndex] = info.maxArrTimeAtDropoff;
                occupancies[start + info.insertIndex] = occupancies[start + info.insertIndex - 1];
                numDropoffsPrefixSum[start + info.insertIndex] = numDropoffsPrefixSum[start + info.insertIndex - 1];

                propagateMaxArrTimeBackward(start + info.insertIndex - 1, start);
                insertedDropoffAsNewStop = true;
                info.fixedStopId = stopIds[start + info.insertIndex];
            } else if(stopLocations[start + info.insertIndex] == info.location) {
                // Dropoff stop already fixed
                maxArrTimes[start + info.insertIndex] = std::min(maxArrTimes[start + info.insertIndex], info.maxArrTimeAtDropoff);
                propagateMaxArrTimeBackward(start + info.insertIndex, start);
            } else {
                // Dropoff is a new stop but not the last one
                stableInsertion(info.vehicleId, info.insertIndex, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + info.insertIndex] = info.location;
                schedArrTimes[start + info.insertIndex] = schedDepTimes[start + info.insertIndex - 1] + calcDistance(stopLocations[start + info.insertIndex - 1], info.location);
                schedDepTimes[start + info.insertIndex] = schedArrTimes[start + info.insertIndex] + stopTime;
                maxArrTimes[start + info.insertIndex] = info.maxArrTimeAtDropoff;
                occupancies[start + info.insertIndex] = occupancies[start + info.insertIndex - 1];
                numDropoffsPrefixSum[start + info.insertIndex] = numDropoffsPrefixSum[start + info.insertIndex - 1];

                propagateSchedArrAndDepForward(start + info.insertIndex + 1, end - 1, calcDistance(info.location, stopLocations[start + info.insertIndex + 1]));
                propagateMaxArrTimeBackward(start + info.insertIndex, start);
                insertedDropoffAsNewStop = true;
                info.fixedStopId = stopIds[start + info.insertIndex];
            }

            propgateNumOfOccupanciesForward(info.vehicleId, info.insertIndex, -1);
            propagateNumOfDropoffsForward(info.vehicleId, info.insertIndex, 1);
            recalculateVehWaitTimesPrefixSum(start + 1, end - 1, vehWaitTimesPrefixSum[start]);
            recalculateVehWaitTimesAtDropoffsPrefixSum(start + 1, end - 1, vehWaitTimesUntilDropoffsPrefixSum[start]);

            const auto size = stopIds[start + info.insertIndex] + 1;
            if (stopIdToIdOfPrevStop.size() < size) {
                stopIdToIdOfPrevStop.resize(size, INVALID_ID);
                stopIdToPosition.resize(size, INVALID_INDEX);
                stopIdToLeeway.resize(size, 0);
                stopIdToVehicleId.resize(size, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(size);
                rangeOfRequestsDroppedOffAtStop.resize(size);
            }

            if (insertedDropoffAsNewStop) {
                stopIdToVehicleId[stopIds[start + info.insertIndex]] = info.vehicleId;
                stopIdToIdOfPrevStop[stopIds[start + info.insertIndex]] = stopIds[start + info.insertIndex - 1];
                if (start + info.insertIndex != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + info.insertIndex + 1]] = stopIds[start + info.insertIndex];
            }

            if (insertedDropoffAsNewStop) {
                for (int i = start + 1; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            updateLeeways(info.vehicleId);

            insertion(stopIds[start + info.insertIndex], requestId,rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

        }


        void removeStartOfCurrentLeg(const int vehId) {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            assert(testNumberOfOccupancies(vehId));
            assert(testNumDropoffsPrefixSum(vehId));
            assert(testVehicleWaitTimes(vehId));
            const auto &start = pos[vehId].start;
            assert(pos[vehId].end - start > 0);
            const bool haveToRecomputeMaxLeeway = stopIds[start] == stopIdOfMaxLeeway;
            stopIdToVehicleId[stopIds[start]] = INVALID_ID;
            stopIdToLeeway[stopIds[start]] = 0;
            stopIdToPosition[stopIds[start]] = INVALID_INDEX;
            removalOfAllCols(stopIds[start], rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            removalOfAllCols(stopIds[start], rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);
            unusedStopIds.push(stopIds[start]);
            assert(stopIdToIdOfPrevStop[stopIds[start]] == INVALID_ID);
            if (numStopsOf(vehId) > 1) {
                stopIdToIdOfPrevStop[stopIds[start + 1]] = INVALID_ID;
            }

            const auto numDropoffsAtStart = numDropoffsPrefixSum[start];
            stableRemoval(vehId, 0,
                          pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                          maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

            const auto &startAfterRemoval = pos[vehId].start;
            const auto &endAfterRemoval = pos[vehId].end;
            for (int i = startAfterRemoval; i < endAfterRemoval; ++i) {
                numDropoffsPrefixSum[i] -= numDropoffsAtStart;
                --stopIdToPosition[stopIds[i]];
                assert(stopIdToPosition[stopIds[i]] == i - startAfterRemoval);
            }

            if (haveToRecomputeMaxLeeway)
                recomputeMaxLeeway();

            assert(testArrAndDepTimes(vehId));
            assert(testMappingVectors(vehId));
        }


        void updateStartOfCurrentLeg(const int vehId, const int location, const int depTime) {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            assert(pos[vehId].end - pos[vehId].start > 0);
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            stopLocations[start] = location;
            schedDepTimes[start] = depTime;
            schedArrTimes[start] = depTime - stopTime;
            recalculateVehWaitTimesPrefixSum(start, end - 1, 0);
            recalculateVehWaitTimesAtDropoffsPrefixSum(start, end - 1, 0);
        }

        bool hasNextScheduledStop(const int vehId) const {
            return numStopsOf(vehId) > 1;
        }

        ScheduledStop getNextScheduledStop(const int vehId) const {
            return getScheduledStop(vehId, 1);
        }

        ScheduledStop getCurrentOrPrevScheduledStop(const int vehId) const {
            return getScheduledStop(vehId, 0);
        }


    private:

        bool testNumberOfOccupancies(const int vehId) {
            bool result = true;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            int currNumOfPassengers = occupancies[start];
            for (int i = start + 1; i < end; i++) {
                const int currStopId = stopIds[i];
                const int numPeoplePickedup = rangeOfRequestsPickedUpAtStop[currStopId].end - rangeOfRequestsPickedUpAtStop[currStopId].start;
                const int numPeopleDroppedoff = rangeOfRequestsDroppedOffAtStop[currStopId].end - rangeOfRequestsDroppedOffAtStop[currStopId].start;
                currNumOfPassengers += numPeoplePickedup - numPeopleDroppedoff;
                result = result && occupancies[i] >= 0 && occupancies[i] == currNumOfPassengers;
            }
            return result && occupancies[end - 1] == 0;
        }

        bool testArrAndDepTimes(const int vehId) {
            bool result = true;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;

            for (int i = start; i < end - 1; i++) {
                result = result && (schedDepTimes[i] - schedArrTimes[i] >= stopTime) &&
                        (schedArrTimes[i + 1] - schedDepTimes[i] == calcDistance(stopLocations[i], stopLocations[i + 1]));
            }
            return result;
        }

        bool testNumDropoffsPrefixSum(const int vehId) {
            bool result = true;
            const auto &start = pos[vehId].start;
            const auto &end  = pos[vehId].end;
            int currNumDropoffs = 0;
            for (int i = start; i < end; i++) {
                currNumDropoffs += rangeOfRequestsDroppedOffAtStop[stopIds[i]].end - rangeOfRequestsDroppedOffAtStop[stopIds[i]].start;
                result = result && numDropoffsPrefixSum[i] == currNumDropoffs;
            }
            return result;
        }

        bool testMappingVectors(const int vehId) {
            bool result = true;
            const auto &start = pos[vehId].start;
            const auto &end  = pos[vehId].end;

            int prevStopId = INVALID_ID;
            for (int i = start; i < end; i++) {
                const auto stopId = stopIds[i];
                result = result && (stopIdToVehicleId[stopId] == vehId) &&
                        (stopIdToPosition[stopId] == i - start) &&
                        (stopIdToIdOfPrevStop[stopId] == prevStopId);
                prevStopId = stopId;
            }
            return result;
        }
        bool testVehicleWaitTimes(const int vehId) {
            bool result = true;
            const auto &start = pos[vehId].start;
            const auto &end  = pos[vehId].end;
            int prevSum = vehWaitTimesPrefixSum[start];
            for (int i = start + 1; i < end; i++) {
                const auto stopLength = schedDepTimes[i] - stopTime - schedArrTimes[i];
                result = result && (vehWaitTimesPrefixSum[i] == prevSum + stopLength);
                prevSum = vehWaitTimesPrefixSum[i];
            }
            return result;
        }

        //Calculates distance between two edges
        int calcDistance(int from, int to) {
            const auto source = vehCh.rank(vehInputGraph.edgeHead(from));
            const auto target = vehCh.rank(vehInputGraph.edgeTail(to));
            vehChQuery.run(source, target);
            return vehChQuery.getDistance() + vehInputGraph.travelTime(to);
        }

        void propgateNumOfOccupanciesForward(const int vehId, const int index, const int change) {
            for (int i = pos[vehId].start + index; i < pos[vehId].end; i++) {
                occupancies[i] += change;
            }
        }

        void propagateNumOfDropoffsForward(const int vehId, const int index, const int change) {
            for (int idx = pos[vehId].start + index; idx < pos[vehId].end; ++idx) {
                numDropoffsPrefixSum[idx] += change;
            }
        }




        ScheduledStop getScheduledStop(const int vehId, const int stopIndex) const {
            assert(numStopsOf(vehId) > stopIndex);
            const auto id = stopIdsFor(vehId)[stopIndex];
            const auto arrTime = schedArrTimesFor(vehId)[stopIndex];
            const auto depTime = schedDepTimesFor(vehId)[stopIndex];
            const auto occ = occupanciesFor(vehId)[stopIndex];
            const auto pickupsRange = rangeOfRequestsPickedUpAtStop[id];
            const ConstantVectorRange<int> pickups = {requestsPickedUpAtStop.begin() + pickupsRange.start,
                                                      requestsPickedUpAtStop.begin() + pickupsRange.end};
            const auto dropoffsRange = rangeOfRequestsDroppedOffAtStop[id];
            const ConstantVectorRange<int> dropoffs = {requestsDroppedOffAtStop.begin() + dropoffsRange.start,
                                                       requestsDroppedOffAtStop.begin() + dropoffsRange.end};
            return {id, arrTime, depTime, occ, pickups, dropoffs};
        }

        int getUnusedStopId() {
            if (!unusedStopIds.empty()) {
                const auto id = unusedStopIds.top();
                unusedStopIds.pop();
                assert(stopIdToVehicleId[id] == INVALID_ID);
                return id;
            }
            ++maxStopId;
            return nextUnusedStopId++;
        }


        // Standard forward propagation of changes to minVehArrTime and minVehDepTime from fromIdx to toIdx (both inclusive)
        // caused by inserting a pickup stop. Needs distance from stop at fromIdx - 1 to stop at fromIdx because that
        // distance cannot be inferred. Indices are direct indices in the 2D arrays.
        void propagateSchedArrAndDepForward(const int fromIdx, const int toIdx, const int distFromPrevOfFromIdx) {
            assert(distFromPrevOfFromIdx > 0);
            int distPrevToCurrent = distFromPrevOfFromIdx;
            for (int l = fromIdx; l <= toIdx; ++l) {
                schedArrTimes[l] = schedDepTimes[l - 1] + distPrevToCurrent;

                // If the planned departure time is already later than the new arrival time demands, then the planned
                // departure time remains unaffected and subsequent arrival/departure times will not change either.
                if (schedDepTimes[l] >= schedArrTimes[l] + stopTime) {
                    break;
                }

                const auto oldMinDepTime = schedDepTimes[l];
                schedDepTimes[l] = schedArrTimes[l] + stopTime; // = max(schedDepTimes[l], schedArrTimes[l] + stopTime);
                if (l < toIdx) distPrevToCurrent = schedArrTimes[l + 1] - oldMinDepTime;
            }
        }

        // Backwards propagation of changes to maxArrTimes from fromIdx down to toIdx
        void propagateMaxArrTimeBackward(const int fromIdx, const int toIdx) {
            for (int l = fromIdx; l >= toIdx; --l) {
                const auto distToNext = schedArrTimes[l + 1] - schedDepTimes[l];
                const auto propagatedMaxArrTime = maxArrTimes[l + 1] - distToNext - stopTime;
                if (maxArrTimes[l] <= propagatedMaxArrTime)
                    break; // Stop propagating if known maxArrTime at l is stricter already
                maxArrTimes[l] = propagatedMaxArrTime;
            }
        }

        void updateLeeways(const int vehId) {
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;

            for (int idx = start; idx < end - 1; ++idx) {
                const auto &stopId = stopIds[idx];

                // Set leeway of stop, possibly update max leeway
                const auto leeway =
                        std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1]) - schedDepTimes[idx] - stopTime;
                assert(leeway >= 0);
                stopIdToLeeway[stopId] = leeway;

                if (leeway > maxLeeway) {
                    maxLeeway = leeway;
                    stopIdOfMaxLeeway = stopId;
                }
            }

            if (stopIdToLeeway[stopIdOfMaxLeeway] < maxLeeway) {
                // Leeway of stop that previously had the max leeway has decreased s.t. it is no longer the stop with
                // the largest leeway, so we recompute the largest leeway from scratch.
                recomputeMaxLeeway();
            }
        }

        void updateMaxLegLength(const int vehId, const int pickupIndex, const int dropoffIndex) {
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;

            bool lengthOfFormerLongestChanged = false;
            bool maxLengthIncreased = false;

            auto updateLegLengthAt = [&](const int stopIdx) {
                const auto toPickup = schedArrTimes[start + stopIdx + 1] - schedDepTimes[start + stopIdx];
                lengthOfFormerLongestChanged |= stopIds[start + stopIdx] == stopIdOfMaxLegLength;
                if (toPickup >= maxLegLength) {
                    maxLegLength = toPickup;
                    stopIdOfMaxLegLength = stopIds[start + stopIdx];
                    maxLengthIncreased = true;
                }
            };

            if (pickupIndex > 0)
                updateLegLengthAt(pickupIndex - 1);
            if (pickupIndex + 1 != dropoffIndex)
                updateLegLengthAt(pickupIndex);
            updateLegLengthAt(dropoffIndex - 1);
            if (start + dropoffIndex < end - 1)
                updateLegLengthAt(dropoffIndex);

            // If we did not find a new maximum leg length but the length of the formerly longest leg changed, we need
            // to recompute the max leg length from scratch.
            if (!maxLengthIncreased && lengthOfFormerLongestChanged)
                recomputeMaxLegLength();
        }

        // Recalculate the prefix sum of vehicle wait times from fromIdx up to toIdx (both inclusive) based on current
        // minVehArrTime and minVehDepTime values. Takes a sum for the element before fromIdx as baseline.
        void recalculateVehWaitTimesPrefixSum(const int fromIdx, const int toIdx, const int baseline) {
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                const auto stopLength = schedDepTimes[l] - stopTime - schedArrTimes[l];
                assert(stopLength >= 0);
                vehWaitTimesPrefixSum[l] = prevSum + stopLength;
                prevSum = vehWaitTimesPrefixSum[l];
            }
        }

        void recalculateVehWaitTimesAtDropoffsPrefixSum(const int fromIdx, const int toIdx, const int baseline) {
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                const auto numDropoffs = numDropoffsPrefixSum[l] - (l == 0 ? 0 : numDropoffsPrefixSum[l - 1]);
                const auto waitPrefixSum = l == 0 ? 0 : vehWaitTimesPrefixSum[l - 1];
                vehWaitTimesUntilDropoffsPrefixSum[l] = prevSum + numDropoffs * waitPrefixSum;
                prevSum = vehWaitTimesUntilDropoffsPrefixSum[l];
            }
        }

        void recomputeMaxLeeway() {
            maxLeeway = 0;
            stopIdOfMaxLeeway = INVALID_ID;
            for (const auto&  [start, end] : pos) {
                for (int idx = start; idx < end - 1; ++idx) {
                    const auto leeway = std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1])
                                        - schedDepTimes[idx] - stopTime;
                    if (leeway > maxLeeway) {
                        maxLeeway = leeway;
                        stopIdOfMaxLeeway = stopIds[idx];
                    }
                }
            }
        }

        void recomputeMaxLegLength() {
            maxLegLength = 0;
            stopIdOfMaxLegLength = INVALID_ID;
            for (const auto& [start, end] : pos) {
                for (int idx = start; idx < end - 1; ++idx) {
                    const auto legLength = schedArrTimes[idx + 1] - schedDepTimes[idx];
                    if (legLength > maxLegLength) {
                        maxLegLength = legLength;
                        stopIdOfMaxLegLength = stopIds[idx];
                    }
                }
            }
        }
    };
}