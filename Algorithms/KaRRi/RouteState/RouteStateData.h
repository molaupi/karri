//
// Created by tim on 07.12.23.
//

#pragma once


#include <vector>
#include <stack>
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Tools/Constants.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "ScheduledStop.h"

namespace karri {

    class RouteStateData {

    public:
        RouteStateData(const Fleet &fleet)
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
                  maxLegLength(INFTY), //TODO: War davor 0 aber momentan updates ausgeschaltet
                  // stopIdOfMaxLegLength(INVALID_ID),
                  unusedStopIds(),
                  nextUnusedStopId(fleet.size()),
                  maxStopId(fleet.size() - 1),
                  fleetSize(fleet.size()) {
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

        const int &getStopIdOfMaxLeeway() const {
            return stopIdOfMaxLeeway;
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

        void addNewStopFor(const int vehId, const int index, const int stopId = INVALID_ID) {
            const auto newStopId = (stopId == INVALID_ID) ? getUnusedStopId() : stopId;
            if (stopId != INVALID_ID)
                maxStopId = (stopId > maxStopId) ? stopId : maxStopId;
            if (stopIdToLeeway.size() < newStopId + 1)
                resizeStopIdRelatedDatastructures(newStopId + 1);
            stableInsertion(vehId, index, newStopId, pos, stopIds, stopLocations,
                            schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                            numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
        }

        void removeStopFor(const int vehId, const int stopId, const int index) {
            stableRemoval(vehId, index,
                          pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                          maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
            unusedStopIds.push(stopId);
        }

        // Range containing the locations (= edges) of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopLocationsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopLocations.begin() + start, stopLocations.begin() + end};
        }

        void updateStopLocationFor(const int vehId, const int index,  const int value) {
            stopLocations[pos[vehId].start + index] = value;
        }

        // Range containing the scheduled arrival times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedArrTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedArrTimes.begin() + start, schedArrTimes.begin() + end};
        }

        void updateSchedArrTimesFor(const int vehId, const int index,  const int value) {
            schedArrTimes[pos[vehId].start + index] = value;
        }

        // Range containing the scheduled departure times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedDepTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedDepTimes.begin() + start, schedDepTimes.begin() + end};
        }

        void updateSchedDepTimesFor(const int vehId, const int index,  const int value) {
            schedDepTimes[pos[vehId].start + index] = value;
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

        void updateMaxArrTimesFor(const int vehId, const int index,  const int value) {
            maxArrTimes[pos[vehId].start + index] = value;
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


        void updateVehWaitTimesPrefixSumFor(const int vehId, const int index,  const int value) {
            vehWaitTimesPrefixSum[pos[vehId].start + index] = value;
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

        void updateOccupanciesFor(const int vehId, const int index,  const int value) {
            occupancies[pos[vehId].start + index] = value;
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

        void updateNumDropoffsPrefixSumFor(const int vehId, const int index,  const int value) {
            numDropoffsPrefixSum[pos[vehId].start + index] = value;
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

        void updateVehWaitTimesUntilDropoffsPrefixSumFor(const int vehId, const int index,  const int value) {
            vehWaitTimesUntilDropoffsPrefixSum[pos[vehId].start + index] = value;
        }

        // Returns the id of the vehicle whose route the stop with the given ID is currently part of.
        int vehicleIdOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToVehicleId[stopId];
        }

        void updateVehicleIdOf(const int stopId, const int vehId) {
            stopIdToVehicleId[stopId] = vehId;
        }

        // Returns the id of the stop that comes before the stop with the given ID in the route of its vehicle.
        int idOfPreviousStopOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToIdOfPrevStop.size());
            return stopIdToIdOfPrevStop[stopId];
        }

        void updateIdOfPreviousStopOf(const int stopId, const int prevStopId) {
            stopIdToIdOfPrevStop[stopId] = prevStopId;
        }

        int stopPositionOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToPosition[stopId];
        }

        void updateStopPositionOf(const int stopId, const int position) {
            stopIdToPosition[stopId] = position;
        }

        int leewayOfLegStartingAt(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToLeeway.size());
            return stopIdToLeeway[stopId];
        }

        void updateLeewayOfLegStartingAt(const int stopId, const int leeway) {
            stopIdToLeeway[stopId] = leeway;
        }

        const int &getMaxLeeway() const {
            return maxLeeway;
        }

        void updateMaxLeeway(const int stopId, const int leeway) {
            maxLeeway = leeway;
            stopIdOfMaxLeeway = stopId;
        }

        const int &getMaxLegLength() const {
            return maxLegLength;
        }

        void updateMaxLegLength(const int legLength) {
            maxLegLength = legLength;
        }

        bool hasNextScheduledStop(const int vehId) const {
            return numStopsOf(vehId) > 1;
        }

        void addPickedUpRequest(const int stopId, const int requestId) {
            insertion(stopId, requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
        }

        void removePickedUpRequests(const int stopId) {
            removalOfAllCols(stopId, rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
        }

        void addDroppedOffRequest(const int stopId, const int requestId) {
            insertion(stopId, requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);
        }

        void removeDroppedOffRequests(const int stopId) {
            removalOfAllCols(stopId, rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);
        }

        ScheduledStop getNextScheduledStop(const int vehId) const {
            return getScheduledStop(vehId, 1);
        }

        ScheduledStop getCurrentOrPrevScheduledStop(const int vehId) const {
            return getScheduledStop(vehId, 0);
        }

        unsigned long getFleetSize() {
            return fleetSize;
        }

        bool stopHasPickups(const int stopId) {
            return rangeOfRequestsPickedUpAtStop[stopId].end - rangeOfRequestsPickedUpAtStop[stopId].start != 0;
        }

    private:

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

        void resizeStopIdRelatedDatastructures(const int newSize) {
            stopIdToIdOfPrevStop.resize(newSize, INVALID_ID);
            stopIdToPosition.resize(newSize, INVALID_INDEX);
            stopIdToLeeway.resize(newSize, 0);
            stopIdToVehicleId.resize(newSize, INVALID_ID);
            rangeOfRequestsPickedUpAtStop.resize(newSize);
            rangeOfRequestsDroppedOffAtStop.resize(newSize);
        }


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
        //int stopIdOfMaxLegLength;

        std::stack<int, std::vector<int>> unusedStopIds;
        int nextUnusedStopId;
        int maxStopId;

        unsigned long fleetSize;
    };

}