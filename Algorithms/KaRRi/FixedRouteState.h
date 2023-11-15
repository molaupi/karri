#pragma once

#include <stack>

#include "Tools/Constants.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Containers/BitVector.h"

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/BaseObjects/Insertion.h"
#include "Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h"

namespace karri {

    template<typename VehicleToPDLocQueryT>
    class FixedRouteState {
    private:
        // Index Array:

        // For a vehicle with ID vehId, the according entries in each value array lie in the index interval
        // [pos[vehId].start, pos[vehId].end).
        std::vector<ValueBlockPosition> pos;

        // Value Arrays:

        // Unique ID for each stop (IDs can be reused after stops are finished, just unique for any point in time)
        std::vector<int> stopIds;

        // toVarStopId[stopId] represents the stopId in RouteState
        std::vector<int> toVarStopId;

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

        VehicleToPDLocQueryT &distQuery;


    public:
        FixedRouteState(const Fleet &fleet, const int stopTime, VehicleToPDLocQueryT &query)
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
                  distQuery(query) {
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
        ConstantVectorRange<int> occupanciesFor(const int vehId) const { //TODO: Muss man hier mit der normalen occ Anzahl arbeiten?
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
            assert(stopId >= 0 && stopId < stopIdToPosition.size());
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

        void insertFixedStop(Insertion &ins) {
            const auto vehId = ins.vehicleId;
            const auto &pickup = ins.pickup;
            const auto &dropoff = ins.dropoff;
            const int now = ins.requestTime;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto pickupIndex = ins.pickupStopIdx;
            auto dropoffIndex = ins.dropoffStopIdx;

            recalculateDistances(ins);

            bool pickupInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;

            if ((pickupIndex > 0 || schedDepTimes[start] > now) && pickup.loc == stopLocations[start + pickupIndex]) {
                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + pickupIndex] = std::max(schedDepTimes[start + pickupIndex], ins.passengerArrAtPickup);

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime = std::max(ins.maxDepTimeAtPickup, ins.passengerArrAtPickup);
                maxArrTimes[start + pickupIndex] = std::min(maxArrTimes[start + pickupIndex], psgMaxDepTime - stopTime);
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], ins.requestTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - stopTime;
                ++pickupIndex;
                ++dropoffIndex;
                stableInsertion(vehId, pickupIndex, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + pickupIndex] = pickup.loc;
                schedArrTimes[start + pickupIndex] = schedDepTimes[start + pickupIndex - 1] + ins.distToPickup; //TODO: eig. distToPickup aber ist eig. 0?
                schedDepTimes[start + pickupIndex] = std::max(schedArrTimes[start + pickupIndex] + stopTime, ins.passengerArrAtPickup);
                maxArrTimes[start + pickupIndex] = ins.maxDepTimeAtPickup - stopTime;
                occupancies[start + pickupIndex] = occupancies[start + pickupIndex - 1];
                numDropoffsPrefixSum[start + pickupIndex] = numDropoffsPrefixSum[start + pickupIndex - 1];
                pickupInsertedAsNewStop = true;
            }

            if (pickupIndex != dropoffIndex) {
                // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
                propagateSchedArrAndDepForward(start + pickupIndex + 1, start + dropoffIndex, ins.distFromPickup);
            }

            if (pickup.loc != dropoff.loc && dropoff.loc == stopLocations[start + dropoffIndex]) {
                maxArrTimes[start + dropoffIndex] = std::min(maxArrTimes[start + dropoffIndex], ins.maxArrTimeAtDropoff);
            } else {
                ++dropoffIndex;
                stableInsertion(vehId, dropoffIndex, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + dropoffIndex] = dropoff.loc;
                schedArrTimes[start + dropoffIndex] =
                        schedDepTimes[start + dropoffIndex - 1] + ins.distToDropoff;
                schedDepTimes[start + dropoffIndex] = schedArrTimes[start + dropoffIndex] + stopTime;
                // compare maxVehArrTime to next stop later
                maxArrTimes[start + dropoffIndex] = ins.maxArrTimeAtDropoff;
                occupancies[start + dropoffIndex] = occupancies[start + dropoffIndex - 1];
                numDropoffsPrefixSum[start + dropoffIndex] = numDropoffsPrefixSum[start + dropoffIndex - 1];
                dropoffInsertedAsNewStop = true;
            }

            // Propagate updated scheduled arrival and departure times as well as latest permissible arrival times.
            if (start + dropoffIndex < end - 1) {
                // At this point minDepTimes[start + dropoffIndex] is correct. If dropoff has been inserted not as the last
                // stop, propagate the changes to minDep and minArr times forward until the last stop.
                propagateSchedArrAndDepForward(start + dropoffIndex + 1, end - 1, ins.distFromDropoff);

                // If there are stops after the dropoff, consider them for propagating changes to the maxArrTimes
                propagateMaxArrTimeBackward(start + dropoffIndex, start + pickupIndex);
            } else {
                // If there are no stops after the dropoff, propagate maxArrTimes backwards not including dropoff
                propagateMaxArrTimeBackward(start + dropoffIndex - 1, start + pickupIndex);
            }
            propagateMaxArrTimeBackward(start + pickupIndex - 1, start);

            // Update occupancies and prefix sums
            for (int idx = start + pickupIndex; idx < start + dropoffIndex; ++idx) {
                ++occupancies[idx]; //TODO: Occupancies Anzahl aus var. Stops beibehalten?
            }

            for (int idx = start + dropoffIndex; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

            const int lastUnchangedPrefixSum = pickupIndex > 0 ? vehWaitTimesPrefixSum[start + pickupIndex - 1] : 0;
            recalculateVehWaitTimesPrefixSum(start + pickupIndex, end - 1, lastUnchangedPrefixSum);
            const int lastUnchangedAtDropoffsPrefixSum =
                    pickupIndex > 0 ? vehWaitTimesUntilDropoffsPrefixSum[start + pickupIndex - 1] : 0;
            recalculateVehWaitTimesAtDropoffsPrefixSum(start + pickupIndex, end - 1, lastUnchangedAtDropoffsPrefixSum);

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(stopIds[start + pickupIndex], stopIds[start + dropoffIndex]) + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            assert(start == pos[vehId].start && end == pos[vehId].end);
            if (pickupInsertedAsNewStop) {
                assert(pickupIndex >= 1 && start + pickupIndex < end - 1);
                stopIdToVehicleId[stopIds[start + pickupIndex]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + pickupIndex]] = stopIds[start + pickupIndex - 1];
                stopIdToIdOfPrevStop[stopIds[start + pickupIndex + 1]] = stopIds[start + pickupIndex];
            }
            if (dropoffInsertedAsNewStop) {
                assert(dropoffIndex > pickupIndex && start + dropoffIndex < end);
                stopIdToVehicleId[stopIds[start + dropoffIndex]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + dropoffIndex]] = stopIds[start + dropoffIndex - 1];
                if (start + dropoffIndex != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + dropoffIndex + 1]] = stopIds[start + dropoffIndex];
            }

            if (pickupInsertedAsNewStop || dropoffInsertedAsNewStop) {
                for (int i = start + pickupIndex; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            updateLeeways(vehId);
            updateMaxLegLength(vehId, pickupIndex, dropoffIndex);


            // Remember that request is picked up and dropped of at respective stops:
            insertion(stopIds[start + pickupIndex], ins.requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            insertion(stopIds[start + dropoffIndex], ins.requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);
        }

        void removeStartOfCurrentLeg(const int vehId) {
            assert(vehId >= 0);
            assert(vehId < pos.size());
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

        // Scheduled stop interface for event simulation
        struct ScheduledStop {
            int stopId;
            int arrTime;
            int depTime;
            int occupancyInFollowingLeg;
            ConstantVectorRange<int> requestsPickedUpHere;
            ConstantVectorRange<int> requestsDroppedOffHere;
        };

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

        void recalculateDistances(Insertion &ins) {
            int start = pos[ins.vehicleId].start;
            int end = pos[ins.vehicleId].end;
            PDLoc point;
            point.loc = stopLocations[start + ins.pickupStopIdx];
            std::vector<PDLoc> locs = {point, ins.pickup};
            distQuery.runForward(locs);
            ins.distToPickup = ins.pickup.vehDistFromCenter;

            if (ins.pickupStopIdx == ins.dropoffStopIdx) {
                locs = {ins.pickup, ins.dropoff};
                distQuery.runForward(locs);
                ins.distToDropoff = ins.dropoff.vehDistFromCenter;
                ins.distFromPickup = 0;
            } else {
                point.loc = stopLocations[start + ins.pickupStopIdx + 1];
                locs = {ins.pickup, point};
                distQuery.runForward(locs);
                ins.distFromPickup = point.vehDistFromCenter;
                locs = {point, ins.dropoff};
                distQuery.runForward(locs);
                ins.distToDropoff = ins.dropoff.vehDistFromCenter;
            }

            if (start + ins.dropoffStopIdx == end - 1) {
                ins.distFromDropoff = 0;
            } else {
                point.loc = stopLocations[start + ins.dropoffStopIdx + 1];
                locs = {ins.dropoff, point};
                distQuery.runForward(locs);
                ins.distFromDropoff = point.vehDistFromCenter;
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