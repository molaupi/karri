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

#include <stack>

#include "Tools/Constants.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Containers/BitVector.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleLocation.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "InputConfig.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/CH/CHQuery.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Containers/Subset.h"

namespace karri {
    // Represents the state of all vehicle routes including the stop locations and schedules.
    class RouteState {
    public:

        // Callback type for calculating direct distances between consecutive stops.
        // Parameters: (curStopEdge, nextStopEdge)
        // The callback should calculate the shortest path travel time between the given stops.
        using DistanceChecker = std::function<int(int curStopEdge, int nextStopEdge)>;

        RouteState(const RouteState&) = delete;
        RouteState(RouteState&&) = delete;

        RouteState(const Fleet &fleet)
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
              isIntermediateStop(fleet.size()),
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
        vehiclesWithChangesByThisRequest(fleet.size()),
    forwardDependenciesPos(fleet.size(), {0, 0}),
    forwardDependenciesStopIds(),
    backwardDependenciesPos(fleet.size(), {0, 0}),
    backwardDependenciesStopIds()  {
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
                isIntermediateStop[i] = static_cast<uint8_t>(false);
                vehWaitTimesUntilDropoffsPrefixSum[i] = 0;
                maxArrTimes[i] = INFTY;
            }
        }

        size_t numVehicles() const {
            return pos.size();
        }

        const int &getMaxStopId() const {
            return maxStopId;
        }

        int numStopsOf(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            return pos[vehId].end - pos[vehId].start;
        }

        // Range containing the ids of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopIdsFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopIds.begin() + start, stopIds.begin() + end};
        }

        // Range containing the locations (= edges) of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopLocationsFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopLocations.begin() + start, stopLocations.begin() + end};
        }

        // Range containing the scheduled arrival times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedArrTimesFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedArrTimes.begin() + start, schedArrTimes.begin() + end};
        }

        // Range containing the scheduled departure times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedDepTimesFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedDepTimes.begin() + start, schedDepTimes.begin() + end};
        }

        // Range containing the latest possible arrival times of vehicle with given ID at its stops s.t. all hard
        // constraints of the vehicle and its passengers are still satisfied.
        ConstantVectorRange<int> maxArrTimesFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {maxArrTimes.begin() + start, maxArrTimes.begin() + end};
        }

        // Range containing a prefix sum over the wait times of the vehicle with given ID at its stops. A vehicle
        // wait time means any amount of time that a vehicle has to wait for a passenger to arrive at a stop (excluding
        // the minimum duration stopTime of each stop).
        ConstantVectorRange<int> vehWaitTimesPrefixSumFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {vehWaitTimesPrefixSum.begin() + start, vehWaitTimesPrefixSum.begin() + end};
        }

        // Range containing the occupancies of each leg of the currently scheduled route of the vehicle with given ID.
        // occupanciesFor(vehId)[i] means the number of passengers that travel in the vehicle between stops i and i+1.
        ConstantVectorRange<int> occupanciesFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {occupancies.begin() + start, occupancies.begin() + end};
        }

        // Range containing a prefix sum over the number of dropoffs that the vehicle with given ID is scheduled to
        // make at each of its stops.
        ConstantVectorRange<int> numDropoffsPrefixSumFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {numDropoffsPrefixSum.begin() + start, numDropoffsPrefixSum.begin() + end};
        }

        // For any vehicle and its i-th stop, this value is the sum of the prefix sums of vehicle wait times up to dropoff
        // d for each dropoff d before the i-th stop.
        // Let N_d(l) be the number of dropoffs at stop l.
        // Then vehWaitTimesUntilDropoffsPrefixSumsFor(vehId)[i] = \sum_{z = 0}^{i} N_d(z) * vehWaitTimesPrefixSumFor(vehId)[z - 1]
        ConstantVectorRange<int> vehWaitTimesUntilDropoffsPrefixSumsFor(const int vehId) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {
                vehWaitTimesUntilDropoffsPrefixSum.begin() + start,
                vehWaitTimesUntilDropoffsPrefixSum.begin() + end
            };
        }

        bool isIntermediateStopForReroute(const int vehId, const int stopIndex) const {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            KASSERT(stopIndex >= 0);
            KASSERT(stopIndex < numStopsOf(vehId));
            return static_cast<bool>(isIntermediateStop[pos[vehId].start + stopIndex]);
        }

        // Returns the id of the vehicle whose route the stop with the given ID is currently part of.
        int vehicleIdOf(const int stopId) const {
            KASSERT(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToVehicleId[stopId];
        }

        // Returns the id of the stop that comes before the stop with the given ID in the route of its vehicle.
        int idOfPreviousStopOf(const int stopId) const {
            KASSERT(stopId >= 0 && stopId < stopIdToIdOfPrevStop.size());
            return stopIdToIdOfPrevStop[stopId];
        }

        int stopPositionOf(const int stopId) const {
            KASSERT(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToPosition[stopId];
        }

        int leewayOfLegStartingAt(const int stopId) const {
            KASSERT(stopId >= 0 && stopId < stopIdToLeeway.size());
            return stopIdToLeeway[stopId];
        }


        ConstantVectorRange<int> getForwardDependencies(const int stopId) const {
            KASSERT(stopId >= 0 && stopId < stopIdToLeeway.size());
            const auto start = forwardDependenciesPos[stopId].start;
            const auto end = forwardDependenciesPos[stopId].end;
            return {forwardDependenciesStopIds.begin() + start, forwardDependenciesStopIds.begin() + end};
        }

        ConstantVectorRange<int> getBackwardDependencies(const int stopId) const {
            KASSERT(stopId >= 0 && stopId < stopIdToLeeway.size());
            const auto start = backwardDependenciesPos[stopId].start;
            const auto end = backwardDependenciesPos[stopId].end;
            return {backwardDependenciesStopIds.begin() + start, backwardDependenciesStopIds.begin() + end};
        }

        const int &getMaxLeeway() const {
            return maxLeeway;
        }

        const int &getMaxLegLength() const {
            return maxLegLength;
        }

        // Set the distance checker callback. This should be called once after construction
        // with a lambda that captures the input graph and CH environment.
        void setDistanceChecker(DistanceChecker checker) {
            distanceChecker = std::move(checker);
        }

        template<typename RequestStateT>
        bool checkAssignmentDistances(const Assignment& asgn, const RequestStateT & requestState,
            VehicleLocation loc = {} // pass for distance checking in case of re-routing for PBNS assignment
            ) {
            unused(requestState);
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &pickup = asgn.pickup;
            const auto &dropoff = asgn.dropoff;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            const auto numStops = end - start;
            auto pickupIndex = asgn.pickupStopIdx;
            auto dropoffIndex = asgn.dropoffStopIdx;

            KASSERT(pickupIndex >= 0);
            KASSERT(pickupIndex < numStops);
            KASSERT(dropoffIndex >= 0);
            KASSERT(dropoffIndex < numStops);

            if (numStops > 1 && pickupIndex == 0 && schedDepTimes[start] < requestState.dispatchingTime) {
                // vehicle is re-routed, so correct distance goes via current vehicle location
                KASSERT(loc.location != INVALID_EDGE);
                const int actualDist = distanceChecker(stopLocations[start], loc.location) + distanceChecker(loc.location, asgn.pickup.loc);
                KASSERT(asgn.distToPickup == actualDist, "Assignment = " << asgn << ", num stops = " << numStops << " has wrong distance with re-routing (current vehicle location = " << loc.location << ")");
            } else if (pickup.loc == stopLocations[start + pickupIndex]) {
                // Pickup at existing stop
                KASSERT(asgn.distToPickup == 0, "Assignment = " << asgn << ", num stops = " << numStops);
            } else if (pickupIndex > 0 || (numStops == 1 && pickupIndex == 0)) {
                // New pickup stop
                const int actualDist = distanceChecker(stopLocations[start + pickupIndex], pickup.loc);
                KASSERT(asgn.distToPickup == actualDist, "Assignment = " << asgn << ", num stops = " << numStops);
            }

            if (dropoffIndex == pickupIndex) {
                KASSERT(asgn.distFromPickup == 0, "Assignment = " << asgn << ", num stops = " << numStops);
            } else {
                const int actualDist = distanceChecker(pickup.loc, stopLocations[start + pickupIndex + 1]);
                KASSERT(asgn.distFromPickup == actualDist, "Assignment = " << asgn << ", num stops = " << numStops);
            }

            if (dropoffIndex == pickupIndex) {
                const int actualDist = distanceChecker(pickup.loc, dropoff.loc);
                KASSERT(asgn.distToDropoff == actualDist, "Assignment = " << asgn << ", num stops = " << numStops);
            } else if (dropoff.loc == stopLocations[start + dropoffIndex]) {
                // Dropoff at existing stop
                KASSERT(asgn.distToDropoff == 0, "Assignment = " << asgn << ", num stops = " << numStops);
            } else {
                const int actualDist = distanceChecker(stopLocations[start + dropoffIndex], dropoff.loc);
                KASSERT(asgn.distToDropoff == actualDist, "Assignment = " << asgn << ", num stops = " << numStops);
            }

            if (dropoffIndex == numStops - 1) {
                KASSERT(asgn.distFromDropoff == 0, "Assignment = " << asgn << ", num stops = " << numStops);
            } else {
                const int actualDist = distanceChecker(dropoff.loc, stopLocations[start + dropoffIndex + 1]);
                KASSERT(asgn.distFromDropoff == actualDist, "Assignment = " << asgn << ", num stops = " << numStops);
            }

            return true;
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insert(const Assignment &asgn, const RequestStateT &requestState,
               Subset &lastStopsWithUpdatedDepTime,
               Subset &vehiclesWithChangesInRoute,
               const int latestVehDepTimeAtPickup,
            const int latestVehArrTimeAtDropoff,
         VehicleLocation curVehLoc = {} // pass for distance checking in case of re-routing for PBNS assignment)
         ) {
            KASSERT(checkAssignmentDistances(asgn, requestState, curVehLoc));
            const auto [pickupIdx, dropoffIdx] = insertVehStops(asgn, requestState, latestVehDepTimeAtPickup,
                                                                latestVehArrTimeAtDropoff);

            lastStopsWithUpdatedDepTime.reserve(maxStopId + 1);
            vehiclesWithChangesByThisRequest.clear();
            updateScheduleAndConstraintsAfterInsertion(asgn.vehicle->vehicleId, pickupIdx, dropoffIdx,
                                                       asgn.pickupStopIdx == asgn.dropoffStopIdx,
                                                       asgn.distFromPickup, asgn.distFromDropoff,
                                                       lastStopsWithUpdatedDepTime, vehiclesWithChangesByThisRequest);

            KASSERT(vehiclesWithChangesByThisRequest.contains(asgn.vehicle->vehicleId));
            for (const auto &vehId: vehiclesWithChangesByThisRequest) {
                updateAdditionalRouteInformation(vehId, requestState.dispatchingTime);
                vehiclesWithChangesInRoute.insert(vehId);
            }

            // Length of new legs could be longer than the previous maximum leg length.
            updateMaxLegLength(asgn.vehicle->vehicleId, pickupIdx, dropoffIdx);

            return {pickupIdx, dropoffIdx};
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insertVehStops(const Assignment &asgn, const RequestStateT &requestState,
                       const int latestVehDepTimeAtPickup, const int latestVehArrTimeAtDropoff) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &pickup = asgn.pickup;
            const auto &dropoff = asgn.dropoff;
            const int now = requestState.dispatchingTime;
            const int numRiders = requestState.originalRequest.numRiders;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto pickupIndex = asgn.pickupStopIdx;
            auto dropoffIndex = asgn.dropoffStopIdx;

            KASSERT(pickupIndex >= 0);
            KASSERT(pickupIndex < end - start);
            KASSERT(dropoffIndex >= 0);
            KASSERT(dropoffIndex < end - start);

            bool pickupInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;

            if ((pickupIndex > 0 || schedDepTimes[start] > now) && pickup.loc == stopLocations[start + pickupIndex]) {
                KASSERT(start + pickupIndex == end - 1 || pickupIndex == dropoffIndex ||
                    asgn.distFromPickup ==
                    schedArrTimes[start + pickupIndex + 1] - schedDepTimes[start + pickupIndex]);

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + pickupIndex] = std::max(schedDepTimes[start + pickupIndex],
                                                              requestState.getPassengerArrAtPickup(pickup));

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime = std::max(latestVehDepTimeAtPickup,
                                                   requestState.getPassengerArrAtPickup(pickup));
                maxArrTimes[start + pickupIndex] = std::min(maxArrTimes[start + pickupIndex], psgMaxDepTime -
                                                                InputConfig::getInstance().stopTime);

            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is dispatched. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], requestState.dispatchingTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - InputConfig::getInstance().stopTime;
                ++pickupIndex;
                ++dropoffIndex;
                dynamic_ragged2d::stableInsertion(vehId, pickupIndex, getUnusedStopId(), pos, stopIds, stopLocations,
                                                  schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                                  numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);
                stopLocations[start + pickupIndex] = pickup.loc;
                schedArrTimes[start + pickupIndex] = schedDepTimes[start + pickupIndex - 1] + asgn.distToPickup;
                schedDepTimes[start + pickupIndex] = std::max(
                    schedArrTimes[start + pickupIndex] + InputConfig::getInstance().stopTime,
                    requestState.getPassengerArrAtPickup(pickup));

                maxArrTimes[start + pickupIndex] =
                        latestVehDepTimeAtPickup - InputConfig::getInstance().stopTime;


                occupancies[start + pickupIndex] = occupancies[start + pickupIndex - 1];
                numDropoffsPrefixSum[start + pickupIndex] = numDropoffsPrefixSum[start + pickupIndex - 1];
                isIntermediateStop[start + pickupIndex] = static_cast<uint8_t>(false);
                pickupInsertedAsNewStop = true;
            }

            // In mt_karri_batch_length_walking:
            // if (pickupIndex != dropoffIndex) {
            //     // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
            //     propagateSchedArrAndDepForward(start + pickupIndex + 1, start + dropoffIndex, asgn.distFromPickup);
            // }

            if (pickup.loc != dropoff.loc && dropoff.loc == stopLocations[start + dropoffIndex]) {
                maxArrTimes[start + dropoffIndex] = std::min(maxArrTimes[start + dropoffIndex],
                                                             latestVehArrTimeAtDropoff);
            } else {
                ++dropoffIndex;
                dynamic_ragged2d::stableInsertion(vehId, dropoffIndex, getUnusedStopId(),
                                                  pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                                  maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum,
                                isIntermediateStop);
                stopLocations[start + dropoffIndex] = dropoff.loc;
                schedArrTimes[start + dropoffIndex] =
                        schedDepTimes[start + dropoffIndex - 1] + asgn.distToDropoff;
                schedDepTimes[start + dropoffIndex] =
                        schedArrTimes[start + dropoffIndex] + InputConfig::getInstance().stopTime;
                // compare maxVehArrTime to next stop later
                maxArrTimes[start + dropoffIndex] = latestVehArrTimeAtDropoff;
                occupancies[start + dropoffIndex] = occupancies[start + dropoffIndex - 1];
                numDropoffsPrefixSum[start + dropoffIndex] = numDropoffsPrefixSum[start + dropoffIndex - 1];
                isIntermediateStop[start + dropoffIndex] = static_cast<uint8_t>(false);
                dropoffInsertedAsNewStop = true;
            }

            // Update occupancies and prefix sums
            for (int idx = start + pickupIndex; idx < start + dropoffIndex; ++idx) {
                occupancies[idx] += numRiders;
                KASSERT(occupancies[idx] <= asgn.vehicle->capacity);
            }

            for (int idx = start + dropoffIndex; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

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
                forwardDependenciesPos.resize(newMinSize, {0, 0});
                backwardDependenciesPos.resize(newMinSize, {0, 0});
            }
            KASSERT(start == pos[vehId].start && end == pos[vehId].end);
            if (pickupInsertedAsNewStop) {
                KASSERT(pickupIndex >= 1 && start + pickupIndex < end - 1);
                stopIdToVehicleId[stopIds[start + pickupIndex]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + pickupIndex]] = stopIds[start + pickupIndex - 1];
                stopIdToIdOfPrevStop[stopIds[start + pickupIndex + 1]] = stopIds[start + pickupIndex];
            }
            if (dropoffInsertedAsNewStop) {
                KASSERT(dropoffIndex > pickupIndex && start + dropoffIndex < end);
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

            // In mt_karri_batch_length_walking:
            // updateLeeways(vehId, now);
            // updateMaxLegLength(vehId, pickupIndex, dropoffIndex);

            // Remember that request is picked up and dropped of at respective stops:
            dynamic_ragged2d::insertion(stopIds[start + pickupIndex], requestState.originalRequest.requestId,
                                        rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            dynamic_ragged2d::insertion(stopIds[start + dropoffIndex], requestState.originalRequest.requestId,
                                        rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            return {pickupIndex, dropoffIndex};
        }

        template<typename RequestStateT>
        std::tuple<int, int, int, int>
        insert(const AssignmentWithTransfer &asgn, const int arrTimeAtTransferPoint,
               const RequestStateT &requestState,
               Subset &lastStopsWithUpdatedDepTime,
               Subset &vehiclesWithChangesInRoute,
               const int latestVehDepTimeAtPickup,
               const int latestVehArrTimeAtDropoff) {
            const auto [pickupIdx, transferIdxPVeh] = insertPVehStops(asgn, requestState, latestVehDepTimeAtPickup);
            const auto [transferIdxDVeh, dropoffIdx] = insertDVehStops(asgn, arrTimeAtTransferPoint,
                                                                       requestState, latestVehArrTimeAtDropoff);

            lastStopsWithUpdatedDepTime.reserve(maxStopId + 1);
            const int transferStopIdPVeh = stopIds[pos[asgn.pVeh->vehicleId].start + transferIdxPVeh];
            const int transferStopIdDVeh = stopIds[pos[asgn.dVeh->vehicleId].start + transferIdxDVeh];
            addTransferDependency(transferStopIdPVeh, transferStopIdDVeh);

            vehiclesWithChangesByThisRequest.clear();
            updateScheduleAndConstraintsAfterInsertion(asgn.pVeh->vehicleId, pickupIdx, transferIdxPVeh,
                                                       asgn.pickupIdx == asgn.transferIdxPVeh,
                                                       asgn.distFromPickup, asgn.distFromTransferPVeh,
                                                       lastStopsWithUpdatedDepTime, vehiclesWithChangesByThisRequest);
            updateScheduleAndConstraintsAfterInsertion(asgn.dVeh->vehicleId, transferIdxDVeh, dropoffIdx,
                                                       asgn.transferIdxDVeh == asgn.dropoffIdx,
                                                       asgn.distFromTransferDVeh, asgn.distFromDropoff,
                                                       lastStopsWithUpdatedDepTime, vehiclesWithChangesByThisRequest);

            KASSERT(vehiclesWithChangesByThisRequest.contains(asgn.pVeh->vehicleId));
            KASSERT(vehiclesWithChangesByThisRequest.contains(asgn.dVeh->vehicleId));
            for (const auto &vehId: vehiclesWithChangesByThisRequest) {
                updateAdditionalRouteInformation(vehId, requestState.dispatchingTime);
                vehiclesWithChangesInRoute.insert(vehId);
            }

            // Length of new legs could be longer than the previous maximum leg length.
            updateMaxLegLength(asgn.pVeh->vehicleId, pickupIdx, transferIdxPVeh);
            updateMaxLegLength(asgn.dVeh->vehicleId, transferIdxDVeh, dropoffIdx);

            return {pickupIdx, transferIdxPVeh, transferIdxDVeh, dropoffIdx};
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insertPVehStops(const AssignmentWithTransfer &asgn,
                        const RequestStateT &requestState,
                        const int latestVehDepTimeAtPickup) {
            const auto vehId = asgn.pVeh->vehicleId;
            const auto &pickup = asgn.pickup;
            const auto transfer = asgn.transfer;
            const int now = requestState.dispatchingTime;
            const int numRiders = requestState.originalRequest.numRiders;

            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto pickupIdx = asgn.pickupIdx;
            auto transferIdx = asgn.transferIdxPVeh;

            KASSERT(pickupIdx >= 0);
            KASSERT(pickupIdx < end - start);
            KASSERT(transferIdx >= 0);
            KASSERT(transferIdx < end - start);

            bool pickupInsertedAsNewStop = false;
            bool transferInsertedAsNewStop = false;

            const bool pickupNotInsertedAsNewStopCond =
                    (pickupIdx > 0 || schedDepTimes[start] > now) && pickup.loc == stopLocations[start + pickupIdx];
            if (pickupNotInsertedAsNewStopCond) {
                KASSERT(start + pickupIdx == end - 1 // Pickup is at the last stop
                    || pickupIdx == transferIdx // Pickup paired
                    || asgn.distFromPickup == schedArrTimes[start + pickupIdx + 1] - schedDepTimes[start +
                        pickupIdx]); // Distance from pickup is the distance to the next stop

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + pickupIdx] = std::max(schedDepTimes[start + pickupIdx],
                                                            requestState.getPassengerArrAtPickup(pickup));

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime = std::max(latestVehDepTimeAtPickup,
                                                   requestState.getPassengerArrAtPickup(pickup));
                maxArrTimes[start + pickupIdx] = std::min(maxArrTimes[start + pickupIdx],
                                                          psgMaxDepTime - InputConfig::getInstance().stopTime);
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], requestState.dispatchingTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - InputConfig::getInstance().stopTime;
                ++pickupIdx;
                ++transferIdx;

                stableInsertion(vehId, pickupIdx, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);

                stopLocations[start + pickupIdx] = pickup.loc;
                schedArrTimes[start + pickupIdx] = schedDepTimes[start + pickupIdx - 1] + asgn.distToPickup;
                schedDepTimes[start + pickupIdx] = std::max(
                    schedArrTimes[start + pickupIdx] + InputConfig::getInstance().stopTime,
                    requestState.getPassengerArrAtPickup(pickup));
                maxArrTimes[start + pickupIdx] = latestVehDepTimeAtPickup - InputConfig::getInstance().stopTime;
                occupancies[start + pickupIdx] = occupancies[start + pickupIdx - 1];
                numDropoffsPrefixSum[start + pickupIdx] = numDropoffsPrefixSum[start + pickupIdx - 1];
                isIntermediateStop[start + pickupIdx] = static_cast<uint8_t>(false);
                pickupInsertedAsNewStop = true;
            }

            const int actualStopLocationTransfer = stopLocations[start + transferIdx];
            const bool conditionTransferNotNewStop =
                    pickup.loc != transfer.loc && transfer.loc == actualStopLocationTransfer;
            if (conditionTransferNotNewStop) {
                // Nothing to do

                //                KASSERT(schedDepTimes[start + transferIdx] > arrTimeAtTransferPoint);
                //                maxArrTimes[start + transferIdx] = std::min(maxArrTimes[start + transferIdx],
                //                                                            arrTimeAtTransferPoint); // This is not optimal, In this case we set the maxArrTime so, that the arrival can not be delayed, otherwise we would have to propagate the maxArrTime to the dropoffVehicle
            } else {
                // Insert transfer as new stop
                ++transferIdx;
                stableInsertion(vehId, transferIdx, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);
                stopLocations[start + transferIdx] = transfer.loc;
                schedArrTimes[start + transferIdx] = schedDepTimes[start + transferIdx - 1] + asgn.distToTransferPVeh;
                schedDepTimes[start + transferIdx] =
                        schedArrTimes[start + transferIdx] + InputConfig::getInstance().stopTime;
                // compare maxVehArrTime to next stop later
                //                maxArrTimes[start +
                //                            transferIdx] = arrTimeAtTransferPoint; // Not optimal, In this case we set the maxArrTime so, that the arrival can not be delayed, otherwise we would have to propagate the maxArrTime to the dropoffVehicle
                maxArrTimes[start +
                            transferIdx] = INFTY;
                // Will be set later by propagation from next stop or from dropoff vehicle
                occupancies[start + transferIdx] = occupancies[start + transferIdx - 1];
                numDropoffsPrefixSum[start + transferIdx] = numDropoffsPrefixSum[start + transferIdx - 1];
                isIntermediateStop[start + transferIdx] = static_cast<uint8_t>(false);
                transferInsertedAsNewStop = true;
            }

            // Update occupancies and prefix sums
            for (int idx = start + pickupIdx; idx < start + transferIdx; ++idx) {
                occupancies[idx] += numRiders;
                KASSERT(occupancies[idx] <= asgn.pVeh->capacity);
            }

            for (int idx = start + transferIdx; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(stopIds[start + pickupIdx], stopIds[start + transferIdx]) + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
                forwardDependenciesPos.resize(newMinSize, {0, 0});
                backwardDependenciesPos.resize(newMinSize, {0, 0});
            }
            KASSERT(start == pos[vehId].start && end == pos[vehId].end);
            if (pickupInsertedAsNewStop) {
                KASSERT(pickupIdx >= 1 && start + pickupIdx < end - 1);
                stopIdToVehicleId[stopIds[start + pickupIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + pickupIdx]] = stopIds[start + pickupIdx - 1];
                stopIdToIdOfPrevStop[stopIds[start + pickupIdx + 1]] = stopIds[start + pickupIdx];
            }
            if (transferInsertedAsNewStop) {
                KASSERT(transferIdx > pickupIdx && start + transferIdx < end);
                stopIdToVehicleId[stopIds[start + transferIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + transferIdx]] = stopIds[start + transferIdx - 1];
                if (start + transferIdx != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + transferIdx + 1]] = stopIds[start + transferIdx];
            }

            if (pickupInsertedAsNewStop || transferInsertedAsNewStop) {
                for (int i = start + pickupIdx; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            //            updateLeeways(vehId);
            //            updateMaxLegLength(vehId, pickupIdx, transferIdx);

            for (int i = start; i < end; i++) {
                KASSERT(stopIdToVehicleId[stopIds[i]] == vehId);
            }

            // Remember that request is picked up and dropped of at respective stops:
            insertion(stopIds[start + pickupIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            insertion(stopIds[start + transferIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            return {pickupIdx, transferIdx};
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insertDVehStops(const AssignmentWithTransfer &asgn,
                        const int arrTimeAtTransferPoint,
                        const RequestStateT &requestState,
                        const int latestVehArrTimeAtDropoff) {
            const auto vehId = asgn.dVeh->vehicleId;
            const auto transfer = asgn.transfer;
            const auto &dropoff = asgn.dropoff;
            const int now = requestState.dispatchingTime;
            const int numRiders = requestState.originalRequest.numRiders;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto transferIdx = asgn.transferIdxDVeh;
            auto dropoffIdx = asgn.dropoffIdx;

            KASSERT(transferIdx >= 0);
            KASSERT(transferIdx < end - start);
            KASSERT(dropoffIdx >= 0);
            KASSERT(dropoffIdx < end - start);

            bool transferInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;

            if ((transferIdx > 0 || schedDepTimes[start] > now) && transfer.loc == stopLocations[start + transferIdx]) {
                KASSERT(start + transferIdx == end - 1 || transferIdx == dropoffIdx || asgn.distFromTransferDVeh ==
                    schedArrTimes[start +
                        transferIdx + 1] -
                    schedDepTimes[start +
                        transferIdx]);

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + transferIdx] = std::max(schedDepTimes[start + transferIdx],
                                                              arrTimeAtTransferPoint);

                // No constraint for maximum arrival time of dropoff vehicle at transfer. Only propagated from
                // pickup vehicle or from dropoff.
                // maxArrTimes[start + transferIdx] = maxArrTimes[start + transferIdx]
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], requestState.dispatchingTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - InputConfig::getInstance().stopTime;
                ++transferIdx;
                ++dropoffIdx;
                stableInsertion(vehId, transferIdx, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);
                stopLocations[start + transferIdx] = transfer.loc;
                schedArrTimes[start + transferIdx] = schedDepTimes[start + transferIdx - 1] + asgn.distToTransferDVeh;
                schedDepTimes[start + transferIdx] = std::max(
                    schedArrTimes[start + transferIdx] + InputConfig::getInstance().stopTime,
                    arrTimeAtTransferPoint);
                // No constraint for maximum arrival time of dropoff vehicle at transfer. Only propagated from
                // pickup vehicle or from dropoff.
                maxArrTimes[start + transferIdx] = INFTY;
                occupancies[start + transferIdx] = occupancies[start + transferIdx - 1];
                numDropoffsPrefixSum[start + transferIdx] = numDropoffsPrefixSum[start + transferIdx - 1];
                isIntermediateStop[start + transferIdx] = static_cast<uint8_t>(false);
                transferInsertedAsNewStop = true;
            }

            if (transfer.loc != dropoff.loc && dropoff.loc == stopLocations[start + dropoffIdx]) {
                maxArrTimes[start + dropoffIdx] = std::min(maxArrTimes[start + dropoffIdx], latestVehArrTimeAtDropoff);
            } else {
                ++dropoffIdx;
                stableInsertion(vehId, dropoffIdx, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);
                stopLocations[start + dropoffIdx] = dropoff.loc;
                schedArrTimes[start + dropoffIdx] =
                        schedDepTimes[start + dropoffIdx - 1] + asgn.distToDropoff;

                schedDepTimes[start + dropoffIdx] =
                        schedArrTimes[start + dropoffIdx] + InputConfig::getInstance().stopTime;
                // compare maxVehArrTime to next stop later
                maxArrTimes[start + dropoffIdx] = latestVehArrTimeAtDropoff;
                occupancies[start + dropoffIdx] = occupancies[start + dropoffIdx - 1];
                numDropoffsPrefixSum[start + dropoffIdx] = numDropoffsPrefixSum[start + dropoffIdx - 1];
                isIntermediateStop[start + dropoffIdx] = static_cast<uint8_t>(false);
                dropoffInsertedAsNewStop = true;
            }

            // Update occupancies and prefix sums
            for (int idx = start + transferIdx; idx < start + dropoffIdx; ++idx) {
                occupancies[idx] += numRiders;
                KASSERT(occupancies[idx] <= asgn.dVeh->capacity);
            }

            for (int idx = start + dropoffIdx; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(stopIds[start + transferIdx], stopIds[start + dropoffIdx]) + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
                forwardDependenciesPos.resize(newMinSize, {0, 0});
                backwardDependenciesPos.resize(newMinSize, {0, 0});
            }
            KASSERT(start == pos[vehId].start && end == pos[vehId].end);
            if (transferInsertedAsNewStop) {
                KASSERT(transferIdx >= 1 && start + transferIdx < end - 1);
                stopIdToVehicleId[stopIds[start + transferIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + transferIdx]] = stopIds[start + transferIdx - 1];
                stopIdToIdOfPrevStop[stopIds[start + transferIdx + 1]] = stopIds[start + transferIdx];
            }
            if (dropoffInsertedAsNewStop) {
                KASSERT(dropoffIdx > transferIdx && start + dropoffIdx < end);
                stopIdToVehicleId[stopIds[start + dropoffIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + dropoffIdx]] = stopIds[start + dropoffIdx - 1];
                if (start + dropoffIdx != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + dropoffIdx + 1]] = stopIds[start + dropoffIdx];
            }

            if (transferInsertedAsNewStop || dropoffInsertedAsNewStop) {
                for (int i = start + transferIdx; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            for (int i = start; i < end; i++) {
                KASSERT(stopIdToVehicleId[stopIds[i]] == vehId);
            }


            // Remember that request is picked up and dropped of at respective stops:
            insertion(stopIds[start + transferIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            insertion(stopIds[start + dropoffIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            return {transferIdx, dropoffIdx};
        }

        void addTransferDependency(const int transferStopIdPVeh, const int transferStopIdDVeh) {
            KASSERT(transferStopIdPVeh != transferStopIdDVeh);
            const bool forwDependencyExists = contains(
                forwardDependenciesStopIds.begin() + forwardDependenciesPos[transferStopIdPVeh].start,
                forwardDependenciesStopIds.begin() + forwardDependenciesPos[transferStopIdPVeh].end,
                transferStopIdDVeh);
            const bool backwDependencyExists = contains(
                backwardDependenciesStopIds.begin() + backwardDependenciesPos[transferStopIdDVeh].start,
                backwardDependenciesStopIds.begin() + backwardDependenciesPos[transferStopIdDVeh].end,
                transferStopIdPVeh);
            unused(backwDependencyExists); // only for assertion
            KASSERT(
                (forwDependencyExists && backwDependencyExists) || (!forwDependencyExists && !backwDependencyExists));

            // Avoid duplicates
            if (forwDependencyExists)
                return;

            dynamic_ragged2d::insertion(transferStopIdPVeh, transferStopIdDVeh, forwardDependenciesPos, forwardDependenciesStopIds);
            dynamic_ragged2d::insertion(transferStopIdDVeh, transferStopIdPVeh, backwardDependenciesPos, backwardDependenciesStopIds);
        }

        // Call after inserting new stops for an assignment with transfer using insertPVehStops and insertDVehStops,
        // and after calling addTransferDependency.
        // pickupIdx and dropoffIdx describe the pickup and transfer in the pVeh or the transfer and dropoff in the dVeh
        void updateScheduleAndConstraintsAfterInsertion(const int vehId,
                                                        const int pickupIdx,
                                                        const int dropoffIdx,
                                                        const bool paired,
                                                        const int distFromPickup,
                                                        const int distFromDropoff,
                                                        Subset &lastStopsWithUpdatedDepTime,
                                                        Subset &vehiclesWithChangesInRoute) {
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            vehiclesWithChangesInRoute.insert(vehId);

            const int pickupStopId = stopIds[start + pickupIdx];
            if (!paired) {
                propagateSchedArrAndDepForward(pickupStopId, distFromPickup, lastStopsWithUpdatedDepTime,
                                               vehiclesWithChangesInRoute);
            }

            int lengthOfLegFollowingDropoff;
            if (start + dropoffIdx < end - 1) {
                lengthOfLegFollowingDropoff = distFromDropoff;
                // Propagate maxArrTime to new dropoff stop, so propagation can start at stop before dropoff and read correct maxArrTime
                maxArrTimes[start + dropoffIdx] = std::min(maxArrTimes[start + dropoffIdx],
                    maxArrTimes[start + dropoffIdx + 1] - distFromDropoff - InputConfig::getInstance().stopTime);
            } else {
                lengthOfLegFollowingDropoff = 0;
            }
            const int dropoffStopId = stopIds[start + dropoffIdx];
            propagateSchedArrAndDepForward(dropoffStopId, lengthOfLegFollowingDropoff, lastStopsWithUpdatedDepTime,
                                           vehiclesWithChangesInRoute);
            int stopIdToPropBackwardsFrom = stopIds[start + dropoffIdx - 1];
            propagateMaxArrTimeBackward(stopIdToPropBackwardsFrom, vehiclesWithChangesInRoute);

            // Propagate maxArrTime to new pickup stop, so propagation can start at stop before dropoff and read correct maxArrTime
            const int distFromPickupToNext = schedArrTimes[start + pickupIdx + 1] - schedDepTimes[start + pickupIdx];
            maxArrTimes[start + pickupIdx] = std::min(maxArrTimes[start + pickupIdx],
                maxArrTimes[start + pickupIdx + 1] - distFromPickupToNext - InputConfig::getInstance().stopTime);
            if (pickupIdx > 0) {
                const int beforePickupStopId = stopIds[start + pickupIdx - 1];
                propagateMaxArrTimeBackward(beforePickupStopId, vehiclesWithChangesInRoute);
            }
        }

        void updateAdditionalRouteInformation(const int vehId, const int now) {
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            const bool scheduleHasIntermediateStop = static_cast<bool>(isIntermediateStop[start + 1]);
            recalculateVehWaitTimesPrefixSum(start, end - 1, 0, scheduleHasIntermediateStop);
            recalculateVehWaitTimesAtDropoffsPrefixSum(start, end - 1, 0);

            updateLeeways(vehId, now);
        }


        void removeForwardDependencies(const int vehId, const int stopIdx) {
            const auto stopId = stopIds[pos[vehId].start + stopIdx];
            for (const auto &forwDepStopId: getForwardDependencies(stopId)) {
                int i = 0;
                const auto backFromForw = getBackwardDependencies(forwDepStopId);
                while (i < backFromForw.size() && backFromForw[i] != stopId) {
                    ++i;
                }
                KASSERT(i < backFromForw.size());
                if (i == backFromForw.size())
                    continue;
                dynamic_ragged2d::removal(forwDepStopId, i, backwardDependenciesPos, backwardDependenciesStopIds);
            }
            dynamic_ragged2d::removalOfAllCols(stopId, forwardDependenciesPos, forwardDependenciesStopIds);
        }

        void removeBackwardDependencies(const int vehId, const int stopIdx) {
            const auto stopId = stopIds[pos[vehId].start + stopIdx];
            for (const auto &backDepStopId: getBackwardDependencies(stopId)) {
                int i = 0;
                const auto forwFromBack = getForwardDependencies(backDepStopId);
                while (i < forwFromBack.size() && forwFromBack[i] != stopId) {
                    ++i;
                }
                KASSERT(i < forwFromBack.size());
                if (i == forwFromBack.size())
                    continue;
                dynamic_ragged2d::removal(backDepStopId, i, forwardDependenciesPos, forwardDependenciesStopIds);
            }
            dynamic_ragged2d::removalOfAllCols(stopId, backwardDependenciesPos, backwardDependenciesStopIds);
        }

        void removeStartOfCurrentLeg(const int vehId) {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            const auto &start = pos[vehId].start;
            KASSERT(pos[vehId].end - start > 0);
            const bool haveToRecomputeMaxLeeway = stopIds[start] == stopIdOfMaxLeeway;
            stopIdToVehicleId[stopIds[start]] = INVALID_ID;
            stopIdToLeeway[stopIds[start]] = 0;
            stopIdToPosition[stopIds[start]] = INVALID_INDEX;

            KASSERT(getForwardDependencies(stopIds[start]).empty());
            KASSERT(getBackwardDependencies(stopIds[start]).empty());

            dynamic_ragged2d::removalOfAllCols(stopIds[start], rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            dynamic_ragged2d::removalOfAllCols(stopIds[start], rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            // // Remove all dependencies to and from the removed stop, also in the other vehicle.
            // for (const auto &forwDepStopId: getForwardDependencies(stopIds[start])) {
            //     int i = 0;
            //     const auto backFromForw = getBackwardDependencies(forwDepStopId);
            //     while (i < backFromForw.size() && backFromForw[i] != stopIds[start]) {
            //         ++i;
            //     }
            //     KASSERT(i < backFromForw.size());
            //     if (i == backFromForw.size())
            //         continue;
            //     dynamic_ragged2d::removal(forwDepStopId, i, backwardDependenciesPos, backwardDependenciesStopIds);
            // }
            // for (const auto &backDepStopId: getBackwardDependencies(stopIds[start])) {
            //     int i = 0;
            //     const auto forwFromBack = getForwardDependencies(backDepStopId);
            //     while (i < forwFromBack.size() && forwFromBack[i] != stopIds[start]) {
            //         ++i;
            //     }
            //     KASSERT(i < forwFromBack.size());
            //     if (i == forwFromBack.size())
            //         continue;
            //     dynamic_ragged2d::removal(backDepStopId, i, forwardDependenciesPos, forwardDependenciesStopIds);
            // }
            // dynamic_ragged2d::removalOfAllCols(stopIds[start], forwardDependenciesPos, forwardDependenciesStopIds);
            // dynamic_ragged2d::removalOfAllCols(stopIds[start], backwardDependenciesPos, backwardDependenciesStopIds);
            unusedStopIds.push(stopIds[start]);
            KASSERT(stopIdToIdOfPrevStop[stopIds[start]] == INVALID_ID);
            if (numStopsOf(vehId) > 1) {
                stopIdToIdOfPrevStop[stopIds[start + 1]] = INVALID_ID;
            }

            const auto numDropoffsAtStart = numDropoffsPrefixSum[start];
            dynamic_ragged2d::stableRemoval(vehId, 0,
                                            pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                            maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);

            const auto &startAfterRemoval = pos[vehId].start;
            const auto &endAfterRemoval = pos[vehId].end;
            for (int i = startAfterRemoval; i < endAfterRemoval; ++i) {
                numDropoffsPrefixSum[i] -= numDropoffsAtStart;
                --stopIdToPosition[stopIds[i]];
                KASSERT(stopIdToPosition[stopIds[i]] == i - startAfterRemoval);
            }

            if (haveToRecomputeMaxLeeway)
                recomputeMaxLeeway();
        }

        // Creates an intermediate stop between stop 0 and stop 1 for a vehicle reroute at the given location.
        void createIntermediateStopForReroute(const int vehId, const int location, const int now, const int depTime) {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            KASSERT(pos[vehId].end - pos[vehId].start > 0);
            KASSERT(depTime >= now);
            dynamic_ragged2d::stableInsertion(vehId, 1, getUnusedStopId(), pos, stopIds, stopLocations,
                                              schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                              numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum, isIntermediateStop);
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            stopLocations[start + 1] = location;
            schedArrTimes[start + 1] = now;
            schedDepTimes[start + 1] = depTime;
            maxArrTimes[start + 1] = now;
            occupancies[start + 1] = occupancies[start];
            numDropoffsPrefixSum[start + 1] = numDropoffsPrefixSum[start];
            vehWaitTimesPrefixSum[start + 1] = vehWaitTimesPrefixSum[start];
            vehWaitTimesUntilDropoffsPrefixSum[start + 1] = vehWaitTimesUntilDropoffsPrefixSum[start];
            isIntermediateStop[start + 1] = static_cast<uint8_t>(true);

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const int newStopId = stopIds[start + 1];
            const auto newMinSize = newStopId + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
                forwardDependenciesPos.resize(newMinSize, {0, 0});
                backwardDependenciesPos.resize(newMinSize, {0, 0});
            }
            KASSERT(start == pos[vehId].start && end == pos[vehId].end);
            stopIdToVehicleId[newStopId] = vehId;
            stopIdToIdOfPrevStop[newStopId] = stopIds[start];
            stopIdToIdOfPrevStop[stopIds[start + 2]] = newStopId;
            for (int i = start; i < end; ++i) {
                stopIdToPosition[stopIds[i]] = i - start;
            }

            const auto leeway =
                    std::max(maxArrTimes[start + 2], schedDepTimes[start + 2]) - schedDepTimes[start + 1] -
                    InputConfig::getInstance().stopTime;
            KASSERT(leeway >= 0);
            stopIdToLeeway[newStopId] = leeway;

            updateMaxLegLength(vehId, 1, 1);
        }

        //* Utility methods to test if the insertion was correct
        bool assertRoutePVeh(const AssignmentWithTransfer &asgn,
            const bool insertedIntermediateStop,
            const bool pickupAtExistingStop,
                             const bool transferAtExistingStop,
                             const int depAtPickup,
                             const int arrAtTransferPoint) {

            const auto numStopsPVeh = numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = stopLocationsFor(asgn.pVeh->vehicleId);
            const auto schedDepTimesPVeh = schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimesPVeh = schedArrTimesFor(asgn.pVeh->vehicleId);
            // const auto maxArrTimesDVeh = maxArrTimesFor(asgn.dVeh->vehicleId);

            // Find the pickup and transfer indices
            const int postPickupIdx = asgn.pickupIdx + insertedIntermediateStop + !pickupAtExistingStop;
            KASSERT(stopLocationsPVeh[postPickupIdx] == asgn.pickup.loc);

            const int postTransferIdx = asgn.transferIdxPVeh + insertedIntermediateStop + !pickupAtExistingStop + !transferAtExistingStop;
            KASSERT(stopLocationsPVeh[postTransferIdx] == asgn.transfer.loc);

            KASSERT(asgn.transferIdxPVeh > asgn.pickupIdx || postPickupIdx + 1 == postTransferIdx);

            // Assert that the scheduled departure at the pickup is as expected
            const int schedDepAtPickup = schedDepTimesPVeh[postPickupIdx];
            KASSERT(schedDepAtPickup == depAtPickup);

            // Assert that the scheduled arrival at the transfer point is as expected
            const int schedArrAtTransfer = schedArrTimesPVeh[postTransferIdx];
            KASSERT(schedArrAtTransfer == arrAtTransferPoint);

            // Check distance to pickup
            if (postPickupIdx > 0 && !insertedIntermediateStop && !pickupAtExistingStop) {
                KASSERT(schedArrTimesPVeh[postPickupIdx] - schedDepTimesPVeh[postPickupIdx - 1] == asgn.distToPickup);
            }

            // Check distance from pickup
            const bool paired = asgn.transferIdxPVeh == asgn.pickupIdx;
            if (!paired) {
                KASSERT(schedArrTimesPVeh[postPickupIdx + 1] - schedDepTimesPVeh[postPickupIdx] == asgn.distFromPickup);
            }

            // Check distance to transfer
            if (!transferAtExistingStop) {
                KASSERT(schedArrTimesPVeh[postTransferIdx] - schedDepTimesPVeh[postTransferIdx - 1] == asgn.distToTransferPVeh);
            }

            // Check distance from transfer
            if (postTransferIdx < numStopsPVeh - 1) {
                KASSERT(schedArrTimesPVeh[postTransferIdx + 1] - schedDepTimesPVeh[postTransferIdx] == asgn.distFromTransferPVeh);
            }

            for (int i = 1; i < numStopsPVeh; i++) {
                const auto lastStopLoc = stopLocationsPVeh[i - 1];
                const auto lastStopDep = schedDepTimesPVeh[i - 1];
                const auto stopLoc = stopLocationsPVeh[i];
                const auto stopArr = schedArrTimesPVeh[i];

                if (!(lastStopLoc == stopLoc || stopArr > lastStopDep)) {
                    KASSERT(false);
                    return false;
                }
            }

            return true;
        }

        bool assertRouteDVeh(const AssignmentWithTransfer &asgn,
                             const bool insertedIntermediateStop,
                             const bool transferAtExistingStop,
                             const bool dropoffAtExistingStop,
                             const int arrAtTransferPoint,
                             const int depAtTransferDVeh,
                             const int arrAtDropoff) {
            const int tripTimeFromArrivalAtTransfer = arrAtDropoff - arrAtTransferPoint;

            // printRoute(asgn.dVeh->vehicleId);
            const auto numStopsDVeh = numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsDVeh = stopLocationsFor(asgn.dVeh->vehicleId);
            const auto schedDepTimesDVeh = schedDepTimesFor(asgn.dVeh->vehicleId);
            const auto schedArrTimesDVeh = schedArrTimesFor(asgn.dVeh->vehicleId);
            // const auto maxArrTimesDVeh = maxArrTimesFor(asgn.dVeh->vehicleId);

            // Find the pickup and transfer indices
            const int postTransferIdx = asgn.transferIdxDVeh + insertedIntermediateStop + !transferAtExistingStop;
            KASSERT(stopLocationsDVeh[postTransferIdx] == asgn.transfer.loc, printRouteOf(asgn.dVeh->vehicleId));

            const int postDropoffIdx = asgn.dropoffIdx + insertedIntermediateStop + !transferAtExistingStop + !dropoffAtExistingStop;
            KASSERT(stopLocationsDVeh[postDropoffIdx] == asgn.dropoff.loc, printRouteOf(asgn.dVeh->vehicleId));

            KASSERT(asgn.dropoffIdx > asgn.transferIdxDVeh || postTransferIdx + 1 == postDropoffIdx, printRouteOf(asgn.dVeh->vehicleId));


            // Assert that the scheduled departure at the transfer is later than the arrival at the transfer
            const int schedArrAtTransfer = schedArrTimesDVeh[postTransferIdx];
            const int schedDepAtTransfer = schedDepTimesDVeh[postTransferIdx];
            const int schedArrAtDropoff = schedArrTimesDVeh[postDropoffIdx];
            KASSERT(schedDepAtTransfer >= arrAtTransferPoint, printRouteOf(asgn.dVeh->vehicleId));
            KASSERT(schedArrAtTransfer + InputConfig::getInstance().stopTime <= schedDepAtTransfer, printRouteOf(asgn.dVeh->vehicleId));
            KASSERT(schedDepAtTransfer == depAtTransferDVeh, printRouteOf(asgn.dVeh->vehicleId));
            KASSERT(schedArrAtDropoff == arrAtDropoff, printRouteOf(asgn.dVeh->vehicleId));

            // Assert that the trip time of the dVeh is correct
            const int waitingTimeAtTransfer = schedDepAtTransfer - arrAtTransferPoint;
            const int actualTripTime = schedArrAtDropoff - schedDepAtTransfer + asgn.dropoff.walkingDist +
                    waitingTimeAtTransfer;
            KASSERT(actualTripTime == tripTimeFromArrivalAtTransfer, printRouteOf(asgn.dVeh->vehicleId));

            // Check distance to transfer
            if (postTransferIdx > 0 && !insertedIntermediateStop && !transferAtExistingStop) {
                KASSERT(schedArrTimesDVeh[postTransferIdx] - schedDepTimesDVeh[postTransferIdx - 1] == asgn.distToTransferDVeh, printRouteOf(asgn.dVeh->vehicleId));
            }

            // Check distance from transfer
            const bool paired = asgn.transferIdxDVeh == asgn.dropoffIdx;
            if (!paired) {
                KASSERT(schedArrTimesDVeh[postTransferIdx + 1] - schedDepTimesDVeh[postTransferIdx] == asgn.distFromTransferDVeh, printRouteOf(asgn.dVeh->vehicleId));
            }

            // Check distance to dropoff
            if (!dropoffAtExistingStop) {
                KASSERT(schedArrTimesDVeh[postDropoffIdx] - schedDepTimesDVeh[postDropoffIdx - 1] == asgn.distToDropoff, printRouteOf(asgn.dVeh->vehicleId));
            }

            // Check distance from dropoff
            if (postDropoffIdx < numStopsDVeh - 1) {
                KASSERT(schedArrTimesDVeh[postDropoffIdx + 1] - schedDepTimesDVeh[postDropoffIdx] == asgn.distFromDropoff, printRouteOf(asgn.dVeh->vehicleId));
            }

            for (int i = 1; i < numStopsDVeh; i++) {
                const auto lastStopLoc = stopLocationsDVeh[i - 1];
                const auto lastStopDep = schedDepTimesDVeh[i - 1];
                const auto stopLoc = stopLocationsDVeh[i];
                const auto stopArr = schedArrTimesDVeh[i];

                KASSERT(lastStopLoc == stopLoc || stopArr > lastStopDep);

                if (!(lastStopLoc == stopLoc || stopArr > lastStopDep)) {
                    KASSERT(false);
                    return false;
                }
            }

            return true;
        }

        bool assertPickupNew(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesPVeh,
                             ConstantVectorRange<int> schedArrTimesPVeh, const int pickupIdx,
                             const int transferIdxPVeh) {
            // const bool bns = asgn.pickupIdx == 0;
            const bool paired = asgn.pickupIdx == asgn.transferIdxPVeh;


            if (!(!paired || pickupIdx + 1 == transferIdxPVeh)) {
                KASSERT(false);
                return false;
            }

            // TODO Find a fix for this condition
            // KASSERT(bns || (pickupIdx > 1 || schedArrTimesPVeh[pickupIdx] - schedDepTimesPVeh[pickupIdx - 1] == asgn.distToPickup));
            // KASSERT(!bns || (pickupIdx >= 0 && pickupIdx <= 2) && (schedArrTimesPVeh[pickupIdx] - schedDepTimesPVeh[0] == asgn.distToPickup));
            // if (!bns && !(pickupIdx > 1 || schedArrTimesPVeh[pickupIdx] - schedDepTimesPVeh[pickupIdx - 1] == asgn.distToPickup)) {
            //     // Pickup is not first stop
            //     return false;
            // } else if (bns && !(pickupIdx >= 0 && pickupIdx <= 2) || !(schedArrTimesPVeh[pickupIdx] - schedDepTimesPVeh[0] == asgn.distToPickup)) {
            //     return false;
            // }

            if (!paired && schedArrTimesPVeh[pickupIdx + 1] - schedDepTimesPVeh[pickupIdx] != asgn.distFromPickup) {
                KASSERT(false);
                return false;
            }

            return true;
        }

        bool assertTransferNewPVeh(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesPVeh,
                                   ConstantVectorRange<int> schedArrTimesPVeh, const int transferIdxPVeh,
                                   const int numStopsPVeh) {
            if (transferIdxPVeh <= 0 ||
                schedArrTimesPVeh[transferIdxPVeh] - schedDepTimesPVeh[transferIdxPVeh - 1] !=
                asgn.distToTransferPVeh) {
                KASSERT(false);
                return false;
            }

            // If the transfer is not als, assert the distance to the next stop
            if (transferIdxPVeh < numStopsPVeh - 1
                && schedArrTimesPVeh[transferIdxPVeh + 1] - schedDepTimesPVeh[transferIdxPVeh] !=
                asgn.distFromTransferPVeh) {
                KASSERT(false);
                return false;
            }

            return true;
        }

        bool assertTransferNewDVeh(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesDVeh,
                                   ConstantVectorRange<int> schedArrTimesDVeh, const int transferIdxDVeh,
                                   const int dropoffIdx) {
            const bool bns = asgn.transferIdxDVeh == 0;
            const bool paired = asgn.transferIdxDVeh == asgn.dropoffIdx;

            if (!(!paired || transferIdxDVeh + 1 == dropoffIdx)) {
                KASSERT(false);
                return false;
            }

            if (!bns && (transferIdxDVeh <= 1 ||
                         schedArrTimesDVeh[transferIdxDVeh] - schedDepTimesDVeh[transferIdxDVeh - 1] !=
                         asgn.distToTransferDVeh)) {
                KASSERT(false);
                return false;
            }

            if (bns && (!(transferIdxDVeh >= 0 && transferIdxDVeh <= 2) ||
                        schedArrTimesDVeh[transferIdxDVeh] - schedDepTimesDVeh[0] != asgn.distToTransferDVeh)) {
                KASSERT(false);
                return false;
            }

            if (!paired && schedArrTimesDVeh[transferIdxDVeh + 1] - schedDepTimesDVeh[transferIdxDVeh] !=
                asgn.distFromTransferDVeh) {
                KASSERT(false);
                return false;
            }

            return true;
        }

        bool assertDropoffNew(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesDVeh,
                              ConstantVectorRange<int> schedArrTimesDVeh, const int dropoffIdx,
                              const int numStopsDVeh) {
            if (dropoffIdx <= 0
                // Assert the distance to the dropoff
                || schedArrTimesDVeh[dropoffIdx] - schedDepTimesDVeh[dropoffIdx - 1] != asgn.distToDropoff) {
                KASSERT(false);
                return false;
            }

            // If the dropoff is not als, assert the distance to the next stop
            if (dropoffIdx < numStopsDVeh - 1 &&
                schedArrTimesDVeh[dropoffIdx + 1] - schedDepTimesDVeh[dropoffIdx] != asgn.distFromDropoff) {
                KASSERT(false);
                return false;
            }

            return true;
        }

        // Scheduled stop interface for event simulation
        struct ScheduledStop {
            int stopId;
            int stopLocation;
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

        std::string printRouteOf(const int vehId) const {
            std::stringstream ss;
            ss << "Route of vehicle " << vehId << ":\n";
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            for (int i = start; i < end; ++i) {
                ss << "  Stop " << stopIds[i] << " at location " << stopLocations[i] << ": arrTime = "
                        << schedArrTimes[i] << ", depTime = " << schedDepTimes[i] << ", maxArrTime = "
                        << maxArrTimes[i] << ", occupancy = " << occupancies[i] << (isIntermediateStop[i]
                            ? " (intermediate)"
                            : "") << "\n";
            }
            return ss.str();
        }

    private:
        ScheduledStop getScheduledStop(const int vehId, const int stopIndex) const {
            KASSERT(numStopsOf(vehId) > stopIndex);
            const auto id = stopIdsFor(vehId)[stopIndex];
            const auto loc = stopLocationsFor(vehId)[stopIndex];
            const auto arrTime = schedArrTimesFor(vehId)[stopIndex];
            const auto depTime = schedDepTimesFor(vehId)[stopIndex];
            const auto occ = occupanciesFor(vehId)[stopIndex];
            const auto pickupsRange = rangeOfRequestsPickedUpAtStop[id];
            const ConstantVectorRange<int> pickups = {
                requestsPickedUpAtStop.begin() + pickupsRange.start,
                requestsPickedUpAtStop.begin() + pickupsRange.end
            };
            const auto dropoffsRange = rangeOfRequestsDroppedOffAtStop[id];
            const ConstantVectorRange<int> dropoffs = {
                requestsDroppedOffAtStop.begin() + dropoffsRange.start,
                requestsDroppedOffAtStop.begin() + dropoffsRange.end
            };
            return {id, loc, arrTime, depTime, occ, pickups, dropoffs};
        }

        int getUnusedStopId() {
            if (!unusedStopIds.empty()) {
                const auto id = unusedStopIds.top();
                unusedStopIds.pop();
                KASSERT(stopIdToVehicleId[id] == INVALID_ID);
                return id;
            }
            ++maxStopId;
            return nextUnusedStopId++;
        }


        //        // Standard forward propagation of changes to minVehArrTime and minVehDepTime from fromIdx to toIdx (both inclusive)
        //        // caused by inserting a pickup stop. Needs distance from stop at fromIdx - 1 to stop at fromIdx because that
        //        // distance cannot be inferred. Indices are direct indices in the 2D arrays.
        //        void propagateSchedArrAndDepForward(const int fromIdx, const int toIdx, const int distFromPrevOfFromIdx) {
        //            KASSERT(distFromPrevOfFromIdx > 0);
        //            int distPrevToCurrent = distFromPrevOfFromIdx;
        //            for (int l = fromIdx; l <= toIdx; ++l) {
        //                schedArrTimes[l] = schedDepTimes[l - 1] + distPrevToCurrent;
        //
        //                // If the planned departure time is already later than the new arrival time demands, then the planned
        //                // departure time remains unaffected and subsequent arrival/departure times will not change either.
        //                if (schedDepTimes[l] >= schedArrTimes[l] + InputConfig::getInstance().stopTime) {
        //                    break;
        //                }
        //
        //                const auto oldMinDepTime = schedDepTimes[l];
        //                schedDepTimes[l] = schedArrTimes[l] +
        //                                   InputConfig::getInstance().stopTime; // = max(schedDepTimes[l], schedArrTimes[l] + stopTime);
        //                if (l < toIdx) distPrevToCurrent = schedArrTimes[l + 1] - oldMinDepTime;
        //            }
        //        }

        // Propagates scheduled arrival and departure times forward from a first stop specified by its ID.
        // Scheduled arrival and departure times of the first stop must already be correct.
        // lengthOfFollowingLeg is required since the distance from the given stop to the following stop can no longer be
        // inferred. If the first stop is the last stop of the route, then lengthOfFollowingLeg may have an arbitrary
        // value.
        void propagateSchedArrAndDepForward(const int firstStopId, int lengthOfFollowingLeg,
                                            Subset &lastStopsWithUpdatedDepTime,
                                            Subset &vehiclesWithChangesInRoute) {
            KASSERT(stopPositionOf(firstStopId) == numStopsOf(vehicleIdOf(firstStopId)) - 1 ||
                (lengthOfFollowingLeg >= 0 && lengthOfFollowingLeg < INFTY));
            static const auto &stopTime = InputConfig::getInstance().stopTime;

            std::vector<std::pair<int, int> > firstStopIdsInRoutesToProcessAndLengthOfNextLeg;
            firstStopIdsInRoutesToProcessAndLengthOfNextLeg.emplace_back(firstStopId, lengthOfFollowingLeg);
            while (!firstStopIdsInRoutesToProcessAndLengthOfNextLeg.empty()) {
                auto [stopId, curLengthOfNextLeg] = firstStopIdsInRoutesToProcessAndLengthOfNextLeg.back();
                firstStopIdsInRoutesToProcessAndLengthOfNextLeg.pop_back();
                const auto vehId = vehicleIdOf(stopId);
                const auto stopIndex = stopPositionOf(stopId);
                const auto startOfVeh = pos[vehId].start;
                const auto endOfVeh = pos[vehId].end;
                vehiclesWithChangesInRoute.insert(vehId);


                for (int i = startOfVeh + stopIndex + 1; i < endOfVeh; ++i) {
                    // Invariant: schedArrTimes[i] and schedDepTimes[i] are already correct for all stops before i.

                    KASSERT(schedArrTimes[i] <= schedDepTimes[i - 1] + curLengthOfNextLeg);
                    schedArrTimes[i] = schedDepTimes[i - 1] + curLengthOfNextLeg;

                    // For any transfers at this stop, the respective dropoff vehicle may be affected.
                    stopId = stopIds[i];
                    for (const auto &dependentStopId: getForwardDependencies(stopId)) {
                        const int dependentVehId = vehicleIdOf(dependentStopId);
                        const int dependentStopIndex = stopPositionOf(dependentStopId);
                        const int j = pos[dependentVehId].start + dependentStopIndex;

                        // If dropoff vehicle leaves transfer later than new arrival time of pickup vehicle, then
                        // no changes are needed in route of dropoff vehicle.
                        if (schedDepTimes[j] >= schedArrTimes[i])
                            continue;

                        const int dependentLengthOfNextLeg = j < pos[dependentVehId].end - 1
                                                                 ? schedArrTimes[j + 1] - schedDepTimes[j]
                                                                 : INFTY;

                        schedDepTimes[j] = schedArrTimes[i];
                        firstStopIdsInRoutesToProcessAndLengthOfNextLeg.emplace_back(dependentStopId,
                            dependentLengthOfNextLeg);

                        if (j == pos[dependentVehId].end - 1)
                            lastStopsWithUpdatedDepTime.insert(dependentStopId);
                    }

                    // If the planned departure time is already later than the new arrival time demands, then the planned
                    // departure time remains unaffected and subsequent arrival/departure times in this route will not
                    // change either.
                    if (schedDepTimes[i] >= schedArrTimes[i] + stopTime)
                        break;

                    curLengthOfNextLeg = i < endOfVeh - 1 ? schedArrTimes[i + 1] - schedDepTimes[i] : INFTY;
                    schedDepTimes[i] = schedArrTimes[i] + stopTime;

                    if (i == endOfVeh - 1)
                        lastStopsWithUpdatedDepTime.insert(stopId);
                }
            }
        }

        //        // Backwards propagation of changes to maxArrTimes from fromIdx down to toIdx
        //        void propagateMaxArrTimeBackward(const int fromIdx, const int toIdx) {
        //            for (int l = fromIdx; l >= toIdx; --l) {
        //                const auto distToNext = schedArrTimes[l + 1] - schedDepTimes[l];
        //                const auto propagatedMaxArrTime = maxArrTimes[l + 1] - distToNext - InputConfig::getInstance().stopTime;
        //                if (maxArrTimes[l] <= propagatedMaxArrTime)
        //                    break; // Stop propagating if known maxArrTime at l is stricter already
        //                maxArrTimes[l] = propagatedMaxArrTime;
        //            }
        //        }

        void propagateMaxArrTimeBackward(const int lastStopId,
                                         Subset &vehiclesWithChangesInRoute) {
            KASSERT(stopPositionOf(lastStopId) < numStopsOf(vehicleIdOf(lastStopId)) - 1);
            static const auto &stopTime = InputConfig::getInstance().stopTime;

            std::vector<std::pair<int, int> > lastStopIdsInRoutesToProcessAndPropagatedMaxArrTime;
            const int lastStopInternalIdx = pos[vehicleIdOf(lastStopId)].start + stopPositionOf(lastStopId);
            const int lastStopPropagatedMaxArrTime = maxArrTimes[lastStopInternalIdx + 1] -
                                                     (schedArrTimes[lastStopInternalIdx + 1] -
                                                      schedDepTimes[lastStopInternalIdx]) - stopTime;
            lastStopIdsInRoutesToProcessAndPropagatedMaxArrTime.emplace_back(lastStopId, lastStopPropagatedMaxArrTime);
            while (!lastStopIdsInRoutesToProcessAndPropagatedMaxArrTime.empty()) {
                auto [stopId, propagatedMaxArrTime] = lastStopIdsInRoutesToProcessAndPropagatedMaxArrTime.back();
                lastStopIdsInRoutesToProcessAndPropagatedMaxArrTime.pop_back();
                const auto vehId = vehicleIdOf(stopId);
                const auto stopIndex = stopPositionOf(stopId);
                const auto startOfVeh = pos[vehId].start;

                for (int i = startOfVeh + stopIndex; i > startOfVeh; --i) {
                    if (maxArrTimes[i] <= propagatedMaxArrTime)
                        break; // Stop propagating if known maxArrTime at i is stricter already

                    maxArrTimes[i] = propagatedMaxArrTime;
                    vehiclesWithChangesInRoute.insert(vehId);

                    // For any transfers at this stop, the respective pickup vehicle may be affected.
                    stopId = stopIds[i];
                    for (const auto &dependentStopId: getBackwardDependencies(stopId)) {
                        // The pickup vehicle needs to arrive at the transfer before the dropoff vehicle has to depart.
                        const int maxDepTime = maxArrTimes[i] + stopTime;
                        lastStopIdsInRoutesToProcessAndPropagatedMaxArrTime.emplace_back(dependentStopId, maxDepTime);
                    }

                    //                    if (i == startOfVeh)
                    //                        break; // First stop of the route, no previous stop to propagate to

                    const auto lengthOfLegFromPrevious = schedArrTimes[i] - schedDepTimes[i - 1];
                    propagatedMaxArrTime = maxArrTimes[i] - lengthOfLegFromPrevious - stopTime;
                }
            }
        }

        void updateLeeways(const int vehId, const int now) {
            unused(now);
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;

            for (int idx = start; idx < end - 1; ++idx) {
                LIGHT_KASSERT(idx <= start + 1 || schedDepTimes[idx] - schedArrTimes[idx] >= InputConfig::getInstance().stopTime);
                const auto &stopId = stopIds[idx];

                // Set leeway of stop, possibly update max leeway
                const auto leeway =
                        std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1]) - schedDepTimes[idx] - InputConfig::getInstance().stopTime;
                // The leeway is only allowed to be negative if the stop at idx 1 is an intermediate stop that has
                // just been created but is not considered reached yet as multiple requests are being inserted at the same time.
                KASSERT((idx == start && schedArrTimes[start + 1] == now) || leeway >= 0);

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
        void recalculateVehWaitTimesPrefixSum(const int fromIdx, const int toIdx, const int baseline,
                                              const bool scheduleHasIntermediateStop) {
            unused(scheduleHasIntermediateStop);
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                auto stopLength = schedDepTimes[l] - InputConfig::getInstance().stopTime - schedArrTimes[l];
                // Stop is only allowed to be shorter than stopTime if it is an intermediate stop. Only the next stop
                // can be an intermediate stop. If it is, set the vehicle wait time at the stop to 0.
                KASSERT(isIntermediateStop[l] || stopLength >= 0,
                        "l = " << l << ", fromIdx = " << fromIdx << ", Route = " << printRouteOf(stopIdToVehicleId[
                            stopIds[l]]));
                stopLength = std::max(stopLength, 0);

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
            for (const auto &[start, end]: pos) {
                for (int idx = start; idx < end - 1; ++idx) {
                    const auto leeway = std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1])
                                        - schedDepTimes[idx] - InputConfig::getInstance().stopTime;
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
            for (const auto &[start, end]: pos) {
                for (int idx = start; idx < end - 1; ++idx) {
                    const auto legLength = schedArrTimes[idx + 1] - schedDepTimes[idx];
                    if (legLength > maxLegLength) {
                        maxLegLength = legLength;
                        stopIdOfMaxLegLength = stopIds[idx];
                    }
                }
            }
        }

        void printStopLocations(const int vehId) {
            std::string sep = "";
            for (int i = 0; i < stopLocationsFor(vehId).size(); ++i) {
                std::cout << sep << i << ": " << stopLocationsFor(vehId)[i];
                sep = ", ";
            }
            std::cout << std::endl;
        }


        // Index Array:

        // For a vehicle with ID vehId, the according entries in each value array lie in the index interval
        // [pos[vehId].start, pos[vehId].end).
        std::vector<dynamic_ragged2d::ValueBlockPosition> pos;

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

        // Flag for whether a stop is an intermediate stop that has been created for a vehicle reroute.
        // Boolean flags stored as uint8_t to avoid std::vector<bool> specialization.
        std::vector<uint8_t> isIntermediateStop;


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
        std::vector<dynamic_ragged2d::ValueBlockPosition> rangeOfRequestsPickedUpAtStop;
        std::vector<int> requestsPickedUpAtStop;
        std::vector<dynamic_ragged2d::ValueBlockPosition> rangeOfRequestsDroppedOffAtStop;
        std::vector<int> requestsDroppedOffAtStop;


        // Other data:

        int maxLeeway;
        int stopIdOfMaxLeeway;

        int maxLegLength;
        int stopIdOfMaxLegLength;

        std::stack<int, std::vector<int> > unusedStopIds;
        int nextUnusedStopId;
        int maxStopId;

        Subset vehiclesWithChangesByThisRequest;

        // Optional callback for verifying direct distances
        DistanceChecker distanceChecker;

        // Information on dependencies between routes introduced by transfers.

        // For a stop s at which a transfer to another vehicle happens,
        // forwardDependenciesStopIds[forwardDependenciesPos[s].start ... forwardDependenciesPos[s].end]
        // defines the IDs of the stops of other vehicles to which a rider transfers.
        // Updates to the scheduled arrival/departure time of s need to be propagated (forwards) to every such stop.
        std::vector<dynamic_ragged2d::ValueBlockPosition> forwardDependenciesPos;
        std::vector<int> forwardDependenciesStopIds;

        // For a stop s at which a rider transfers to this vehicle from another vehicle,
        // backwardDependenciesStopIds[backwardDependenciesPos[s].start ... backwardDependenciesPos[s].end]
        // defines the IDs of the stops of other vehicles from which a rider transfers.
        // Updates to the maximum arrival time of s need to be propagated (backwards) to every such stop.
        std::vector<dynamic_ragged2d::ValueBlockPosition> backwardDependenciesPos;
        std::vector<int> backwardDependenciesStopIds;
    };
}
