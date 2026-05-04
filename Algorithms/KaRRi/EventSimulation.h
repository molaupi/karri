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

#include "DataStructures/Queues/AddressableKHeap.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Tools/Workarounds.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/Timer.h"

namespace karri {


    template<typename AssignmentFinderT,
            typename AssignmentAcceptanceT,
            typename SystemStateUpdaterT,
            typename ScheduledStopsT>
    class EventSimulation {

        enum VehicleState {
            OUT_OF_SERVICE,
            IDLING,
            DRIVING,
            STOPPING
        };

        enum RequestState {
            NOT_RECEIVED,
            ASSIGNED_TO_PVEH, // Assignment with transfer, request is assigned to the pickup vehicle
            ASSIGNED_TO_DVEH, // Assignment with transfer, request is assigned to the pickup vehicle
            ASSIGNED_TO_VEH, // Assigned to vehicle or assigned to dropoff vehicle if the request will be satisfied including a transfer
            WALKING_TO_DEST,
            FINISHED
        };

        // Stores information about assignment and departure time of a request needed for logging on arrival of the
        // request.
        struct RequestData {
            int depTimeAtPickup = INFTY;
            int walkingTimeToPickup = INFTY;
            int walkingTimeFromDropoff = INFTY;
            int assignmentCost = INFTY;

            // Indicate if the request is satisfied using assignment with transfer
            bool usingTransfer = false;

            // Arrival and departure at transfer
            int arrAtTransferPoint = INFTY;
            int depTimeAtTransfer = INFTY;
        };

    public:


        EventSimulation(
                const Fleet &fleet, const std::vector<Request> &requests,
                AssignmentFinderT &assignmentFinder,
                AssignmentAcceptanceT &assignmentAcceptance,
                SystemStateUpdaterT &systemStateUpdater,
                const ScheduledStopsT &scheduledStops,
                const bool verbose = true)
                : fleet(fleet),
                  requests(requests),
                  assignmentFinder(assignmentFinder),
                    assignmentAcceptance(assignmentAcceptance),
                  systemStateUpdater(systemStateUpdater),
                  scheduledStops(scheduledStops),
                  requestEvents(requests.size()),
//                  vehicleEvents(fleet.size()),
                  vehicleStartEvents(fleet.size()),
                  vehicleShutdownEvents(fleet.size()),
                  vehicleArrivalEvents(fleet.size()),
                  vehicleDepartureEvents(fleet.size()),
                  vehiclesWithChangesInRoute(fleet.size()),
                  vehicleState(fleet.size(), OUT_OF_SERVICE),
                  requestState(requests.size(), NOT_RECEIVED),
                  requestData(requests.size(), RequestData()),
                  eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
                                                                                  "occurrence_time,"
                                                                                  "type,"
                                                                                  "running_time\n")),
                  assignmentQualityStatsLogger(LogManager<std::ofstream>::getLogger("assignmentquality.csv",
                                                                                    "request_id,"
                                                                                    "using_transfer,"
                                                                                    "arr_time,"
                                                                                    "wait_time,"
                                                                                    "ride_time,"
                                                                                    "trip_time,"
                                                                                    "walk_to_pickup_time,"
                                                                                    "walk_to_dropoff_time,"
                                                                                    "cost\n")),
                  legStatsLogger(LogManager<std::ofstream>::getLogger("legstats.csv",
                                                                      "vehicle_id,"
                                                                      "stop_time,"
                                                                      "dep_time,"
                                                                      "arr_time,"
                                                                      "drive_time,"
                                                                      "occupancy\n")),
                  progressBar(requests.size(), verbose) {
            progressBar.setDotOutputInterval(1);
            progressBar.setPercentageOutputInterval(5);
            for (const auto &veh: fleet) {
//                vehicleEvents.insert(veh.vehicleId, veh.startOfServiceTime);
                vehicleStartEvents.insert(veh.vehicleId, veh.startOfServiceTime);
            }
            for (const auto &req: requests)
                requestEvents.insert(req.requestId, req.requestTime);
        }

        void run() {

            bool vehicleEventsEmpty = vehicleStartEvents.empty() && vehicleShutdownEvents.empty()
                                      && vehicleArrivalEvents.empty() && vehicleDepartureEvents.empty();
            while (!(vehicleEventsEmpty && requestEvents.empty())) {
                // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
                int id, occTime;


                if (requestEvents.empty()) {
                    handleVehicleEvent();
//                    vehicleEvents.min(id, occTime);
//                    handleVehicleEvent(id, occTime);
                    vehicleEventsEmpty = vehicleStartEvents.empty() && vehicleShutdownEvents.empty()
                                         && vehicleArrivalEvents.empty() && vehicleDepartureEvents.empty();
                    continue;
                }

                if (vehicleEventsEmpty) {
                    requestEvents.min(id, occTime);
                    handleRequestEvent(id, occTime);
                    continue;
                }

                const int nextStartupTime = vehicleStartEvents.empty() ? INFTY : vehicleStartEvents.minKey();
                const int nextShutdownTime = vehicleShutdownEvents.empty() ? INFTY : vehicleShutdownEvents.minKey();
                const int nextArrivalTime = vehicleArrivalEvents.empty() ? INFTY : vehicleArrivalEvents.minKey();
                const int nextDepartureTime = vehicleDepartureEvents.empty() ? INFTY : vehicleDepartureEvents.minKey();
                const int nextVehicleTime = std::min(nextStartupTime,
                                                     std::min(nextShutdownTime,
                                                              std::min(nextArrivalTime,
                                                                       nextDepartureTime)));

                // In the case of a tie, a vehicle startup, shutdown, or arrival take precedence over requests but a
                // request takes precedence over a vehicle departure.
                const int nextRequestTime = requestEvents.minKey();
                if (nextVehicleTime < nextRequestTime || nextStartupTime == nextRequestTime ||
                    nextShutdownTime == nextRequestTime || nextArrivalTime == nextRequestTime) {
                    KASSERT(nextVehicleTime <= nextRequestTime);
//                    vehicleEvents.min(id, occTime);
//                    handleVehicleEvent(id, occTime);
                    handleVehicleEvent();
                    vehicleEventsEmpty = vehicleStartEvents.empty() && vehicleShutdownEvents.empty()
                                         && vehicleArrivalEvents.empty() && vehicleDepartureEvents.empty();
                    continue;
                }

                requestEvents.min(id, occTime);
                handleRequestEvent(id, occTime);
            }
        }

    private:

//        void handleVehicleEvent(const int vehId, const int occTime) {
//            switch (vehicleState[vehId]) {
//                case OUT_OF_SERVICE:
//                    handleVehicleStartup(vehId, occTime);
//                    break;
//                case IDLING:
//                    handleVehicleShutdown(vehId, occTime);
//                    break;
//                case DRIVING:
//                    handleVehicleArrivalAtStop(vehId, occTime);
//                    break;
//                case STOPPING:
//                    handleVehicleDepartureFromStop(vehId, occTime);
//                    break;
//                default:
//                    break;
//            }
//        }

        void handleVehicleEvent() {
            int id, occTime;

            const int nextStartupTime = vehicleStartEvents.empty() ? INFTY : vehicleStartEvents.minKey();
            const int nextShutdownTime = vehicleShutdownEvents.empty() ? INFTY : vehicleShutdownEvents.minKey();
            const int nextArrivalTime = vehicleArrivalEvents.empty() ? INFTY : vehicleArrivalEvents.minKey();
            const int nextDepartureTime = vehicleDepartureEvents.empty() ? INFTY : vehicleDepartureEvents.minKey();

            if (nextStartupTime <= nextShutdownTime && nextStartupTime <= nextArrivalTime &&
                nextStartupTime <= nextDepartureTime) {
                vehicleStartEvents.deleteMin(id, occTime);
                handleVehicleStartup(id, occTime);
                return;
            }

            if (nextArrivalTime < nextStartupTime && nextArrivalTime <= nextDepartureTime &&
                nextArrivalTime <= nextShutdownTime) {
                vehicleArrivalEvents.deleteMin(id, occTime);
                handleVehicleArrivalAtStop(id, occTime);
                return;
            }

            if (nextDepartureTime < nextStartupTime && nextDepartureTime < nextArrivalTime &&
                nextDepartureTime <= nextShutdownTime) {
                vehicleDepartureEvents.deleteMin(id, occTime);
                handleVehicleDepartureFromStop(id, occTime);
                return;
            }

            KASSERT(nextShutdownTime < nextStartupTime && nextShutdownTime < nextArrivalTime &&
                    nextShutdownTime < nextDepartureTime);
            vehicleShutdownEvents.deleteMin(id, occTime);
            KASSERT(!vehicleStartEvents.contains(id) && !vehicleArrivalEvents.contains(id) &&
                    !vehicleDepartureEvents.contains(id));
            handleVehicleShutdown(id, occTime);
        }

        void handleRequestEvent(const int reqId, const int occTime) {
            switch (requestState[reqId]) {
                case NOT_RECEIVED:
                    handleRequestReceipt(reqId, occTime);
                    break;

                case ASSIGNED_TO_VEH:
                case ASSIGNED_TO_DVEH:
                case ASSIGNED_TO_PVEH:
                    // When assigned to a vehicle, there should be no request event until the dropoff.
                    // At that point the request state becomes WALKING_TO_DEST.
                    KASSERT(false);
                    break;
                case WALKING_TO_DEST:
                    handleWalkingArrivalAtDest(reqId, occTime);
                    break;
                case FINISHED:
                    KASSERT(false);
                    break;
                default:
                    break;
            }
        }

        void handleVehicleStartup(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == OUT_OF_SERVICE);
            KASSERT(fleet[vehId].startOfServiceTime == occTime);
            unused(occTime);
            Timer timer;

            // Every vehicle gets an event in vehicleShutdownEvents.
            vehicleShutdownEvents.insert(vehId, fleet[vehId].endOfServiceTime);

            // Vehicle may have already been assigned stops. In this case it will start driving right away:
            if (scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = DRIVING;
                vehicleArrivalEvents.insert(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
//                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            } else {
                // An idling vehicle has no events other than its shutdown event.
                vehicleState[vehId] = IDLING;
//                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleStartup," << time << '\n';
        }

        void handleVehicleShutdown(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == IDLING);
            KASSERT(fleet[vehId].endOfServiceTime == occTime);
            unused(occTime);
            KASSERT(!scheduledStops.hasNextScheduledStop(vehId));
            Timer timer;

            vehicleState[vehId] = OUT_OF_SERVICE;

//            int id, key;
////            vehicleEvents.deleteMin(id, key);
//            KASSERT(id == vehId && key == occTime);
            systemStateUpdater.notifyVehicleReachedEndOfServiceTime(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleShutdown," << time << '\n';
        }

        void handleVehicleArrivalAtStop(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == DRIVING);
            KASSERT(scheduledStops.getNextScheduledStop(vehId).arrTime == occTime);
            Timer timer;

            const auto prevStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
            legStatsLogger << vehId << ','
                           << prevStop.depTime - prevStop.arrTime << ','
                           << prevStop.depTime << ','
                           << occTime << ','
                           << occTime - prevStop.depTime << ','
                           << prevStop.occupancyInFollowingLeg << '\n';

            vehicleState[vehId] = STOPPING;
            const auto reachedStop = scheduledStops.getNextScheduledStop(vehId);

            // Handle dropoffs at reached stop: Insert walking arrival event at the time when passenger will arrive at
            // destination. Thus, all requests are logged in the order of the arrival at their destination.
            for (const auto &reqId: reachedStop.requestsDroppedOffHere) {
                const auto &reqData = requestData[reqId];

                if (requestState[reqId] == ASSIGNED_TO_PVEH) {
                    requestData[reqId].arrAtTransferPoint = occTime;
                    requestState[reqId] = ASSIGNED_TO_DVEH;
                } else {
                    requestState[reqId] = WALKING_TO_DEST;
                    requestEvents.insert(reqId, occTime + reqData.walkingTimeFromDropoff);
                }
            }

            // Next event for this vehicle is the departure at this stop:
//            vehicleEvents.increaseKey(vehId, reachedStop.depTime);
            vehicleDepartureEvents.insert(vehId, reachedStop.depTime);
            systemStateUpdater.notifyStopStarted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleArrival," << time << '\n';
        }

        void handleVehicleDepartureFromStop(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == STOPPING);
            KASSERT(scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime == occTime);
            Timer timer;

            if (!scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = IDLING;
//                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
//                vehicleShutdownEvents.insert(vehId, fleet[vehId].endOfServiceTime);
            } else {
                // Remember departure time for all requests picked up at this stop:
                const auto curStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);

                for (const auto &reqId: curStop.requestsPickedUpHere) {
                    // If this is the pickup vehicle of a rider, mark their departure time.
                    if (requestState[reqId] == ASSIGNED_TO_VEH || requestState[reqId] == ASSIGNED_TO_PVEH) {
                        requestData[reqId].depTimeAtPickup = occTime;
                    }

                    // If this is the dropoff vehicle of a rider, mark their departure time at the transfer point.
                    if (requestState[reqId] == ASSIGNED_TO_DVEH) {
                        requestData[reqId].depTimeAtTransfer = occTime;
                    }
                }
                vehicleState[vehId] = DRIVING;
//                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                vehicleArrivalEvents.insert(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            }

            systemStateUpdater.notifyStopCompleted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void handleRequestReceipt(const int reqId, const int occTime) {
            ++progressBar;
            KASSERT(requestState[reqId] == NOT_RECEIVED);
            KASSERT(requests[reqId].requestTime == occTime);
            Timer timer;

            const auto &request = requests[reqId];
            const auto &asgnFinderResponse = assignmentFinder.findBestAssignment(request);

            const bool accepted = assignmentAcceptance.doesRiderAcceptAssignment(request, asgnFinderResponse);
            systemStateUpdater.writeBestAssignmentToLogger(accepted);
            if (!accepted) {
                requestState[reqId] = FINISHED;
                int id, key;
                requestEvents.deleteMin(id, key);
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            if (asgnFinderResponse.improvementThroughTransfer()) {
                applyAssignmentWithTransfer(asgnFinderResponse.getBestAssignmentWithTransfer(),
                                            asgnFinderResponse.getBestCostWithTransfer(), reqId);
            } else {
                applyAssignment(asgnFinderResponse, reqId, occTime);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        template<typename AssignmentWithTransferT>
        void applyAssignmentWithTransfer(const AssignmentWithTransferT &asgn, const int &cost, const int reqId) {
            if (!asgn.dVeh || !asgn.pVeh || !asgn.pickup || !asgn.dropoff) {
                requestState[reqId] = FINISHED;
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            int id, key;
            requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff

            requestState[reqId] = ASSIGNED_TO_PVEH;
            requestData[reqId].walkingTimeToPickup = asgn.pickup->walkingDist;
            requestData[reqId].walkingTimeFromDropoff = asgn.dropoff->walkingDist;
            requestData[reqId].assignmentCost = cost;
            requestData[reqId].usingTransfer = true;

            int pickupStopId, transferStopIdPVeh, transferStopIdDVeh, dropoffStopId;
            vehiclesWithChangesInRoute.clear();
            systemStateUpdater.insertBestAssignmentWithTransfer(asgn, pickupStopId, transferStopIdPVeh,
                                                                transferStopIdDVeh, dropoffStopId, vehiclesWithChangesInRoute);
            systemStateUpdater.writePerformanceLogs();

            KASSERT(vehiclesWithChangesInRoute.contains(asgn.pVeh->vehicleId));
            KASSERT(vehiclesWithChangesInRoute.contains(asgn.dVeh->vehicleId));
            for (const auto &vehId: vehiclesWithChangesInRoute) {
                switch (vehicleState[vehId]) {
                    case STOPPING:
                        // Update event time to departure time at current stop since it may have changed
                        vehicleDepartureEvents.updateKey(vehId,
                                                         scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime);
                        break;
                    case IDLING:
                        // If vehicle was idling it is now driving and gets an arrival event at the next stop.
                        KASSERT(vehId == asgn.pVeh->vehicleId || vehId == asgn.dVeh->vehicleId);
                        vehicleState[vehId] = VehicleState::DRIVING;
                        vehicleArrivalEvents.insert(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                        break;
                    case DRIVING:
                        // Update event time to arrival time at next stop since it may have changed.
                        vehicleArrivalEvents.updateKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                        break;
                    default:
                        break;
                }
            }
        }

        template<typename AssignmentFinderResponseT>
        void applyAssignment(const AssignmentFinderResponseT &asgnFinderResponse, const int reqId, const int occTime) {
            if (asgnFinderResponse.isNotUsingVehicleBest()) {
                requestState[reqId] = WALKING_TO_DEST;
                requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();
                requestData[reqId].depTimeAtPickup = occTime;
                requestData[reqId].walkingTimeToPickup = 0;
                requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.getNotUsingVehicleDist();
                requestData[reqId].usingTransfer = false;
                requestData[reqId].arrAtTransferPoint = INFTY;
                requestData[reqId].depTimeAtTransfer = INFTY;
                requestEvents.increaseKey(reqId, occTime + asgnFinderResponse.getNotUsingVehicleDist());
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            int id, key;
            requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff
            KASSERT(id == reqId && key == occTime);

            const auto &bestAsgn = asgnFinderResponse.getBestAssignmentWithoutTransfer();
            if (!bestAsgn.vehicle || !bestAsgn.pickup || !bestAsgn.dropoff) {
                requestState[reqId] = FINISHED;
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            requestState[reqId] = ASSIGNED_TO_VEH;
            requestData[reqId].walkingTimeToPickup = bestAsgn.pickup->walkingDist;
            requestData[reqId].walkingTimeFromDropoff = bestAsgn.dropoff->walkingDist;
            requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();

            int pickupStopId, dropoffStopId;
            vehiclesWithChangesInRoute.clear();
            systemStateUpdater.insertBestAssignment(pickupStopId, dropoffStopId, vehiclesWithChangesInRoute);
            systemStateUpdater.writePerformanceLogs();
            KASSERT(pickupStopId >= 0 && dropoffStopId >= 0);

            KASSERT(vehiclesWithChangesInRoute.contains(bestAsgn.vehicle->vehicleId));
            for (const auto &vehId: vehiclesWithChangesInRoute) {
                switch (vehicleState[vehId]) {
                    case STOPPING:
                        // Update event time to departure time at current stop since it may have changed
                        vehicleDepartureEvents.updateKey(vehId,
                                                         scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime);
                        break;
                    case IDLING:
                        // If vehicle was idling it is now driving and gets an arrival event at the next stop.
                        KASSERT(vehId == bestAsgn.vehicle->vehicleId);
                        vehicleState[vehId] = VehicleState::DRIVING;
                        vehicleArrivalEvents.insert(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                        break;
                    case DRIVING:
                        // Update event time to arrival time at next stop since it may have changed.
                        vehicleArrivalEvents.updateKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                        break;
                    default:
                        break;
                }
            }
        }

        void handleWalkingArrivalAtDest(const int reqId, const int occTime) {
            KASSERT(requestState[reqId] == WALKING_TO_DEST);
            Timer timer;

            const RequestData &reqData = requestData[reqId];
            requestState[reqId] = FINISHED;
            int id, key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key == occTime);

            int waitTime;
            int arrTime;
            int rideTime;
            int tripTime;
            if (!reqData.usingTransfer) {
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                waitTime = reqData.depTimeAtPickup - requests[reqId].requestTime;
                rideTime = occTime - reqData.walkingTimeFromDropoff - reqData.depTimeAtPickup;
            } else {
                // Calculate the values if the assignment consists of two vehicles
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                int waitAtTransfer = reqData.depTimeAtTransfer - reqData.arrAtTransferPoint;
                KASSERT(waitAtTransfer >= 0);
                waitTime = reqData.depTimeAtPickup - requests[reqId].requestTime + waitAtTransfer;
                rideTime = arrTime - reqData.walkingTimeFromDropoff - reqData.depTimeAtPickup - waitAtTransfer;
            }

            KASSERT(waitTime >= 0 && rideTime >= 0);

            assignmentQualityStatsLogger << reqId << ','
                                         << reqData.usingTransfer << ","
                                         << arrTime << ','
                                         << waitTime << ','
                                         << rideTime << ','
                                         << tripTime << ','
                                         << reqData.walkingTimeToPickup << ','
                                         << reqData.walkingTimeFromDropoff << ','
                                         << reqData.assignmentCost << '\n';

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestWalkingArrival," << time << '\n';
        }


        const Fleet &fleet;
        const std::vector<Request> &requests;
        AssignmentFinderT &assignmentFinder;
        AssignmentAcceptanceT &assignmentAcceptance;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;

        AddressableQuadHeap requestEvents;
//        AddressableQuadHeap vehicleEvents;

        // Every vehicle is initialized to only have a start event. Once the start is reached, a shutdown event is
        // added which is active until the end of the service time.
        // An idle vehicle has no event other than the shutdown event.
        // A vehicle that is currently driving has an arrival event but no departure event.
        // A vehicle that is currently stopping has a departure event but no arrival event.
        AddressableQuadHeap vehicleStartEvents;
        AddressableQuadHeap vehicleShutdownEvents;
        AddressableQuadHeap vehicleArrivalEvents;
        AddressableQuadHeap vehicleDepartureEvents;

        Subset vehiclesWithChangesInRoute;

        std::vector<VehicleState> vehicleState;
        std::vector<RequestState> requestState;

        std::vector<RequestData> requestData;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &assignmentQualityStatsLogger;
        std::ofstream &legStatsLogger;
        ProgressBar progressBar;

    };
}