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
#include "RiderModeChoice/PTJourneyData.h"
#include "RiderModeChoice/TaxiResult.h"
#include "RiderModeChoice/TaxiResultConstructor.h"
#include "RiderModeChoice/TransportMode.h"
#include "Tools/Workarounds.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/Timer.h"
#include "RiderModeChoice/TransportMode.h"
#include "Algorithms/KaRRi/RiderModeChoice/PTJourneyData.h"

namespace karri {
    template<typename AssignmentFinderT,
    typename WalkingTripFinderT,
    typename CarTripFinderT,
    typename TaxiResultConstructorT,
        typename RiderModeChoiceT,
        typename SystemStateUpdaterT,
        typename ScheduledStopsT,
        bool BATCHED_DISPATCHING>
    class EventSimulation {
        enum VehicleState {
            OUT_OF_SERVICE,
            IDLING,
            DRIVING,
            STOPPING
        };

        enum RiderState {
            NOT_RECEIVED,
            WAITING_FOR_DISPATCH,
            ASSIGNED_TO_PVEH, // Assignment with transfer, request is assigned to the pickup vehicle
            ASSIGNED_TO_DVEH, // Assignment with transfer, request is assigned to the pickup vehicle
            ASSIGNED_TO_VEH,
            // Assigned to vehicle or assigned to dropoff vehicle if the request will be satisfied including a transfer
            WALKING_FROM_DROPOFF_TO_DESTINATION,
            OTHER_MODE_TO_DESTINATION,
            FINISHED
        };

        // Stores information about assignment and departure time of a request needed for logging on arrival of the
        // request.
        struct RequestData {
            int depTimeAtPickup;
            int walkingTimeToPickup;
            int walkingTimeFromDropoff;
            int assignmentCost;
            int directOdDist;

            // Indicate if the request is satisfied using assignment with transfer
            bool usingTransfer = false;

            // Arrival and departure at transfer
            int arrAtTransferPoint = INFTY;
            int depTimeAtTransfer = INFTY;
        };

        // Key for event queues.
        struct EntityKey {
            int time = INFTY;
            int id = INVALID_ID;

            auto operator<=>(const EntityKey &) const = default;
        };

    public:
        EventSimulation(
            const Fleet &fleet, const std::vector<Request> &requests,
            const std::vector<mode_choice::PTJourneyData> &ptJourneyData,
            AssignmentFinderT &assignmentFinder,
            WalkingTripFinderT &walkingTripFinder,
            CarTripFinderT &carTripFinder,
            TaxiResultConstructorT &taxiResultConstructor,
            RiderModeChoiceT &riderModeChoice,
            SystemStateUpdaterT &systemStateUpdater,
            const ScheduledStopsT &scheduledStops,
            const bool verbose = false)
            : fleet(fleet),
              requests(requests),
              ptJourneyData(ptJourneyData),
              assignmentFinder(assignmentFinder),
        walkingTripFinder(walkingTripFinder),
        carTripFinder(carTripFinder),
        taxiResultConstructor(taxiResultConstructor),
              riderModeChoice(riderModeChoice),
              systemStateUpdater(systemStateUpdater),
              scheduledStops(scheduledStops),
              vehicleEvents(fleet.size()),
              requestEvents(requests.size()),
              nextRequestBatchDeadline(InputConfig::getInstance().requestBatchInterval),
              batchId(-1),
              requestBatch(),
        vehiclesWithChangesInRoute(fleet.size()),
              vehicleState(fleet.size(), OUT_OF_SERVICE),
              riderState(requests.size(), NOT_RECEIVED),
              requestData(requests.size(), RequestData()),
              eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
                                                                              "occurrence_time,"
                                                                              "type,"
                                                                              "running_time\n")),
              batchDispatchStatsLogger(LogManager<std::ofstream>::getLogger("batchdispatchstats.csv",
                                                                            "occurence_time,"
                                                                            "is_single_request,"
                                                                            "batch_id,"
                                                                            "iteration,"
                                                                            "num_requests,"
                                                                            "num_accepted,"
                                                                            "find_assignments_running_time,"
                                                                            "choose_accepted_running_time,"
                                                                            "num_elliptic_bucket_entry_deletions,"
                                                                            "update_system_state_running_time\n")),
              assignmentQualityStats(LogManager<std::ofstream>::getLogger("assignmentquality.csv",
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
                                                                  "from_edge,"
                                                                  "to_edge,"
                                                                  "stop_time,"
                                                                  "dep_time,"
                                                                  "arr_time,"
                                                                  "drive_time,"
                                                                  "occupancy\n")),
        requestDispatchedSingleLogger(LogManager<std::ofstream>::getLogger("requestsdispatchedsingle.csv", "request_id\n")),
              progressBar(requests.size(), verbose) {
            progressBar.setDotOutputInterval(1);
            progressBar.setPercentageOutputInterval(5);
            for (const auto &veh: fleet)
                vehicleEvents.insert(veh.vehicleId, {veh.startOfServiceTime, veh.vehicleId});
            for (const auto &req: requests)
                requestEvents.insert(req.requestId, {req.requestTime, req.requestId});
        }

        void run() {
            static int prevReqTime = -1;

            while (!(vehicleEvents.empty() && requestEvents.empty())) {
                // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
                int id;
                EntityKey key;
                const bool nextEventIsRequest =
                        !requestEvents.empty() &&
                        (vehicleEvents.empty() || requestEvents.minKey().time <= vehicleEvents.minKey().time);

                if (nextEventIsRequest)
                    requestEvents.min(id, key);
                else
                    vehicleEvents.min(id, key);

                const int occTime = key.time;

                if constexpr (BATCHED_DISPATCHING) {
                    // If the deadline for the next request batch has been reached, dispatch the batch of requests
                    // collected before continuing with the next request or vehicle events.
                    if ((InputConfig::getInstance().requestBatchInterval > 0 && occTime > nextRequestBatchDeadline) || (
                            InputConfig::getInstance().requestBatchInterval == 0 && !requestBatch.empty())) {
                        KASSERT(InputConfig::getInstance().requestBatchInterval > 0 || requestBatch.size() == 1);
                        KASSERT(
                            InputConfig::getInstance().requestBatchInterval > 0 || requestBatch[0].requestTime ==
                            prevReqTime);
                        dispatchRequestBatch(InputConfig::getInstance().requestBatchInterval == 0
                                                 ? prevReqTime
                                                 : nextRequestBatchDeadline);
                        continue;
                    }
                }

                if (nextEventIsRequest) {
                    handleRequestEvent(id, occTime);
                    prevReqTime = occTime;
                } else
                    handleVehicleEvent(id, occTime);
            }
        }

    private:
        void handleVehicleEvent(const int vehId, const int occTime) {
            switch (vehicleState[vehId]) {
                case OUT_OF_SERVICE:
                    handleVehicleStartup(vehId, occTime);
                    break;
                case IDLING:
                    handleVehicleShutdown(vehId, occTime);
                    break;
                case DRIVING:
                    handleVehicleArrivalAtStop(vehId, occTime);
                    break;
                case STOPPING:
                    handleVehicleDepartureFromStop(vehId, occTime);
                    break;
                default:
                    break;
            }
        }

        void handleRequestEvent(const int reqId, const int occTime) {
            switch (riderState[reqId]) {
                case NOT_RECEIVED:
                    handleRequestReceipt(reqId, occTime);
                    break;
                case WAITING_FOR_DISPATCH:
                    // // debug code for testing delayed requests in sequential dispatching
                    // if constexpr (!BATCHED_DISPATCHING) {
                    //     int id;
                    //     EntityKey key;
                    //     requestEvents.deleteMin(id, key);
                    //     KASSERT(reqId == id && key.time == occTime);
                    //     dispatchSingleRequest(requests[reqId], occTime);
                    //     break;
                    // }
                    KASSERT(false);
                    break;
                case ASSIGNED_TO_VEH:
                case ASSIGNED_TO_DVEH:
                case ASSIGNED_TO_PVEH:
                    // When assigned to a vehicle, there should be no request event until the dropoff.
                    // At that point the request state becomes OTHER_MODE_TO_DESTINATION.
                    KASSERT(false);
                    break;
                case WALKING_FROM_DROPOFF_TO_DESTINATION:
                    handleWalkingArrivalAtDest(reqId, occTime);
                    break;
                case OTHER_MODE_TO_DESTINATION:
                    handleOtherModeArrivalAtDest(reqId, occTime);
                    break;
                case FINISHED:
                    KASSERT(false);
                    break;
                default:
                    assert(false);
                    break;
            }
        }

        void handleVehicleStartup(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == OUT_OF_SERVICE);
            KASSERT(fleet[vehId].startOfServiceTime == occTime);
            unused(occTime);
            Timer timer;

            // Vehicle may have already been assigned stops. In this case it will start driving right away:
            if (scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = DRIVING;
                vehicleEvents.increaseKey(vehId, {scheduledStops.getNextScheduledStop(vehId).arrTime, vehId});
            } else {
                // An idling vehicle has no events other than its shutdown event.
                vehicleState[vehId] = IDLING;
                vehicleEvents.increaseKey(vehId, {fleet[vehId].endOfServiceTime, vehId});
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

            int id;
            EntityKey key;
            vehicleEvents.deleteMin(id, key);
            assert(id == vehId && key.time == occTime);
            systemStateUpdater.notifyVehicleReachedEndOfServiceTime(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleShutdown," << time << '\n';
        }

        void handleVehicleArrivalAtStop(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == DRIVING);
            KASSERT(scheduledStops.getNextScheduledStop(vehId).arrTime == occTime);
            Timer timer;

            const auto prevStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
            const auto reachedStop = scheduledStops.getNextScheduledStop(vehId);
            legStatsLogger << vehId << ',' << prevStop.stopLocation << ',' << reachedStop.stopLocation << ','
                    << prevStop.depTime - prevStop.arrTime << ','
                    << prevStop.depTime << ','
                    << reachedStop.arrTime << ','
                    << reachedStop.arrTime - prevStop.depTime << ','
                    << prevStop.occupancyInFollowingLeg << '\n';

            vehicleState[vehId] = STOPPING;

            // Handle dropoffs at reached stop: Insert walking arrival event at the time when passenger will arrive at
            // destination. Thus, all requests are logged in the order of the arrival at their destination.
            for (const auto &reqId: reachedStop.requestsDroppedOffHere) {
                const auto &reqData = requestData[reqId];
                if (riderState[reqId] == ASSIGNED_TO_PVEH) {
                    requestData[reqId].arrAtTransferPoint = occTime;
                    riderState[reqId] = ASSIGNED_TO_DVEH;
                } else {
                    riderState[reqId] = WALKING_FROM_DROPOFF_TO_DESTINATION;
                    requestEvents.insert(reqId, {occTime + reqData.walkingTimeFromDropoff, reqId});
                }
            }

            // Next event for this vehicle is the departure at this stop:
            vehicleEvents.increaseKey(vehId, {reachedStop.depTime, vehId});
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
                vehicleEvents.increaseKey(vehId, {fleet[vehId].endOfServiceTime, vehId});
            } else {
                // Remember departure time for all requests picked up at this stop:
                const auto curStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);

                for (const auto &reqId: curStop.requestsPickedUpHere) {
                    // If this is the pickup vehicle of a rider, mark their departure time.
                    if (riderState[reqId] == ASSIGNED_TO_VEH || riderState[reqId] == ASSIGNED_TO_PVEH) {
                        requestData[reqId].depTimeAtPickup = occTime;
                    }

                    // If this is the dropoff vehicle of a rider, mark their departure time at the transfer point.
                    if (riderState[reqId] == ASSIGNED_TO_DVEH) {
                        requestData[reqId].depTimeAtTransfer = occTime;
                    }
                }
                vehicleState[vehId] = DRIVING;
                vehicleEvents.increaseKey(vehId, {scheduledStops.getNextScheduledStop(vehId).arrTime, vehId});
            }

            systemStateUpdater.notifyStopCompleted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void handleRequestReceipt(const int reqId, const int occTime) {
            KASSERT(riderState[reqId] == NOT_RECEIVED);
            KASSERT(requests[reqId].requestTime == occTime);

            Timer timer;

            riderState[reqId] = WAITING_FOR_DISPATCH;
            if constexpr (BATCHED_DISPATCHING) {
                // event for walking arrival at dest inserted at dispatching time for walking-only or when vehicle reaches dropoff
                int id;
                EntityKey key;
                requestEvents.deleteMin(id, key);
                assert(id == reqId && key.time == occTime);

                // Every sampleSingleFrequency-th request is dispatched individually, all others are added to the current batch.;
                if (InputConfig::getInstance().sampleSingleFrequency != 0 && reqId > 0 && reqId % InputConfig::getInstance().sampleSingleFrequency == 0) {
                    dispatchSingleRequest(requests[reqId], occTime);
                } else {
                    // Add to current request batch for later dispatching
                    requestBatch.push_back(requests[reqId]);
                }
            } else {
                // // Dispatch immediately
                int id;
                EntityKey key;
                requestEvents.deleteMin(id, key);
                assert(id == reqId && key.time == occTime);
                dispatchSingleRequest(requests[reqId], occTime);

                // // debug code for testing delayed requests in sequential dispatching
                // requestEvents.increaseKey(reqId, {occTime + InputConfig::getInstance().requestBatchInterval, reqId});
                // // requestEvents.increaseKey(reqId, {occTime + 10, reqId});
            }
// =======
//             ++progressBar;
//             KASSERT(riderState[reqId] == NOT_RECEIVED);
//             KASSERT(requests[reqId].requestTime == occTime);
//             Timer timer;
//
//
//             int id, key;
//             requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff
//             KASSERT(id == reqId && key == occTime);
//
//             const auto &request = requests[reqId];
//             auto asgnFinderResponse = assignmentFinder.findBestAssignment(request);
//             const auto walkingResult = walkingTripFinder.findWalkingTrip(
//                 asgnFinderResponse, asgnFinderResponse.stats().initializationStats);
//             const auto carResult = carTripFinder.findCarTrip(asgnFinderResponse);
//             const auto taxiResult = taxiResultConstructor.constructTaxiResult(asgnFinderResponse);
//             const auto &ptData = ptJourneyData[reqId];
//
//             const auto mode = modeChoice.chooseMode(asgnFinderResponse, walkingResult, carResult, taxiResult,
//                                                     ptJourneyData[reqId]);
//             systemStateUpdater.writeBestAssignmentToLogger(mode == mode_choice::TransportMode::Taxi);
//
//             using mode_choice::TransportMode;
//             if (mode == TransportMode::Ped || mode == TransportMode::Car) {
//                 const int arrTime = mode == TransportMode::Ped
//                                         ? request.requestTime + walkingResult.walkingDist
//                                         : request.requestTime + carResult.carDist;
//                 processChoiceOtherMode(reqId, occTime, arrTime);
//             } else if (mode == TransportMode::PublicTransport) {
//                 const int arrTime = request.requestTime + ptData.totalJourneyTimeTenthsOfSeconds();
//                 processChoiceOtherMode(reqId, occTime, arrTime);
//             } else if (mode == TransportMode::Taxi) {
//                 if (asgnFinderResponse.improvementThroughTransfer()) {
//                     applyAssignmentWithTransfer(asgnFinderResponse.getBestAssignmentWithTransfer(),
//                                                 asgnFinderResponse.getBestCostWithTransfer(), reqId);
//                 } else {
//                     applyAssignment(asgnFinderResponse, reqId, occTime);
//                 }
//             } else if (mode == TransportMode::None) {
//                 processNoMode(reqId);
//             } else {
//                 KASSERT(false);
//             }
//
//             // const bool accepted = assignmentAcceptance.doesRiderAcceptAssignment(request, asgnFinderResponse);
//             // if (!accepted) {
//             //     riderState[reqId] = FINISHED;
//             //     int id, key;
//             //     requestEvents.deleteMin(id, key);
//             //     systemStateUpdater.writePerformanceLogs();
//             //     return;
//             // }
//             //
//             // if (asgnFinderResponse.improvementThroughTransfer()) {
//             //     applyAssignmentWithTransfer(asgnFinderResponse.getBestAssignmentWithTransfer(),
//             //                                 asgnFinderResponse.getBestCostWithTransfer(), reqId);
//             // } else {
//             //     applyAssignment(asgnFinderResponse, reqId, occTime);
//             // }
// >>>>>>> karrit

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        void dispatchSingleRequest(const Request &request, const int now) {
            using mode_choice::TransportMode;
            stats::DispatchingPerformanceStats stats;
            Timer overallTimer;
            Timer componentTimer;

            int numEllipticBucketEntryDeletions = 0;
            if constexpr (BATCHED_DISPATCHING) {
                // If we are running batched dispatching and calling this method to sample sequential running times,
                // we need to commit all pending updates before searching for the assignment. If we are running fully
                // sequential, these updates are commited as soon as they occur, so there are no pending updates at this point.
                // Commit all pending deletions to the elliptic buckets as well as insertions and deletions to the
                // last stop buckets that were caused by vehicles progressing in their routes since the last batch.
                numEllipticBucketEntryDeletions = systemStateUpdater.numPendingEllipticBucketEntryDeletions();
                systemStateUpdater.commitPendingEllipticBucketEntryDeletions();
                systemStateUpdater.commitPendingLastStopBucketEntryInsertionsAndDeletions();
            }

            componentTimer.restart();
            const auto asgnFinderResponse = assignmentFinder.findBestAssignment(request, now, stats);
            const auto walkingResult = walkingTripFinder.findWalkingTrip(
                asgnFinderResponse, stats.initializationStats);
            const auto carResult = carTripFinder.findCarTrip(asgnFinderResponse);
            const auto taxiResult = taxiResultConstructor.constructTaxiResult(asgnFinderResponse);
            const auto &ptData = ptJourneyData[request.requestId];
            const auto findAssignmentTime = componentTimer.elapsed<std::chrono::nanoseconds>();

            componentTimer.restart();
            const auto mode = riderModeChoice.chooseMode(asgnFinderResponse, walkingResult, carResult, taxiResult, ptData);
            const auto chooseAcceptedTime = componentTimer.elapsed<std::chrono::nanoseconds>();

            componentTimer.restart();
            systemStateUpdater.writeBestAssignmentToLogger(mode, asgnFinderResponse, stats.costStats);
            if (mode == TransportMode::Taxi) {
                vehiclesWithChangesInRoute.clear();
                systemStateUpdater.insertSingleBestAssignment(asgnFinderResponse, vehiclesWithChangesInRoute, stats);
                updateRequestData(asgnFinderResponse);
                updateSimulationForAffectedVehicles(vehiclesWithChangesInRoute);
            } else {
                if (mode == TransportMode::Ped) {
                    processChoiceOnlyWalking(asgnFinderResponse, stats, request.requestId, now);
                } else if (mode == TransportMode::Car) {
                    processChoiceOtherMode(asgnFinderResponse, stats, request.requestId, now,
                                           now + asgnFinderResponse.originalReqDirectDist);
                } else if (mode == TransportMode::PublicTransport) {
                    processChoiceOtherMode(asgnFinderResponse, stats, request.requestId, now,
                                           now + ptJourneyData[request.requestId].totalJourneyTimeTenthsOfSeconds());
                } else if (mode == TransportMode::None) {
                    processNoMode(asgnFinderResponse, stats, request.requestId);
                } else {
                    throw std::runtime_error("Unsupported transport mode chosen in event simulation.");
                }
            }
            const auto updateSystemStateTime = componentTimer.elapsed<std::chrono::nanoseconds>();

            ++progressBar;

            ++batchId;
            batchDispatchStatsLogger << now << "," << "1" << "," << batchId << "," << 1 << "," << 1 << "," << 1 << ","
                    << findAssignmentTime << ","
                    << chooseAcceptedTime << ","
                    << numEllipticBucketEntryDeletions << ","
                    << updateSystemStateTime << '\n';

            requestDispatchedSingleLogger << request.requestId << '\n';

            const auto time = overallTimer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << now << ",RequestBatchDispatch," << time << '\n';
        }

        void dispatchRequestBatch(const int now) requires BATCHED_DISPATCHING {
            nextRequestBatchDeadline += InputConfig::getInstance().requestBatchInterval;

            if (requestBatch.empty())
                return;

            ++batchId;

            Timer timer;

            std::vector<stats::DispatchingPerformanceStats> stats(requestBatch.size());

            int numEllipticBucketEntryDeletions = systemStateUpdater.numPendingEllipticBucketEntryDeletions();

            // Before searching for assignments, commit all pending deletions to the elliptic buckets as well as
            // insertions and deletions to the last stop buckets that were caused by vehicles progressing in their
            // routes since the last batch.
            systemStateUpdater.commitPendingEllipticBucketEntryDeletions();
            systemStateUpdater.commitPendingLastStopBucketEntryInsertionsAndDeletions();

            KASSERT(systemStateUpdater.noPendingEllipticBucketEntryInsertionsOrDeletions());
            KASSERT(systemStateUpdater.noPendingLastStopBucketEntryInsertionsOrDeletions());

            // Find an assignment for each request in requestBatch. May require multiple rounds for some requests
            // if there are conflicting assignments.
            int iteration = 0;
            while (!requestBatch.empty()) {
                ++iteration;
                const auto iterationNumRequests = requestBatch.size();

                Timer iterationTimer;
                auto responses = assignmentFinder.findBestAssignmentsForBatchParallel(requestBatch, now, stats);
                const auto iterationFindAssignmentsTime = iterationTimer.elapsed<std::chrono::nanoseconds>();

                // Requests for which no assignment could be found can immediately do modal choice for other modes,
                // and be finished.
                iterationTimer.restart();
                KASSERT(responses.size() == requestBatch.size() && stats.size() == requestBatch.size());
                for (int i = requestBatch.size() - 1; i >= 0; --i) {
                    KASSERT(responses[i].originalRequest.requestId == requestBatch[i].requestId);

                    if (!responses[i].validTaxiTripKnown()) {
                        using mode_choice::TransportMode;
                        const auto walkingResult = walkingTripFinder.findWalkingTrip(
                                    responses[i], stats[i].initializationStats);
                        const auto carResult = carTripFinder.findCarTrip(responses[i]);
                        const auto &ptData = ptJourneyData[responses[i].originalRequest.requestId];

                        const TransportMode mode = riderModeChoice.chooseMode(responses[i], walkingResult, carResult, TaxiResult(), ptData);
                        KASSERT(mode != TransportMode::Taxi);

                        systemStateUpdater.writeBestAssignmentToLogger(mode, responses[i], stats[i].costStats);

                        if (mode == TransportMode::Ped) {
                            processChoiceOnlyWalking(responses[i], stats[i], requestBatch[i].requestId, now);
                        } else if (mode == TransportMode::Car) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now,
                                                   now + responses[i].originalReqDirectDist);
                        } else if (mode == TransportMode::PublicTransport) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now, now +
                                                       ptJourneyData[requestBatch[i].requestId].
                                                       totalJourneyTimeTenthsOfSeconds());
                        } else if (mode == TransportMode::None) {
                            processNoMode(responses[i], stats[i], requestBatch[i].requestId);
                        } else {
                            throw std::runtime_error("Unsupported transport mode chosen in event simulation.");
                        }

                        // Remove request from batch, responses and stats
                        requestBatch[i] = requestBatch.back();
                        requestBatch.pop_back();
                        responses[i] = responses.back();
                        responses.pop_back();
                        stats[i] = stats.back();
                        stats.pop_back();

                        ++progressBar;
                    }
                }

                // To avoid conflicts, we only accept one assignment per vehicle. If there are multiple assignments to
                // the same vehicle, only one of them can be accepted, while the others are postponed for a second run
                // of assignments.
                using mode_choice::TransportMode;
                std::vector<TransportMode> modes(requestBatch.size(), TransportMode::None);
                vehiclesWithChangesInRoute.clear();
                for (int i = 0; i < requestBatch.size(); ++i) {
                    KASSERT(responses[i].validTaxiTripKnown());
                    std::vector<int> vehiclesAffectedByAsgn;
                    if (responses[i].improvementThroughTransfer()) {
                        const AssignmentWithTransfer &asgnWithTransfer = responses[i].getBestAssignmentWithTransfer();
                       vehiclesAffectedByAsgn = systemStateUpdater.getVehiclesAffectedByAssignmentWithTransfer(asgnWithTransfer, responses[i]);
                    } else {
                        const Assignment &asgn = responses[i].getBestAssignmentWithoutTransfer();
                        vehiclesAffectedByAsgn = systemStateUpdater.getVehiclesAffectedByAssignmentWithoutTransfer(asgn, responses[i]);
                    }
                    // If another request already accepted an assignment that affects the route of any vehicle that
                    // would be affected by this assignment, we cannot safely perform the assignment, so we need
                    // to be reassign this request.
                    bool hasConflict = false;
                    for (const auto &vehId : vehiclesAffectedByAsgn) {
                        if (vehiclesWithChangesInRoute.contains(vehId)) {
                            hasConflict = true;
                            break;
                        }
                    }
                    if (hasConflict)
                        continue;

                    const TaxiResult taxiResult = taxiResultConstructor.constructTaxiResult(responses[i]);

                    const auto walkingResult = walkingTripFinder.findWalkingTrip(
                                    responses[i], stats[i].initializationStats);
                    const auto carResult = carTripFinder.findCarTrip(responses[i]);
                    const auto &ptData = ptJourneyData[responses[i].originalRequest.requestId];

                    // Check if this rider wants to accept the assignment. If so, no other riders can accept an
                    // assignment to this vehicle. In either case, the request can be considered finished since it
                    // had its chance to accept the taxi assignment.
                    modes[i] = riderModeChoice.chooseMode(responses[i], walkingResult, carResult, taxiResult, ptData);
                    if (modes[i] == TransportMode::Taxi) {
                        // Register all affected vehicles as changed to block other requests that affect these vehicles
                        for (const auto &vehId : vehiclesAffectedByAsgn) {
                            vehiclesWithChangesInRoute.insert(vehId);
                        }
                    } else {
                        systemStateUpdater.writeBestAssignmentToLogger(modes[i], responses[i], stats[i].costStats);
                        if (modes[i] == TransportMode::Ped) {
                            processChoiceOnlyWalking(responses[i], stats[i], requestBatch[i].requestId, now);
                        } else if (modes[i] == TransportMode::Car) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now,
                                                   now + responses[i].originalReqDirectDist);
                        } else if (modes[i] == TransportMode::PublicTransport) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now, now +
                                                       ptJourneyData[requestBatch[i].requestId].
                                                       totalJourneyTimeTenthsOfSeconds());
                        } else if (modes[i] == TransportMode::None) {
                            processNoMode(responses[i], stats[i], requestBatch[i].requestId);
                        } else {
                            throw std::runtime_error("Unsupported transport mode chosen in event simulation.");
                        }
                    }

                    // // Marker that an assignment to this vehicle has been accepted so no other requests can accept.
                    // // Accepted assignment will be processed in batch later.
                    // lastAcceptedVehId = vehId;
                }

                // Get permutation with non-finished requests first, then finished taxi requests,
                // then finished non-taxi requests.
                std::vector<int> order(requestBatch.size());
                std::iota(order.begin(), order.end(), 0);
                std::sort(order.begin(), order.end(), [&](const auto &i1, const auto &i2) {
                    const bool i1Finished = modes[i1] != TransportMode::None;
                    const bool i2Finished = modes[i2] != TransportMode::None;
                    if (i1Finished != i2Finished)
                        return !i1Finished; // Finished requests go to the back.
                    if (!i1Finished)
                        return false; // Both not finished, keep original order.
                    // Both finished, taxi requests go before non-taxi requests.
                    if (modes[i1] != modes[i2])
                        return modes[i1] == TransportMode::Taxi;
                    return false; // Both same mode, keep original order.
                });
                Permutation orderPerm(order.begin(), order.end());
                orderPerm.invert();
                orderPerm.applyTo(requestBatch);
                orderPerm.applyTo(responses);
                orderPerm.applyTo(stats);
                orderPerm.applyTo(modes);

                auto firstFinishedTaxiIt = std::find_if(modes.begin(), modes.end(), [](const TransportMode &m) {
                    return m != TransportMode::None;
                });
                auto firstFinishedOtherIt = std::find_if(firstFinishedTaxiIt, modes.end(), [](const TransportMode &m) {
                    return m != TransportMode::Taxi && m != TransportMode::None;
                });
                const int firstFinishedTaxi = static_cast<int>(std::distance(modes.begin(), firstFinishedTaxiIt));
                const int firstFinishedOther = static_cast<int>(std::distance(modes.begin(), firstFinishedOtherIt));
                const int numFinished = static_cast<int>(modes.size() - firstFinishedTaxi);

                // Remove finished non-taxi assignments from batch.
                requestBatch.erase(requestBatch.begin() + firstFinishedOther, requestBatch.end());
                responses.erase(responses.begin() + firstFinishedOther, responses.end());
                stats.erase(stats.begin() + firstFinishedOther, stats.end());

                const auto iterationChooseAcceptedTime = iterationTimer.elapsed<std::chrono::nanoseconds>();

                iterationTimer.restart();
                const auto acceptedResponses = IteratorRange(responses.begin() + firstFinishedTaxi, responses.end());
                auto acceptedStats = IteratorRange(stats.begin() + firstFinishedTaxi, stats.end());
                for (int i = 0; i < acceptedResponses.size(); ++i) {
                    const auto & resp = acceptedResponses[i];
                    const auto & stat = acceptedStats[i];
                    systemStateUpdater.writeBestAssignmentToLogger(TransportMode::Taxi, resp, stat.costStats);
                }

                std::vector<int> expectedAffectedVehicles(vehiclesWithChangesInRoute.begin(), vehiclesWithChangesInRoute.end());
                vehiclesWithChangesInRoute.clear();
                systemStateUpdater.insertBatchOfBestAssignments(acceptedResponses, acceptedStats, now, iteration, vehiclesWithChangesInRoute);
                unused(expectedAffectedVehicles);
                KASSERT(allContained(expectedAffectedVehicles, vehiclesWithChangesInRoute), "Actually affected vehicles and expected affected vehicles do not match.");
                for (const auto &acceptedResp: acceptedResponses) {
                    updateRequestData(acceptedResp);
                }
                updateSimulationForAffectedVehicles(vehiclesWithChangesInRoute);
                progressBar += numFinished;
                const auto iterationUpdateSystemStateTime = iterationTimer.elapsed<std::chrono::nanoseconds>();


                // Remove all finished requests from batch and retry rejected ones.
                requestBatch.erase(requestBatch.begin() + firstFinishedTaxi, requestBatch.end());
                stats.erase(stats.begin() + firstFinishedTaxi, stats.end());

                batchDispatchStatsLogger << now << "," << "0" << "," << batchId << "," << iteration << ","
                        << iterationNumRequests << "," << numFinished << "," << iterationFindAssignmentsTime << ","
                        << iterationChooseAcceptedTime << ","
                        << numEllipticBucketEntryDeletions << ","
                        << iterationUpdateSystemStateTime << '\n';
                numEllipticBucketEntryDeletions = 0; // Deletions are associated with first iteration.
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << now << ",RequestBatchDispatch," << time << '\n';
        }

        static bool allContained(const std::vector<int> &s1, const Subset &s2) {
            if (s1.size() != s2.size())
                return false;
            for (const auto &element : s1) {
                if (!s2.contains(element))
                    return false;
            }
            return true;
        }

        template<typename AssignmentFinderResponseT>
        void processChoiceOnlyWalking(const AssignmentFinderResponseT &asgnFinderResponse,
                                      stats::DispatchingPerformanceStats stats, const int reqId,
                                      const int occTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to walk to their destination and insert event for their arrival.
            riderState[reqId] = WALKING_FROM_DROPOFF_TO_DESTINATION;
            requestData[reqId].assignmentCost = CostCalculator::calcCostForNotUsingVehicle(
                asgnFinderResponse.odWalkingDist, asgnFinderResponse);
            requestData[reqId].directOdDist = asgnFinderResponse.originalReqDirectDist;
            requestData[reqId].depTimeAtPickup = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.odWalkingDist;
            requestEvents.insert(reqId, {occTime + asgnFinderResponse.odWalkingDist, reqId});
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void processChoiceOtherMode(const AssignmentFinderResponseT &asgnFinderResponse,
                                    stats::DispatchingPerformanceStats stats,
                                    const int reqId, const int occTime, const int arrivalTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to take other mode to their destination and insert event for their arrival.
            riderState[reqId] = OTHER_MODE_TO_DESTINATION;
            requestData[reqId].assignmentCost = INFTY;
            requestData[reqId].directOdDist = asgnFinderResponse.originalReqDirectDist;
            // No meaningful cost can be assigned since this is not a possibility in local cost function
            requestData[reqId].depTimeAtPickup = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = 0;
            requestEvents.insert(reqId, {arrivalTime, reqId});
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void processNoMode(const AssignmentFinderResponseT &asgnFinderResponse,
                                    stats::DispatchingPerformanceStats stats,
                                    const int reqId) {
            KASSERT(!requestEvents.contains(reqId));
            riderState[reqId] = FINISHED;
            requestData[reqId].assignmentCost = INFTY;
            requestData[reqId].directOdDist = asgnFinderResponse.originalReqDirectDist;
            requestData[reqId].depTimeAtPickup = -1;
            requestData[reqId].walkingTimeToPickup = -1;
            requestData[reqId].walkingTimeFromDropoff = -1;
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void
        updateRequestData(const AssignmentFinderResponseT &asgnFinderResponse) {
            const int reqId = asgnFinderResponse.originalRequest.requestId;
            KASSERT(riderState[reqId] == WAITING_FOR_DISPATCH);
            KASSERT(!requestEvents.contains(reqId));

            requestData[reqId].directOdDist = asgnFinderResponse.originalReqDirectDist;
            if (asgnFinderResponse.improvementThroughTransfer()) {
                const AssignmentWithTransfer &asgnWithTransfer = asgnFinderResponse.getBestAssignmentWithTransfer();
                KASSERT(asgnWithTransfer.pVeh && asgnWithTransfer.dVeh && asgnWithTransfer.pickup.id != INVALID_ID &&
                        asgnWithTransfer.dropoff.id != INVALID_ID);
                riderState[reqId] = ASSIGNED_TO_PVEH;
                requestData[reqId].walkingTimeToPickup = asgnWithTransfer.pickup.walkingDist;
                requestData[reqId].walkingTimeFromDropoff = asgnWithTransfer.dropoff.walkingDist;
                requestData[reqId].assignmentCost = asgnFinderResponse.getBestCostWithTransfer();
            } else {
                const Assignment &asgnWithoutTransfer = asgnFinderResponse.getBestAssignmentWithoutTransfer();
                KASSERT(asgnWithoutTransfer.vehicle && asgnWithoutTransfer.pickup.id != INVALID_ID && asgnWithoutTransfer.dropoff.id != INVALID_ID);

                riderState[reqId] = ASSIGNED_TO_VEH;
                requestData[reqId].walkingTimeToPickup = asgnWithoutTransfer.pickup.walkingDist;
                requestData[reqId].walkingTimeFromDropoff = asgnWithoutTransfer.dropoff.walkingDist;
                requestData[reqId].assignmentCost = asgnFinderResponse.getBestCostWithoutTransfer();
            }
        }

        void updateSimulationForAffectedVehicles(const Subset &vehiclesWithChanges) {
            for (const auto &vehId: vehiclesWithChanges) {
                switch (vehicleState[vehId]) {
                    case STOPPING:
                        // Update event time to departure time at current stop since it may have changed
                        vehicleEvents.updateKey(vehId, {
                                                    scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime, vehId
                                                });
                        break;
                    case IDLING:
                        vehicleState[vehId] = VehicleState::DRIVING;
                        [[fallthrough]];
                    case DRIVING:
                        // Update event time to arrival time at next stop since it may have changed (also for case of idling).
                        vehicleEvents.updateKey(vehId, {scheduledStops.getNextScheduledStop(vehId).arrTime, vehId});
                        [[fallthrough]];
                    default:
                        break;
                }
            }
        }

        void handleWalkingArrivalAtDest(const int reqId, const int occTime) {
            KASSERT(riderState[reqId] == WALKING_FROM_DROPOFF_TO_DESTINATION);
            Timer timer;

            const RequestData &reqData = requestData[reqId];
            riderState[reqId] = FINISHED;
            int id;
            EntityKey key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key.time == occTime);

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

            assignmentQualityStats << reqId << ','
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

        // For other modes than walking or taxi
        void handleOtherModeArrivalAtDest(const int reqId, const int occTime) {
            KASSERT(riderState[reqId] == OTHER_MODE_TO_DESTINATION);
            Timer timer;

            const auto &reqData = requestData[reqId];
            riderState[reqId] = FINISHED;
            int id;
            EntityKey key;
            requestEvents.deleteMin(id, key);
            assert(id == reqId && key.time == occTime);

            constexpr auto waitTime = 0;
            const auto arrTime = occTime;
            const auto rideTime = occTime - reqData.walkingTimeFromDropoff - reqData.depTimeAtPickup;
            KASSERT(reqData.walkingTimeToPickup > 0 || reqData.walkingTimeFromDropoff > 0 || rideTime >= reqData.directOdDist);
            const auto tripTime = arrTime - requests[reqId].requestTime;
            assignmentQualityStats << reqId << ','
            << false << ","
                    << arrTime << ','
                    << waitTime << ','
                    << rideTime << ','
                    << tripTime << ','
                    << reqData.walkingTimeToPickup << ','
                    << reqData.walkingTimeFromDropoff << ','
                    << reqData.assignmentCost << '\n';


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestOtherModeArrival," << time << '\n';
        }


        const Fleet &fleet;
        const std::vector<Request> &requests;
        const std::vector<mode_choice::PTJourneyData> &ptJourneyData;
        AssignmentFinderT &assignmentFinder;
        WalkingTripFinderT &walkingTripFinder;
        CarTripFinderT &carTripFinder;
        TaxiResultConstructorT &taxiResultConstructor;
        RiderModeChoiceT &riderModeChoice;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;

        AddressableKHeapAnyKeyType<4, EntityKey> vehicleEvents;
        AddressableKHeapAnyKeyType<4, EntityKey> requestEvents;

        int nextRequestBatchDeadline;
        int batchId;
        std::vector<Request> requestBatch;

        Subset vehiclesWithChangesInRoute;

        std::vector<VehicleState> vehicleState;
        std::vector<RiderState> riderState;

        std::vector<RequestData> requestData;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &batchDispatchStatsLogger;
        std::ofstream &assignmentQualityStats;

        std::ofstream &legStatsLogger;
        std::ofstream &requestDispatchedSingleLogger;
        ProgressBar progressBar;
    };
}
