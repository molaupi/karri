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
#include "RiderModeChoice/TransportMode.h"
#include "Algorithms/KaRRi/RiderModeChoice/PTJourneyData.h"

namespace karri {


    template<typename AssignmentFinderT,
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

        enum RequestState {
            NOT_RECEIVED,
            WAITING_FOR_DISPATCH,
            ASSIGNED_TO_VEH,
            WALKING_TO_DEST,
            IN_OTHER_MODE, // rider is in mode other than taxi or walking with fixed arrival time
            FINISHED
        };

        // Stores information about assignment and departure time of a request needed for logging on arrival of the
        // request.
        struct RequestData {
            int depTime;
            int walkingTimeToPickup;
            int walkingTimeFromDropoff;
            int assignmentCost;
        };

    public:


        EventSimulation(
                const Fleet &fleet, const std::vector<Request> &requests,
                const std::vector<PTJourneyData> &ptJourneyData,
                AssignmentFinderT &assignmentFinder,
                const RiderModeChoiceT &riderModeChoice,
                SystemStateUpdaterT &systemStateUpdater,
                const ScheduledStopsT &scheduledStops,
                const bool verbose = false)
                : fleet(fleet),
                  requests(requests),
                  ptJourneyData(ptJourneyData),
                  assignmentFinder(assignmentFinder),
                  riderModeChoice(riderModeChoice),
                  systemStateUpdater(systemStateUpdater),
                  scheduledStops(scheduledStops),
                  vehicleEvents(fleet.size()),
                  requestEvents(requests.size()),
                  nextRequestBatchDeadline(InputConfig::getInstance().requestBatchInterval),
                  requestBatch(),
                  vehicleState(fleet.size(), OUT_OF_SERVICE),
                  requestState(requests.size(), NOT_RECEIVED),
                  requestData(requests.size(), RequestData()),
                  eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
                                                                                  "occurrence_time,"
                                                                                  "type,"
                                                                                  "running_time\n")),
                  batchDispatchStatsLogger(LogManager<std::ofstream>::getLogger("batchdispatchstats.csv",
                                                                                "occurence_time,"
                                                                                "iteration,"
                                                                                "num_requests,"
                                                                                "num_accepted,"
                                                                                "find_assignments_running_time,"
                                                                                "choose_accepted_running_time,"
                                                                                "num_elliptic_bucket_entry_deletions,"
                                                                                "update_system_state_running_time\n")),
                  assignmentQualityStats(LogManager<std::ofstream>::getLogger("assignmentquality.csv",
                                                                              "request_id,"
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
            for (const auto &veh: fleet)
                vehicleEvents.insert(veh.vehicleId, veh.startOfServiceTime);
            for (const auto &req: requests)
                requestEvents.insert(req.requestId, req.requestTime);
        }

        void run() {

            while (!(vehicleEvents.empty() && requestEvents.empty())) {
                // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
                int id, occTime;

                const bool nextEventIsRequest =
                        !requestEvents.empty() &&
                        (vehicleEvents.empty() || requestEvents.minKey() <= vehicleEvents.minKey());

                if (nextEventIsRequest)
                    requestEvents.min(id, occTime);
                else
                    vehicleEvents.min(id, occTime);

                if constexpr (BATCHED_DISPATCHING) {
                    // If the deadline for the next request batch has been reached, dispatch the batch of requests
                    // collected before continuing with the next request or vehicle events.
                    if (occTime > nextRequestBatchDeadline) {
                        dispatchRequestBatch();
                        continue;
                    }
                }

                if (nextEventIsRequest)
                    handleRequestEvent(id, occTime);
                else
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
            switch (requestState[reqId]) {
                case NOT_RECEIVED:
                    handleRequestReceipt(reqId, occTime);
                    break;
                case ASSIGNED_TO_VEH:
                    // When assigned to a vehicle, there should be no request event until the dropoff.
                    // At that point the request state becomes WALKING_TO_DEST.
                    assert(false);
                    break;
                case WALKING_TO_DEST:
                    handleWalkingArrivalAtDest(reqId, occTime);
                    break;
                case IN_OTHER_MODE:
                    handleOtherModeArrivalAtDest(reqId, occTime);
                    break;
                case FINISHED:
                    assert(false);
                    break;
                default:
                    assert(false);
                    break;
            }
        }

        void handleVehicleStartup(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == OUT_OF_SERVICE);
            assert(fleet[vehId].startOfServiceTime == occTime);
            unused(occTime);
            Timer timer;

            // Vehicle may have already been assigned stops. In this case it will start driving right away:
            if (scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = DRIVING;
                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            } else {
                vehicleState[vehId] = IDLING;
                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleStartup," << time << '\n';
        }

        void handleVehicleShutdown(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == IDLING);
            assert(fleet[vehId].endOfServiceTime == occTime);
            unused(occTime);
            assert(!scheduledStops.hasNextScheduledStop(vehId));
            Timer timer;

            vehicleState[vehId] = OUT_OF_SERVICE;

            int id, key;
            vehicleEvents.deleteMin(id, key);
            assert(id == vehId && key == occTime);
            systemStateUpdater.notifyVehicleReachedEndOfServiceTime(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleShutdown," << time << '\n';
        }

        void handleVehicleArrivalAtStop(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == DRIVING);
            assert(scheduledStops.getNextScheduledStop(vehId).arrTime == occTime);
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
                requestState[reqId] = WALKING_TO_DEST;
                requestEvents.insert(reqId, occTime + reqData.walkingTimeFromDropoff);
            }


            // Next event for this vehicle is the departure at this stop:
            vehicleEvents.increaseKey(vehId, reachedStop.depTime);
            systemStateUpdater.notifyStopStarted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleArrival," << time << '\n';
        }

        void handleVehicleDepartureFromStop(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == STOPPING);
            assert(scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime == occTime);
            Timer timer;

            if (!scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = IDLING;
                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            } else {
                // Remember departure time for all requests picked up at this stop:
                const auto curStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
                for (const auto &reqId: curStop.requestsPickedUpHere) {
                    requestData[reqId].depTime = occTime;
                }
                vehicleState[vehId] = DRIVING;
                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            }

            systemStateUpdater.notifyStopCompleted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void handleRequestReceipt(const int reqId, const int occTime) {
            KASSERT(requestState[reqId] == NOT_RECEIVED);
            KASSERT(requests[reqId].requestTime == occTime);

            Timer timer;

            // event for walking arrival at dest inserted at dispatching time for walking-only or when vehicle reaches dropoff
            int id, key;
            requestEvents.deleteMin(id, key);
            assert(id == reqId && key == occTime);

            requestState[reqId] = WAITING_FOR_DISPATCH;
            if constexpr (BATCHED_DISPATCHING) {
                // Add to current request batch for later dispatching
                requestBatch.push_back(requests[reqId]);
            } else {
                // Dispatch immediately
                dispatchSingleRequest(requests[reqId]);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        void dispatchSingleRequest(const Request &request) requires (!BATCHED_DISPATCHING) {
            using mode_choice::TransportMode;
            const int now = request.requestTime;
            stats::DispatchingPerformanceStats stats;
            Timer overallTimer;
            Timer componentTimer;

            componentTimer.restart();
            const auto asgnFinderResponse = assignmentFinder.findBestAssignment(request, now, stats);
            const auto findAssignmentTime = componentTimer.elapsed<std::chrono::nanoseconds>();

            componentTimer.restart();
            const auto mode = riderModeChoice.chooseMode(request, asgnFinderResponse, ptJourneyData[request.requestId]);
            const auto chooseAcceptedTime = componentTimer.elapsed<std::chrono::nanoseconds>();

            componentTimer.restart();
            systemStateUpdater.writeBestAssignmentToLogger(asgnFinderResponse);
            if (mode == TransportMode::Taxi) {
                systemStateUpdater.insertSingleBestAssignment(asgnFinderResponse, stats);
                updateSimulationForAssignment(asgnFinderResponse);
            } else {
                if (mode == TransportMode::Ped) {
                    processChoiceOnlyWalking(asgnFinderResponse, stats, request.requestId, now);
                } else if (mode == TransportMode::Car) {
                    processChoiceOtherMode(asgnFinderResponse, stats, request.requestId, now,
                                           now + asgnFinderResponse.originalReqDirectDist);
                } else if (mode == TransportMode::PublicTransport) {
                    processChoiceOtherMode(asgnFinderResponse, stats, request.requestId, now,
                                           now + ptJourneyData[request.requestId].totalJourneyTimeTenthsOfSeconds());
                } else {
                    throw std::runtime_error("Unsupported transport mode chosen in event simulation.");
                }
            }
            const auto updateSystemStateTime = componentTimer.elapsed<std::chrono::nanoseconds>();

            ++progressBar;

            batchDispatchStatsLogger << now << "," << 1 << "," << 1 << "," << 1 << ","
                                     << findAssignmentTime << ","
                                     << chooseAcceptedTime << ","
                                     << 0 << ","
                                     << updateSystemStateTime << '\n';

            const auto time = overallTimer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << now << ",RequestBatchDispatch," << time << '\n';
        }

        void dispatchRequestBatch() requires BATCHED_DISPATCHING {

            const int now = nextRequestBatchDeadline;
            nextRequestBatchDeadline += InputConfig::getInstance().requestBatchInterval;

            if (requestBatch.empty())
                return;

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

                    if (!responses[i].getBestAssignment().vehicle) {
                        using mode_choice::TransportMode;
                        const TransportMode mode = riderModeChoice.chooseMode(requestBatch[i], responses[i],
                                                                              ptJourneyData[requestBatch[i].requestId]);
                        KASSERT(mode != TransportMode::Taxi);

                        systemStateUpdater.writeBestAssignmentToLogger(responses[i]);

                        if (mode == TransportMode::Ped) {
                            processChoiceOnlyWalking(responses[i], stats[i], requestBatch[i].requestId, now);
                        } else if (mode == TransportMode::Car) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now,
                                                   now + responses[i].originalReqDirectDist);
                        } else if (mode == TransportMode::PublicTransport) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now, now +
                                                                                                           ptJourneyData[requestBatch[i].requestId].totalJourneyTimeTenthsOfSeconds());
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
                std::vector<int> order(requestBatch.size());
                std::iota(order.begin(), order.end(), 0);
                std::sort(order.begin(), order.end(), [&](const auto &i1, const auto &i2) {
                    LIGHT_KASSERT(
                            responses[i1].getBestAssignment().vehicle && responses[i2].getBestAssignment().vehicle);
                    const auto &vehId1 = responses[i1].getBestAssignment().vehicle->vehicleId;
                    const auto &vehId2 = responses[i2].getBestAssignment().vehicle->vehicleId;
                    return vehId1 < vehId2;
                });
                Permutation orderPerm(order.begin(), order.end());
                orderPerm.invert();
                orderPerm.applyTo(requestBatch);
                orderPerm.applyTo(responses);
                orderPerm.applyTo(stats);
                using mode_choice::TransportMode;
                std::vector<TransportMode> modes(requestBatch.size(), TransportMode::None);
                int lastAcceptedVehId = INVALID_ID;
                for (int i = 0; i < requestBatch.size(); ++i) {
                    LIGHT_KASSERT(responses[i].getBestAssignment().vehicle);
                    const auto &vehId = responses[i].getBestAssignment().vehicle->vehicleId;

                    // If another request already accepted an assignment to this vehicle, this request cannot be
                    // finished here but needs to be reassigned.
                    if (vehId == lastAcceptedVehId)
                        continue;

                    // Check if this rider wants to accept the assignment. If so, no other riders can accept an
                    // assignment to this vehicle. In either case, the request can be considered finished since it
                    // had its chance to accept the taxi assignment.

                    modes[i] = riderModeChoice.chooseMode(requestBatch[i], responses[i],
                                                          ptJourneyData[requestBatch[i].requestId]);
                    if (modes[i] != TransportMode::Taxi) {
                        systemStateUpdater.writeBestAssignmentToLogger(responses[i]);
                        if (modes[i] == TransportMode::Ped) {
                            processChoiceOnlyWalking(responses[i], stats[i], requestBatch[i].requestId, now);
                        } else if (modes[i] == TransportMode::Car) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now,
                                                   now + responses[i].originalReqDirectDist);
                        } else if (modes[i] == TransportMode::PublicTransport) {
                            processChoiceOtherMode(responses[i], stats[i], requestBatch[i].requestId, now, now +
                                                                                                           ptJourneyData[requestBatch[i].requestId].totalJourneyTimeTenthsOfSeconds());
                        } else {
                            throw std::runtime_error("Unsupported transport mode chosen in event simulation.");
                        }
                        continue;
                    }

                    // Marker that an assignment to this vehicle has been accepted so no other requests can accept.
                    // Accepted assignment will be processed in batch later.
                    lastAcceptedVehId = vehId;
                }

                // Get permutation with non-finished requests first, then finished taxi requests,
                // then finished non-taxi requests.
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
                orderPerm.assign(order.begin(), order.end());
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
                for (const auto &resp: acceptedResponses) {
                    systemStateUpdater.writeBestAssignmentToLogger(resp);
                }

                systemStateUpdater.insertBatchOfBestAssignments(acceptedResponses, acceptedStats, now, iteration);
                for (const auto &acceptedResp: acceptedResponses) {
                    updateSimulationForAssignment(acceptedResp);
                }
                progressBar += numFinished;
                const auto iterationUpdateSystemStateTime = iterationTimer.elapsed<std::chrono::nanoseconds>();


                // Remove all finished requests from batch and retry rejected ones.
                requestBatch.erase(requestBatch.begin() + firstFinishedTaxi, requestBatch.end());
                stats.erase(stats.begin() + firstFinishedTaxi, stats.end());

                batchDispatchStatsLogger << now << "," << iteration << "," << iterationNumRequests << ","
                                         << numFinished << "," << iterationFindAssignmentsTime << ","
                                         << iterationChooseAcceptedTime << ","
                                         << numEllipticBucketEntryDeletions << ","
                                         << iterationUpdateSystemStateTime << '\n';
                numEllipticBucketEntryDeletions = 0; // Deletions are associated with first iteration.
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << now << ",RequestBatchDispatch," << time << '\n';

        }

        template<typename AssignmentFinderResponseT>
        void processChoiceOnlyWalking(const AssignmentFinderResponseT &asgnFinderResponse,
                                      stats::DispatchingPerformanceStats stats, const int reqId,
                                      const int occTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to walk to their destination and insert event for their arrival.
            requestState[reqId] = WALKING_TO_DEST;
            requestData[reqId].assignmentCost = CostCalculator::calcCostForNotUsingVehicle(
                    asgnFinderResponse.odWalkingDist, asgnFinderResponse);
            requestData[reqId].depTime = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.odWalkingDist;
            requestEvents.insert(reqId, occTime + asgnFinderResponse.odWalkingDist);
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void processChoiceOtherMode(const AssignmentFinderResponseT &asgnFinderResponse,
                                    stats::DispatchingPerformanceStats stats,
                                    const int reqId, const int occTime, const int arrivalTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to take other mode to their destination and insert event for their arrival.
            requestState[reqId] = IN_OTHER_MODE;
            requestData[reqId].assignmentCost = INFTY; // No meaningful cost can be assigned since this is not a possibility in local cost function
            requestData[reqId].depTime = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = 0;
            requestEvents.insert(reqId, arrivalTime);
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void
        updateSimulationForAssignment(const AssignmentFinderResponseT &asgnFinderResponse) {
            const int reqId = asgnFinderResponse.originalRequest.requestId;
            KASSERT(requestState[reqId] == WAITING_FOR_DISPATCH);
            KASSERT(!requestEvents.contains(reqId));

            const auto &bestAsgn = asgnFinderResponse.getBestAssignment();
            KASSERT(bestAsgn.vehicle && bestAsgn.pickup.id != INVALID_ID && bestAsgn.dropoff.id != INVALID_ID);

            requestState[reqId] = ASSIGNED_TO_VEH;
            requestData[reqId].walkingTimeToPickup = bestAsgn.pickup.walkingDist;
            requestData[reqId].walkingTimeFromDropoff = bestAsgn.dropoff.walkingDist;
            requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();

            const auto vehId = asgnFinderResponse.getBestAssignment().vehicle->vehicleId;
            switch (vehicleState[vehId]) {
                case STOPPING:
                    // Update event time to departure time at current stop since it may have changed
                    vehicleEvents.updateKey(vehId, scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime);
                    break;
                case IDLING:
                    vehicleState[vehId] = VehicleState::DRIVING;
                    [[fallthrough]];
                case DRIVING:
                    // Update event time to arrival time at next stop since it may have changed (also for case of idling).
                    vehicleEvents.updateKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                    [[fallthrough]];
                default:
                    break;
            }
        }

        void handleWalkingArrivalAtDest(const int reqId, const int occTime) {
            assert(requestState[reqId] == WALKING_TO_DEST);
            Timer timer;

            const auto &reqData = requestData[reqId];
            requestState[reqId] = FINISHED;
            int id, key;
            requestEvents.deleteMin(id, key);
            assert(id == reqId && key == occTime);

            const auto waitTime = reqData.depTime - requests[reqId].requestTime;
            const auto arrTime = occTime;
            const auto rideTime = occTime - reqData.walkingTimeFromDropoff - reqData.depTime;
            const auto tripTime = arrTime - requests[reqId].requestTime;
            assignmentQualityStats << reqId << ','
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
            KASSERT(requestState[reqId] == IN_OTHER_MODE);
            Timer timer;

            const auto &reqData = requestData[reqId];
            requestState[reqId] = FINISHED;
            int id, key;
            requestEvents.deleteMin(id, key);
            assert(id == reqId && key == occTime);

            const auto waitTime = 0;
            const auto arrTime = occTime;
            const auto rideTime = 0;
            const auto tripTime = arrTime - requests[reqId].requestTime;
            assignmentQualityStats << reqId << ','
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
        const std::vector<PTJourneyData> &ptJourneyData;
        AssignmentFinderT &assignmentFinder;
        const RiderModeChoiceT &riderModeChoice;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;

        AddressableQuadHeap vehicleEvents;
        AddressableQuadHeap requestEvents;

        int nextRequestBatchDeadline;
        std::vector<Request> requestBatch;

        std::vector<VehicleState> vehicleState;
        std::vector<RequestState> requestState;

        std::vector<RequestData> requestData;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &batchDispatchStatsLogger;
        std::ofstream &assignmentQualityStats;
        std::ofstream &legStatsLogger;
        ProgressBar progressBar;

    };
}