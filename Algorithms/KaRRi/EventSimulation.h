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

namespace karri {


    template<typename AssignmentFinderT,
            typename RiderModeChoiceT,
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
            WAITING_FOR_DISPATCH,
            ASSIGNED_TO_VEH,
            WALKING_TO_DEST,
            IN_PRIVATE_CAR,
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
                AssignmentFinderT &assignmentFinder,
                const RiderModeChoiceT &riderModeChoice,
                SystemStateUpdaterT &systemStateUpdater,
                const ScheduledStopsT &scheduledStops,
                const bool verbose = false)
                : fleet(fleet),
                  requests(requests),
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

                // If the deadline for the next request batch has been reached, dispatch the batch of requests
                // collected before continuing with the next request or vehicle events.
                if (occTime > nextRequestBatchDeadline) {
                    dispatchRequestBatch();
                    continue;
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
                case IN_PRIVATE_CAR:
                    handlePrivatCarArrivalAtDest(reqId, occTime);
                    break;
                case FINISHED:
                    assert(false);
                    break;
                default:
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
            requestBatch.push_back(requests[reqId]);
            requestState[reqId] = WAITING_FOR_DISPATCH;

            // event for walking arrival at dest inserted at dispatching time for walking-only or when vehicle reaches dropoff
            int id, key;
            requestEvents.deleteMin(id, key);
            assert(id == reqId && key == occTime);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        void dispatchRequestBatch() {

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


                // Requests for which no assignment could be found can be discarded. Requests for which the best
                // assignment does not use a vehicle are conflict-free and can be applied right away.
                iterationTimer.restart();
                KASSERT(responses.size() == requestBatch.size() && stats.size() == requestBatch.size());
                for (int i = requestBatch.size() - 1; i >= 0; --i) {
                    KASSERT(responses[i].originalRequest.requestId == requestBatch[i].requestId);

                    using mode_choice::TransportMode;
                    const TransportMode mode = riderModeChoice.chooseMode(requestBatch[i], responses[i]);

                    // If response uses a taxi vehicle, process later
                    if (mode == TransportMode::Taxi && responses[i].getBestAssignment().vehicle) {
                        continue;
                    }

                    systemStateUpdater.writeBestAssignmentToLogger(responses[i], mode);

                    if (mode == TransportMode::Ped) {
                        processChoiceOnlyWalking(responses[i], stats[i], requestBatch[i].requestId, now);
                    } else {
                        processChoicePrivateCar(responses[i], stats[i], requestBatch[i].requestId, now);
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

                // To avoid conflicts, we only accept one assignment per vehicle. If there are multiple assignments to
                // the same vehicle, only one of them is accepted, while the others are postponed for a second run of
                // assignments.
                std::vector<int> orderOfVehicle(requestBatch.size());
                std::iota(orderOfVehicle.begin(), orderOfVehicle.end(), 0);
                std::sort(orderOfVehicle.begin(), orderOfVehicle.end(), [&](const auto &i1, const auto &i2) {
                    LIGHT_KASSERT(
                            responses[i1].getBestAssignment().vehicle && responses[i2].getBestAssignment().vehicle);
                    const auto &vehId1 = responses[i1].getBestAssignment().vehicle->vehicleId;
                    const auto &vehId2 = responses[i2].getBestAssignment().vehicle->vehicleId;
                    return vehId1 < vehId2;
                });
                Permutation orderOfVehiclePerm(orderOfVehicle.begin(), orderOfVehicle.end());
                orderOfVehiclePerm.invert();
                orderOfVehiclePerm.applyTo(requestBatch);
                orderOfVehiclePerm.applyTo(responses);
                orderOfVehiclePerm.applyTo(stats);
                std::vector<int> accepted;
                int lastAcceptedVehId = INVALID_ID;
                for (int i = 0; i < requestBatch.size(); ++i) {
                    LIGHT_KASSERT(responses[i].getBestAssignment().vehicle);
                    const auto &vehId = responses[i].getBestAssignment().vehicle->vehicleId;
                    if (vehId == lastAcceptedVehId)
                        continue;
                    accepted.push_back(i);
                    lastAcceptedVehId = vehId;
                }

                // Move all accepted to the back.
                int firstAcc = requestBatch.size();
                for (int j = accepted.size() - 1; j >= 0; --j) {
                    const auto &idxOfAcc = accepted[j];
                    --firstAcc;
                    std::swap(requestBatch[idxOfAcc], requestBatch[firstAcc]);
                    std::swap(responses[idxOfAcc], responses[firstAcc]);
                    std::swap(stats[idxOfAcc], stats[firstAcc]);
                }

                const auto iterationChooseAcceptedTime = iterationTimer.elapsed<std::chrono::nanoseconds>();

                iterationTimer.restart();
                const auto acceptedResponses = IteratorRange(responses.begin() + firstAcc, responses.end());
                auto acceptedStats = IteratorRange(stats.begin() + firstAcc, stats.end());
                systemStateUpdater.insertBatchOfBestAssignments(acceptedResponses, acceptedStats, now, iteration);
                updateSimulationForAssignmentBatch(acceptedResponses);
                progressBar += accepted.size();
                const auto iterationUpdateSystemStateTime = iterationTimer.elapsed<std::chrono::nanoseconds>();


                // Remove all accepted requests from batch and retry rejected ones.
                requestBatch.erase(requestBatch.begin() + firstAcc, requestBatch.end());
                stats.erase(stats.begin() + firstAcc, stats.end());

                batchDispatchStatsLogger << now << "," << iteration << "," << iterationNumRequests << ","
                                         << accepted.size() << "," << iterationFindAssignmentsTime << ","
                                         << iterationChooseAcceptedTime << ","
                                         << numEllipticBucketEntryDeletions << ","
                                         << iterationUpdateSystemStateTime << '\n';
                numEllipticBucketEntryDeletions = 0; // Deletions are associated with first iteration.
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << now << ",RequestBatchDispatch," << time << '\n';

        }

        template<typename AssignmentFinderResponseT>
        void processChoicePrivateCar(const AssignmentFinderResponseT &asgnFinderResponse,
                                     stats::DispatchingPerformanceStats stats, const int reqId, const int occTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to take private car to their destination and insert event for their arrival.
            requestState[reqId] = IN_PRIVATE_CAR;
            requestData[reqId].assignmentCost = INFTY; // No meaningful cost can be assigned since this is not a possibility in local cost function
            requestData[reqId].depTime = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = 0;
            requestEvents.insert(reqId, occTime + asgnFinderResponse.originalReqDirectDist);
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void processChoiceOnlyWalking(const AssignmentFinderResponseT &asgnFinderResponse,
                                      stats::DispatchingPerformanceStats stats, const int reqId,
                                      const int occTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to walk to their destination and insert event for their arrival.
            requestState[reqId] = WALKING_TO_DEST;
            requestData[reqId].assignmentCost = CostCalculator::calcCostForNotUsingVehicle(asgnFinderResponse.odWalkingDist, asgnFinderResponse);
            requestData[reqId].depTime = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.odWalkingDist;
            requestEvents.insert(reqId, occTime + asgnFinderResponse.odWalkingDist);
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AssignmentFinderResponseT>
        void
        applyAssignment(const AssignmentFinderResponseT &asgnFinderResponse,
                        stats::DispatchingPerformanceStats &stats) {
            const int reqId = asgnFinderResponse.originalRequest.requestId;
            KASSERT(!requestEvents.contains(reqId));
            KASSERT(!asgnFinderResponse.isNotUsingVehicleBest());

            const auto &bestAsgn = asgnFinderResponse.getBestAssignment();
            KASSERT(bestAsgn.vehicle && bestAsgn.pickup.id != INVALID_ID && bestAsgn.dropoff.id != INVALID_ID);

            requestState[reqId] = ASSIGNED_TO_VEH;
            requestData[reqId].walkingTimeToPickup = bestAsgn.pickup.walkingDist;
            requestData[reqId].walkingTimeFromDropoff = bestAsgn.dropoff.walkingDist;
            requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();

            systemStateUpdater.insertBestAssignment(asgnFinderResponse, stats.updateStats);
            systemStateUpdater.writePerformanceLogs(asgnFinderResponse, stats);

            const auto vehId = bestAsgn.vehicle->vehicleId;
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

        template<typename AssignmentFinderResponsesT>
        void
        updateSimulationForAssignmentBatch(const AssignmentFinderResponsesT &asgnFinderResponses) {

            // Write system state logs, update simulation for request and vehicles.
            for (auto respIt = asgnFinderResponses.begin(); respIt != asgnFinderResponses.end(); ++respIt) {
                const auto asgnFinderResponse = *respIt;

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

        void handlePrivatCarArrivalAtDest(const int reqId, const int occTime) {
            KASSERT(requestState[reqId] == IN_PRIVATE_CAR);
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
            eventSimulationStatsLogger << occTime << ",RequestPrivateCarArrival," << time << '\n';
        }


        const Fleet &fleet;
        const std::vector<Request> &requests;
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