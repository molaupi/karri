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

#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include <utility>

#include "DataStructures/Queues/AddressableKHeap.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Tools/Workarounds.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/Timer.h"
#include "Tools/EnumParser.h"


namespace karri {

    template<typename AssignmentFinderT,
            typename SystemStateUpdaterT,
            typename ScheduledStopsT>
    class MobitoppEventSimulation {

        enum VehicleState {
            OUT_OF_SERVICE,
            IDLING,
            DRIVING,
            STOPPING
        };

        enum RequestState {
            NOT_RECEIVED,
            RECEIVED,
            ASSIGNED_TO_VEH,
            WALKING_TO_DEST,
            FINISHED
        };

        // Stores information about assignment and departure time of a request needed for logging on arrival of the
        // request.
        struct RequestData {
            int requestId = INFTY; // Set when request is received
            int minDepTime = INFTY; // Set when request is received.

            int vehicleId = INVALID_ID; // Set when request is assigned.
            int walkingTimeToPickup = INFTY; // Set when request is assigned.
            int walkingTimeFromDropoff = INFTY; // Set when request is assigned.
            int assignmentCost = INFTY; // Set when request is assigned.

            int depTime = INFTY; // Set when rider enters vehicle and departs.

            int arrTime = INFTY; // Set at arrival at destination.
        };


    public:

        MobitoppEventSimulation(
                const Fleet &fleet, const RouteState &routeState,
                AssignmentFinderT &assignmentFinder, SystemStateUpdaterT &systemStateUpdater,
                const ScheduledStopsT &scheduledStops, const int expectedMinNumRequests = 1000
//                , const bool verbose = false
        )
                : fleet(fleet),
                  routeState(routeState),
                  assignmentFinder(assignmentFinder),
                  systemStateUpdater(systemStateUpdater),
                  scheduledStops(scheduledStops),
                  vehicleEvents(fleet.size()),
                  walkingArrivalEvents(expectedMinNumRequests),
                  vehicleState(fleet.size(), OUT_OF_SERVICE),
                  requestState(),
                  requestData(),
                  eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
                                                                                  "occurrence_time,"
                                                                                  "type,"
                                                                                  "running_time\n")),
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
                                                                      "occupancy\n"))
//                  progressBar(requests.size(), verbose)
        {}

        void init() {
            vehicleEvents.clear();
            for (const auto &veh: fleet) {
                vehicleEvents.insert(veh.vehicleId, veh.startOfServiceTime);
            }
        }


        std::vector<RequestData const *> handleFleetControlAndUpdate(const int now) {
            advanceEventsUntil(now);

            std::vector<RequestData const *> arrivals;
            arrivals.swap(recentlyArrivedRequests);
            return arrivals;
        }

        const Offer &handleRequest(const Request &request) {

//            advanceEventsUntil(request.issuingTime);

            Timer timer;

            if (request.requestId >= requestState.size()) {
                requestState.resize(request.requestId + 1, NOT_RECEIVED);
                requestData.resize(request.requestId + 1, {});
            }

            requestState[request.requestId] = RECEIVED;
            requestData[request.requestId].requestId = request.requestId;
            requestData[request.requestId].minDepTime = request.minDepTime;

            const auto &asgnFinderResponse = assignmentFinder.findBestAssignment(request);
            systemStateUpdater.writeBestAssignmentToLogger();

            const auto offerId = offers.size() + 1; // offer IDs start at 1
            offers.push_back(createOffer(offerId, request.issuingTime, request, asgnFinderResponse));


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << request.issuingTime << ",RequestReceipt," << time << '\n';
            return offers.back();
        }

        ConstantVectorRange<Offer> handleFirstMileRequest(const FirstMileRequest &request) {
//            advanceEventsUntil(request.issuingTime);

            Timer timer;

            if (request.requestId >= requestState.size()) {
                requestState.resize(request.requestId + 1, NOT_RECEIVED);
                requestData.resize(request.requestId + 1, {});
            }

            requestState[request.requestId] = RECEIVED;

            const auto prevOffersSize = offers.size();
            for (const auto &transfer: request.transfers) {
                // todo: We are not considering the additional trip time from the transfer to the destination while
                //  finding an assignment.
                Request simpleReq = {request.requestId, request.origin, transfer.loc, request.numRiders,
                                     request.issuingTime, request.minDepTime};

                const auto &asgnFinderResponse = assignmentFinder.findBestAssignment(simpleReq);
                // todo: when to write best found assignment? Do we only want to write a single one?
                systemStateUpdater.writeBestAssignmentToLogger();

                const auto offerId = offers.size() + 1; // offer IDs start at 1
                offers.push_back(createOffer(offerId, request.issuingTime, simpleReq, asgnFinderResponse));
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << request.issuingTime << ",FirstMileRequestReceipt," << time << '\n';
            return {offers.begin() + prevOffersSize, offers.end()};
        }

        ConstantVectorRange<Offer> handleLastMileRequest(const LastMileRequest &request) {
//            advanceEventsUntil(request.issuingTime);

            Timer timer;

            if (request.requestId >= requestState.size()) {
                requestState.resize(request.requestId + 1, NOT_RECEIVED);
                requestData.resize(request.requestId + 1, {});
            }

            requestState[request.requestId] = RECEIVED;

            const auto prevOffersSize = offers.size();
            for (const auto &transfer: request.transfers) {
                // todo: We are not considering the additional trip time from the origin to the transfer while
                //  finding an assignment.
                Request simpleReq = {request.requestId, transfer.loc, request.destination, request.numRiders,
                                     request.issuingTime, transfer.minDepTimeAtTransfer};

                const auto &asgnFinderResponse = assignmentFinder.findBestAssignment(simpleReq);
                // todo: when to write best found assignment? Do we only want to write a single one?
                systemStateUpdater.writeBestAssignmentToLogger();

                const auto offerId = offers.size() + 1; // offer IDs start at 1
                offers.push_back(createOffer(offerId, request.issuingTime, simpleReq, asgnFinderResponse));
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << request.issuingTime << ",FirstMileRequestReceipt," << time << '\n';
            return {offers.begin() + prevOffersSize, offers.end()};
        }

        void insertBookedOffer(const int offerId) {
            applyBookedOffer(offers[offerId - 1]);
        }

        void shutDown() {
            vehicleEvents.clear();
            recentlyArrivedRequests.clear();
            requestData.clear();
            requestState.clear();
            offers.clear();
        }

    private:

        template<typename AssignmentFinderResponseT>
        Offer
        createOffer(const int offerId, const int offerTime, const Request &request,
                    const AssignmentFinderResponseT &asgnFinderResponse) {
            using namespace time_utils;
            const auto &asgn = asgnFinderResponse.getBestAssignment();

            const int actualDepTime = getActualDepTimeAtPickup(asgn, asgnFinderResponse, routeState);
            const int waitTime = actualDepTime - request.minDepTime;
            const auto initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTime, asgnFinderResponse,
                                                                     routeState);
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            const int rideTime = getArrTimeAtDropoff(actualDepTime, asgn, initialPickupDetour, dropoffAtExistingStop,
                                                     routeState);

            // Fare model: 2 euro plus 30ct per minute of direct distance.
            const int fare = static_cast<int>(std::floor(
                    2.0 + 0.3 * ((double) asgnFinderResponse.originalReqDirectDist / 600.0)));


            return {offerId,
                    offerTime,
                    request.requestId,
                    request.origin,
                    request.destination,
                    request.numRiders,
                    request.issuingTime,
                    request.minDepTime,
                    asgnFinderResponse.originalReqDirectDist,
                    asgn.vehicle->vehicleId,
                    asgn.pickup->loc,
                    asgn.pickup->walkingDist,
                    asgn.dropoff->loc,
                    asgn.dropoff->walkingDist,
                    asgn.pickupStopIdx,
                    asgn.dropoffStopIdx,
                    asgn.distToPickup,
                    asgn.distFromPickup,
                    asgn.distToDropoff,
                    asgn.distFromDropoff,
                    waitTime,
                    rideTime,
                    fare,
                    asgnFinderResponse.getBestCost()};
        }

        // Processes all vehicle events and rider walking arrival that happen until the given time.
        void advanceEventsUntil(const int time) {
            int id, occTime;
            while (!((vehicleEvents.empty() || vehicleEvents.minKey() > time) &&
                     (walkingArrivalEvents.empty() || walkingArrivalEvents.minKey() > time))) {
                // Pop next event from either queue. Walking arrival event has precedence if at the same time as
                // vehicle event.

                if (walkingArrivalEvents.empty()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.empty()) {
                    walkingArrivalEvents.min(id, occTime);
                    handleWalkingArrivalAtDest(id, occTime);
                    continue;
                }

                if (vehicleEvents.minKey() < walkingArrivalEvents.minKey()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                walkingArrivalEvents.min(id, occTime);
                handleWalkingArrivalAtDest(id, occTime);
            }
        }

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
                walkingArrivalEvents.growToAllowID(reqId);
                walkingArrivalEvents.insert(reqId, occTime + reqData.walkingTimeFromDropoff);
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


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void applyBookedOffer(const Offer &offer) {
            const auto &reqId = offer.requestId;
            requestState[reqId] = ASSIGNED_TO_VEH;
            requestData[reqId].minDepTime = offer.minDepTimeAtOrigin;
            requestData[reqId].walkingTimeToPickup = offer.walkingTimeToPickup;
            requestData[reqId].walkingTimeFromDropoff = offer.walkingTimeFromDropoff;
            requestData[reqId].assignmentCost = offer.cost;

            int pickupStopId, dropoffStopId;
            systemStateUpdater.insertOffer(offer, pickupStopId, dropoffStopId);
            systemStateUpdater.writePerformanceLogs();
            assert(pickupStopId >= 0 && dropoffStopId >= 0);

            const auto vehId = offer.vehicleId;
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

            auto &reqData = requestData[reqId];
            reqData.arrTime = occTime;
            requestState[reqId] = FINISHED;
            int id, key;
            walkingArrivalEvents.deleteMin(id, key);
            assert(id == reqId && key == occTime);

            recentlyArrivedRequests.push_back(&reqData);

            const auto waitTime = reqData.depTime - reqData.minDepTime;
            const auto arrTime = reqData.arrTime;
            const auto rideTime = arrTime - reqData.walkingTimeFromDropoff - reqData.depTime;
            const auto tripTime = arrTime - reqData.minDepTime;
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


        const Fleet &fleet;
        const RouteState &routeState;

        AssignmentFinderT &assignmentFinder;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;

        AddressableQuadHeap vehicleEvents;
        AddressableQuadHeap walkingArrivalEvents;

        std::vector<VehicleState> vehicleState;
        std::vector<RequestState> requestState;

        std::vector<RequestData> requestData;

        std::vector<Offer> offers;

        // Data on requests that have arrived since the last call to handleFleetControlAndUpdate().
        std::vector<RequestData const *> recentlyArrivedRequests;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &assignmentQualityStats;
        std::ofstream &legStatsLogger;
//        ProgressBar progressBar;

    };
}

