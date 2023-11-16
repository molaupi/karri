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

enum MobitoppMsgType {
    MT_ERROR,
    INIT_COMMUNICATION,
    FLEET_CONTROL_AND_UPDATE,
    REQUEST_OFFER,
    FIRST_MILE_REQUEST_OFFER,
    LAST_MILE_REQUEST_OFFER,
    BOOK_OFFER,
    END_OF_SIMULATION,
    INTERMODAL_REQUEST
};
static const unsigned long NUM_MOBITOPP_MSG_TYPES = static_cast<unsigned long>(OsmRoadCategory::ROAD) + 1;

// Make EnumParser usable with MobitoppMsgType.
template<>
void EnumParser<MobitoppMsgType>::initNameToEnumMap() {
    nameToEnum = {
            {"mt_error",                 MobitoppMsgType::MT_ERROR},
            {"init_communication",       MobitoppMsgType::INIT_COMMUNICATION},
            {"fleet_control_and_update", MobitoppMsgType::FLEET_CONTROL_AND_UPDATE},
            {"request_offer",            MobitoppMsgType::REQUEST_OFFER},
            {"first_mile_request_offer", MobitoppMsgType::FIRST_MILE_REQUEST_OFFER},
            {"last_mile_request_offer",  MobitoppMsgType::LAST_MILE_REQUEST_OFFER},
            {"book_offer",               MobitoppMsgType::BOOK_OFFER},
            {"end_of_simulation",        MobitoppMsgType::END_OF_SIMULATION},
            {"intermodal_request",       MobitoppMsgType::INTERMODAL_REQUEST}
    };
}


static std::array<std::string, NUM_MOBITOPP_MSG_TYPES> mobitoppMsgTypeNames = {
        "mt_error",
        "init_communication",
        "fleet_control_and_update",
        "request_offer",
        "first_mile_request_offer",
        "last_mile_request_offer",
        "book_offer",
        "end_of_simulation",
        "intermodal_request"
};

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
            ASSIGNED_TO_VEH,
            WALKING_TO_DEST,
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


        class MobitoppCommunicator {

            // todo: Read config file for potential other ports
            static constexpr int PORT = 6666;
            static constexpr int BUF_NUM_BYTES = 1024;

            // todo: Input argument for verbose
            static constexpr bool VERBOSE = true;

            enum CommState {
                WAITING_FOR_INITIALIZATION, // Waiting for initialization message
                WAITING_FOR_NEW_EVENT, // Waiting for new request, fleet control, or end of simulation
                WAITING_FOR_OFFER_RESPONSE, // Sent offer for request, waiting for book_offer message
                SIMULATION_ENDED,
                COMM_ERROR
            };


            struct MobitoppMessageParseError : public std::runtime_error {

                MobitoppMessageParseError(const std::string &what, std::string receivedMessage = "<omitted>")
                        : std::runtime_error(what), receivedMessage(std::move(receivedMessage)) {}

                std::string receivedMessage;
            };

            struct MobitoppCommError : public std::runtime_error {

            };

        public:

            void run() {
                init();

                while (listen()) {}
            }

            void init() {
                int status;
                struct sockaddr_in address;

                if ((fs_fd = socket(AF_LOCAL, SOCK_STREAM, 0)) < 0) {
                    throw std::runtime_error("Socket creation error.");
                }

                address.sin_family = AF_LOCAL;
                address.sin_port = PORT;

                if ((status = connect(fs_fd, &address, sizeof(address))) < 0) {
                    throw std::runtime_error("Socket connection to mobiTopp failed.");
                }

            }


            bool listen() {

                ssize_t numBytesRead = read(fs_fd, (void *) readbuf.data(), BUF_NUM_BYTES - 1);

                try {
                    auto msgJson = getMsgJsonFromBuf(numBytesRead);
                    return handleMessage(msgJson);
                } catch (MobitoppMessageParseError &e) {
                    state = COMM_ERROR;
                    std::cerr << e.what() << std::endl;
                    if (VERBOSE)
                        std::cerr << "Message received: " << e.receivedMessage << std::endl;
                    sendError();
                    return false; // todo: Do we try to recover errors?
                } catch (MobitoppCommError &e) {
                    state = COMM_ERROR;
                    std::cerr << e.what() << std::endl;
                    sendError();
                    return false; // todo: Do we try to recover errors?
                }
            }


        private:

            nlohmann::json getMsgJsonFromBuf(const ssize_t numBytesRead) {
                std::string msg = readbuf.data();
                if (msg.size() != numBytesRead) {
                    throw MobitoppMessageParseError("Parsed message has size " + std::to_string(msg.size()) + " but " +
                                                    std::to_string(numBytesRead) + " bytes were read.", msg);
                }
                if (!endsWith(msg, '\n')) {
                    throw MobitoppMessageParseError("Message does not end in '\\n'", msg);
                }

                msg = msg.substr(0, msg.size() - 1);

                try {
                    auto msgJson = nlohmann::json::parse(msg);
                    return msgJson;
                } catch (const nlohmann::json::parse_error &e) {
                    throw MobitoppMessageParseError("Message could not be parsed to JSON.", msg);
                }
            }


            bool handleMessage(const nlohmann::json &msg) {

                if (!msg.contains("type"))
                    throw MobitoppMessageParseError("Message JSON does not contain 'type' field.", msg.dump());

                const std::string typeStr = msg["type"];
                EnumParser<MobitoppMsgType> msgTypeParser;
                const auto type = msgTypeParser(typeStr);
                verifyMessageKeys(type, msg);
                switch (type) {
                    case MT_ERROR:
                        return handleMobitoppError(msg);
                    case INIT_COMMUNICATION:
                        return handleInitCommunication(msg);
                    case FLEET_CONTROL_AND_UPDATE:
                        return handleFleetControlAndUpdate(msg);
                    case REQUEST_OFFER:
                        return handleRequestOffer(msg);
                    case FIRST_MILE_REQUEST_OFFER:
                        return handleFirstMileRequestOffer(msg);
                    case LAST_MILE_REQUEST_OFFER:
                        return handleLastMileRequestOffer(msg);
                    case BOOK_OFFER:
                        return handleBookOffer(msg);
                    case END_OF_SIMULATION:
                        return handleEndOfSimulation(msg);
                    default:
                        throw MobitoppMessageParseError("Message type " + typeStr + " cannot be handled.", msg.dump());
                }
            }

            std::unordered_map<MobitoppMsgType, std::vector<std::string>> requiredKeysForMsg = {
                    {MT_ERROR,                 {}},
                    {INIT_COMMUNICATION,       {"status_code"}},
                    {FLEET_CONTROL_AND_UPDATE, {"time"}},
                    {REQUEST_OFFER,            {"agent_id", "origin",      "destination", "time",   "nr_pax"}},
                    {FIRST_MILE_REQUEST_OFFER, {"agent_id", "origin",      "time",        "nr_pax", "destinations"}},
                    {LAST_MILE_REQUEST_OFFER,  {"agent_id", "destination", "nr_pax",      "origins"}},
                    {BOOK_OFFER,               {"agent_id", "confirms_offer"}},
                    {END_OF_SIMULATION,        {}},
                    {INTERMODAL_REQUEST,       {"transfer", "time"}}
            };

            void verifyMessageKeys(const MobitoppMsgType &type, const nlohmann::json &msg) {
                const auto requiredKeys = requiredKeysForMsg[type];
                for (const auto &key: requiredKeys) {
                    if (!msg.contains(key))
                        throw MobitoppMessageParseError(
                                "Message of type '" + mobitoppMsgTypeNames[type] + "' is missing key '" + key + "'.",
                                msg.dump());
                }
            }

            bool handleMobitoppError(const nlohmann::json &) {
                return false;
            }

            bool handleInitCommunication(const nlohmann::json &msg) {
                if (state != WAITING_FOR_INITIALIZATION)
                    throw MobitoppCommError("Received 'init_communication' but already initialized.");

                const bool statusCode = msg["status_code"];
                if (!statusCode)
                    throw MobitoppCommError("Init status code 0 received.");
                sim.init();
                state = WAITING_FOR_NEW_EVENT;
                sendInitCommunication();
                return true;
            }

            bool handleFleetControlAndUpdate(const nlohmann::json &msg) {
                if (state != WAITING_FOR_NEW_EVENT)
                    throw MobitoppCommError("Received 'fleet_control_and_update' but " +
                                            (state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                 : "waiting for response to offer."));
                const int time = msg["time"];
                const auto mobiToppArrivals = sim.handleFleetControlAndUpdate(time);
                sendArrivalEvents(time, mobiToppArrivals);
                state = WAITING_FOR_NEW_EVENT;
                return true;
            }

            bool handleRequestOffer(const nlohmann::json &msg) {
                if (state != WAITING_FOR_NEW_EVENT)
                    throw MobitoppCommError("Received 'request_offer' but " +
                                            (state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                 : "waiting for response to offer."));
                const int agentId = msg["agent_id"];
                const int origin = msg["origin"];
                const int destination = msg["destination"];
                const int time = msg["time"];
                const int numRiders = msg["nr_pax"];

                const int requestId = agentIdForRequest.size();
                agentIdForRequest.push_back(agentId);

                Request r = {requestId, origin, destination, time, numRiders};
                const auto offer = sim.handleRequest(r);
                sendOffer(agentId, offer);
                state = WAITING_FOR_OFFER_RESPONSE;
                return true;
            }


            bool handleFirstMileRequestOffer(const nlohmann::json &msg) {
                if (state != WAITING_FOR_NEW_EVENT)
                    throw MobitoppCommError("Received 'first_mile_request_offer' but " +
                                            (state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                 : "waiting for response to offer."));

                const int agentId = msg["agent_id"];
                const int origin = msg["origin"];
                const nlohmann::json destinations = msg["destinations"];
                const int time = msg["time"];
                const int numRiders = msg["nr_pax"];

                std::vector<TransferAfterFirstMile> transfers;
                for (const auto &destination: destinations) {
                    if (destination["type"] != "intermodal_request")
                        throw MobitoppMessageParseError(
                                "Destination in 'first_mile_request_offer' is not of type 'intermodal_request'.",
                                msg.dump());
                    verifyMessageKeys(INTERMODAL_REQUEST, destination);
                    transfers.push_back({destination["transfer"], destination["time"]});
                }


                const int requestId = agentIdForRequest.size();
                agentIdForRequest.push_back(agentId);

                FirstMileRequest r = {requestId, origin, transfers, time, numRiders};
                const auto offers = sim.handleFirstMileRequest(r);
                sendFirstMileOffers(agentId, offers);
                state = WAITING_FOR_OFFER_RESPONSE;
                return true;
            }

            bool handleLastMileRequestOffer(const nlohmann::json &msg) {
                if (state != WAITING_FOR_NEW_EVENT)
                    throw MobitoppCommError("Received 'last_mile_request_offer' but " +
                                            (state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                 : "waiting for response to offer."));

                const int agentId = msg["agent_id"];
                const nlohmann::json origins = msg["origins"];
                const int destination = msg["destination"];
                const int numRiders = msg["nr_pax"];

                std::vector<TransferBeforeLastMile> transfers;
                for (const auto &origin: origins) {
                    if (origin["type"] != "intermodal_request")
                        throw MobitoppMessageParseError(
                                "Origin in 'last_mile_request_offer' is not of type 'intermodal_request'.", msg.dump());
                    verifyMessageKeys(INTERMODAL_REQUEST, origin);
                    transfers.push_back({origin["transfer"], origin["time"]});
                }


                const int requestId = agentIdForRequest.size();
                agentIdForRequest.push_back(agentId);

                LastMileRequest r = {requestId, transfers, destination, numRiders};
                const auto offers = sim.handleLastMileRequest(r);
                sendLastMileOffers(agentId, offers);
                state = WAITING_FOR_OFFER_RESPONSE;
                return true;
            }


            bool handleBookOffer(const nlohmann::json &msg) {
                if (state != WAITING_FOR_OFFER_RESPONSE)
                    throw MobitoppCommError("Received 'book_offer' but " +
                                            (state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                 : "waiting for new event."));

                const int agentId = msg["agent_id"];
                const int offerId = msg["confirms_offer"];
                if (offerId) { // offerId of 0 indicates declined offer
                    sim.insertBookedOffer(agentId, offerId);
                }
                sendConfirmBooking(agentId, offerId);
                state = WAITING_FOR_NEW_EVENT;
                return true;
            }


            bool handleEndOfSimulation(const nlohmann::json &) {
                if (state != WAITING_FOR_NEW_EVENT)
                    throw MobitoppCommError("Received 'end_of_simulation' but " +
                                            (state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                 : "waiting for response to offer."));
                sim.shutDown();
                state = SIMULATION_ENDED;
                return false;
            }


            void sendJsonMsg(const nlohmann::json &msg) {
                std::string msgStr = msg.dump();
                msgStr += '\n';
                strcpy(readbuf.data(), msgStr.c_str());
                write(fs_fd, (void *) readbuf, msgStr.size() + 1);
            }

            void sendError() {
                nlohmann::json msg;
                msg["type"] = "fs_error";
                sendJsonMsg(msg);
            }

            void sendInitCommunication() {
                nlohmann::json msg;
                msg["type"] = "init_communication";
                msg["status_code"] = 1;
                sendJsonMsg(msg);
            }

            template<typename Arrivals>
            void sendArrivalEvents(const int time, const Arrivals &arrivals) {
                nlohmann::json msg;
                msg["type"] = "customers_arriving";
                msg["time"] = time;
                nlohmann::json arrList = nlohmann::json::array();
                for (const auto &arrival: arrivals) {
                    nlohmann::json arrMsg;
                    arrMsg["type"] = "arrival";
                    arrMsg["agent_id"] = agentIdForRequest[arrival.requestId];
                    arrMsg["t_access"] = arrival.walkingTimeToPickup;
                    arrMsg["t_wait"] = arrival.waitTime;
                    arrMsg["t_drive"] = arrival.rideTime;
                    arrMsg["t_egress"] = arrival.walkingTimeFromDropoff;
                    arrMsg["car_id"] = arrival.vehicleId;
                    arrList.push_back(arrMsg);
                }
                msg["list_arrivals"] = arrList;
                sendJsonMsg(msg);
            }

            void sendOffer(const Offer &offer) {
                nlohmann::json msg;
                msg["type"] = "offer";
                msg["agent_id"] = agentIdForRequest[offer.requestId];
                msg["offer_id"] = offer.id;
                msg["t_access"] = offer.walkingTimeToPickup;
                msg["t_wait"] = offer.waitTime;
                msg["t_drive"] = offer.rideTime;
                msg["t_egress"] = offer.walkingTimeFromDropoff;
                msg["fare"] = offer.fare;
                sendJsonMsg(msg);
            }

            template<typename Offers>
            void sendFirstMileOffers(const Offers &offers) {
                assert(offers.size() >= 0);
                const int reqId = offers.begin()->reqId;
                assert(std::all_of(offers.begin(), offers.end(), [&](const auto &o) { o.reqId == reqId; }));
                nlohmann::json msg;
                msg["type"] = "first_mile_offer";
                msg["agent_id"] = agentIdForRequest[reqId];
                nlohmann::json offerList = nlohmann::json::array();
                for (const auto& o : offers) {
                    nlohmann::json oMsg;
                    oMsg["offer_id"] = o.offerId;
                    oMsg["destination"] = o.destination;
                    oMsg["t_access"] = o.walkingTimeToPickup;
                    oMsg["t_wait"] = o.waitTime;
                    oMsg["t_drive"] = o.rideTime;
                    oMsg["t_egress"] = o.walkingTimeFromDropoff;
                    oMsg["fare"] = o.fare;
                    oMsg["objective"] = 0.0f; // GW: objective has no useful meaning in mobiTopp
                }
                msg["offers"] = offerList;
                sendJsonMsg(msg);
            }

            template<typename Offers>
            void sendLastMileOffers(const Offers &offers) {
                assert(offers.size() >= 0);
                const int reqId = offers.begin()->reqId;
                assert(std::all_of(offers.begin(), offers.end(), [&](const auto &o) { o.reqId == reqId; }));
                nlohmann::json msg;
                msg["type"] = "last_mile_offer";
                msg["agent_id"] = agentIdForRequest[reqId];
                nlohmann::json offerList = nlohmann::json::array();
                for (const auto& o : offers) {
                    nlohmann::json oMsg;
                    oMsg["offer_id"] = o.offerId;
                    oMsg["origin"] = o.origin;
                    oMsg["t_access"] = o.walkingTimeToPickup;
                    oMsg["t_wait"] = o.waitTime;
                    oMsg["t_drive"] = o.rideTime;
                    oMsg["t_egress"] = o.walkingTimeFromDropoff;
                    oMsg["fare"] = o.fare;
                    oMsg["objective"] = 0.0f; // GW: objective has no useful meaning in mobiTopp
                }
                msg["offers"] = offerList;
                sendJsonMsg(msg);
            }

            void sendConfirmBooking(const int agentId, const int offerId) {
                nlohmann::json msg;
                msg["type"] = "confirm_booking";
                msg["agent_id"] = agentId;
                msg["offer_id"] = offerId;
                sendJsonMsg(msg);
            }


            int fs_fd; // socket descriptor for socket communication
            std::array<char, BUF_NUM_BYTES> readbuf;

            CommState state;
            std::vector<int> agentIdForRequest; // maps request IDs to agent IDs

            MobitoppEventSimulation &sim;


        };


    public:


        MobitoppEventSimulation(
                const Fleet &fleet, const int numRequests, const int stopTime,
                AssignmentFinderT &assignmentFinder, SystemStateUpdaterT &systemStateUpdater,
                const ScheduledStopsT &scheduledStops,
                const bool verbose = false)
                : fleet(fleet),
                  stopTime(stopTime),
                  assignmentFinder(assignmentFinder),
                  systemStateUpdater(systemStateUpdater),
                  scheduledStops(scheduledStops),
                  vehicleEvents(fleet.size()),
                  riderArrivalEvents(numRequests),
                  vehicleState(fleet.size(), OUT_OF_SERVICE),
                  requestState(numRequests, NOT_RECEIVED),
                  requestData(numRequests, RequestData()),
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
        {
            for (const auto &veh: fleet)
                vehicleEvents.insert(veh.vehicleId, veh.startOfServiceTime);
        }

        void run() {


            const MobitoppEvent event = waitForEvent();
            if (event.type == REQUEST_OFFER) {

            }


            while (!(vehicleEvents.empty() && requestEvents.empty())) {
                // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
                int id, occTime;

                if (requestEvents.empty()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.empty()) {
                    requestEvents.min(id, occTime);
                    handleRequestEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.minKey() < requestEvents.minKey()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                requestEvents.min(id, occTime);
                handleRequestEvent(id, occTime);
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


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void handleRequestReceipt(const int reqId, const int occTime) {
            ++progressBar;
            assert(requestState[reqId] == NOT_RECEIVED);
            assert(requests[reqId].requestTime == occTime);
            Timer timer;

            const auto &request = requests[reqId];
            const auto &asgnFinderResponse = assignmentFinder.findBestAssignment(request);
            systemStateUpdater.writeBestAssignmentToLogger();

            applyAssignment(asgnFinderResponse, reqId, occTime);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        template<typename AssignmentFinderResponseT>
        void applyAssignment(const AssignmentFinderResponseT &asgnFinderResponse, const int reqId, const int occTime) {
            if (asgnFinderResponse.isNotUsingVehicleBest()) {
                requestState[reqId] = WALKING_TO_DEST;
                requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();
                requestData[reqId].depTime = occTime;
                requestData[reqId].walkingTimeToPickup = 0;
                requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.getNotUsingVehicleDist();
                requestEvents.increaseKey(reqId, occTime + asgnFinderResponse.getNotUsingVehicleDist());
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            int id, key;
            requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff
            assert(id == reqId && key == occTime);

            const auto &bestAsgn = asgnFinderResponse.getBestAssignment();
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
            systemStateUpdater.insertBestAssignment(pickupStopId, dropoffStopId);
            systemStateUpdater.writePerformanceLogs();
            assert(pickupStopId >= 0 && dropoffStopId >= 0);

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


        const Fleet &fleet;
        const int stopTime;
        AssignmentFinderT &assignmentFinder;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;

        AddressableQuadHeap vehicleEvents;
        AddressableQuadHeap riderArrivalEvents;

        std::vector<VehicleState> vehicleState;
        std::vector<RequestState> requestState;

        std::vector<RequestData> requestData;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &assignmentQualityStats;
        std::ofstream &legStatsLogger;
//        ProgressBar progressBar;

    };
}

