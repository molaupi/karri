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
#include "Tools/EnumParser.h"
#include "Algorithms/KaRRi/BaseObjects/Offer.h"
#include "MobitoppErrors.h"
#include "Tools/SocketIO/BlockingSocketClient.h"
#include "MobitoppMessageHelpers.h"

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <dirent.h>


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
static const unsigned long NUM_MOBITOPP_MSG_TYPES =
        static_cast<unsigned long>(MobitoppMsgType::INTERMODAL_REQUEST) + 1;

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

namespace karri {
    template<typename FleetSimulationT>
    class MobitoppCommunicator {

        enum CommState {
            WAITING_FOR_INITIALIZATION, // Waiting for initialization message
            WAITING_FOR_NEW_EVENT, // Waiting for new request, fleet control, or end of simulation
            WAITING_FOR_OFFER_RESPONSE, // Sent offer for request, waiting for book_offer message
            SIMULATION_ENDED,
            COMM_ERROR
        };




    public:

        MobitoppCommunicator(FleetSimulationT &sim, const int port) : sim(sim), io(port) {}

        void run() {
            try {
                init();
                while (listen()) {}
            } catch (socketio::SocketError& e) {
                state = COMM_ERROR;
                std::cerr << "Socket IO Error: " << e.what() << std::endl;
            } catch (MobitoppMessageParseError &e) {
                state = COMM_ERROR;
                std::cerr << e.what() << std::endl;
                if (VERBOSE)
                    std::cerr << "Message received: " << e.receivedMessage << std::endl;
                sendError();
                // todo: Do we try to recover errors?
            } catch (MobitoppCommError &e) {
                state = COMM_ERROR;
                std::cerr << e.what() << std::endl;
                sendError();
                // todo: Do we try to recover errors?
            } catch (std::invalid_argument& e) {
                state = COMM_ERROR;
                std::cerr << e.what() << std::endl;
                sendError();
            }
            shutDown();
        }

    private:

        void init() {
            io.init();
            state = WAITING_FOR_INITIALIZATION;
        }


        bool listen() {
            auto msg = io.receive();
            if (VERBOSE)
                std::cout << "Received raw message: " << msg << std::endl;
            auto msgJson = parseMobitoppJsonMsg(std::move(msg));
            return handleMessage(msgJson);
        }

        void shutDown() {
            io.shutDown();
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
                throw MobitoppCommError("Received 'fleet_control_and_update' but " + std::string(
                        state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                            : "waiting for response to offer."));
            const int time = msg["time"];
            const auto mobiToppArrivals = sim.handleFleetControlAndUpdate(time);
            sendArrivalEvents(time, mobiToppArrivals);
            lastFleetControlTime = time;
            state = WAITING_FOR_NEW_EVENT;
            return true;
        }

        bool handleRequestOffer(const nlohmann::json &msg) {
            if (state != WAITING_FOR_NEW_EVENT)
                throw MobitoppCommError("Received 'request_offer' but " + std::string(
                        state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                            : "waiting for response to offer."));
            const int agentId = msg["agent_id"];
            const int origin = msg["origin"];
            const int destination = msg["destination"];
            const int minDepTime = msg["time"];
            const int numRiders = msg["nr_pax"];

            const int requestId = agentIdForRequest.size();
            agentIdForRequest.push_back(agentId);

            Request r = {requestId, origin, destination, numRiders, lastFleetControlTime, minDepTime};
            const auto &offer = sim.handleRequest(r);
            sendOffer(offer);
            state = WAITING_FOR_OFFER_RESPONSE;
            return true;
        }


        bool handleFirstMileRequestOffer(const nlohmann::json &msg) {
            if (state != WAITING_FOR_NEW_EVENT)
                throw MobitoppCommError("Received 'first_mile_request_offer' but " + std::string(
                        state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                            : "waiting for response to offer."));

            const int agentId = msg["agent_id"];
            const int origin = msg["origin"];
            const nlohmann::json &destinations = msg["destinations"];
            const int minDepTime = msg["time"];
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

            FirstMileRequest r = {requestId, origin, transfers, numRiders, lastFleetControlTime, minDepTime};
            const auto offers = sim.handleFirstMileRequest(r);
            sendBestFirstMileOffer(offers, transfers);
            state = WAITING_FOR_OFFER_RESPONSE;
            return true;
        }

        bool handleLastMileRequestOffer(const nlohmann::json &msg) {
            if (state != WAITING_FOR_NEW_EVENT)
                throw MobitoppCommError("Received 'last_mile_request_offer' but " + std::string(
                        state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                            : "waiting for response to offer."));

            const int agentId = msg["agent_id"];
            const nlohmann::json &origins = msg["origins"];
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

            LastMileRequest r = {requestId, transfers, destination, numRiders, lastFleetControlTime};
            const auto offers = sim.handleLastMileRequest(r);
            sendBestLastMileOffer(offers, transfers);
            state = WAITING_FOR_OFFER_RESPONSE;
            return true;
        }


        bool handleBookOffer(const nlohmann::json &msg) {
            if (state != WAITING_FOR_OFFER_RESPONSE)
                throw MobitoppCommError("Received 'book_offer' but " + std::string(
                        state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                            : "waiting for new event."));

            const int agentId = msg["agent_id"];
            const int offerId = msg["confirms_offer"];
            if (offerId) { // offerId of 0 indicates declined offer
                sim.insertBookedOffer(offerId);
            }
            sendConfirmBooking(agentId, offerId);
            state = WAITING_FOR_NEW_EVENT;
            return true;
        }


        bool handleEndOfSimulation(const nlohmann::json &) {
            if (state != WAITING_FOR_NEW_EVENT)
                throw MobitoppCommError("Received 'end_of_simulation' but " +
                                        std::string(state == WAITING_FOR_INITIALIZATION ? "waiting for initialization."
                                                                                        : "waiting for response to offer."));
            sim.shutDown();
            state = SIMULATION_ENDED;
            return false;
        }


        void sendJsonMsg(const nlohmann::json &msg) {
            std::string msgStr = msg.dump();
            msgStr += '\n';
            io.send(msgStr);
        }

        void sendError() {
            nlohmann::json msg;
            msg["type"] = "fs_error";
            try {
                sendJsonMsg(msg);
            } catch (MobitoppCommError& e) {
                std::cerr << "Could not send error." << std::endl;
            }
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
            for (const auto &reqDataPtr: arrivals) {
                const auto &reqData = *reqDataPtr;
                nlohmann::json arrMsg;
                arrMsg["type"] = "arrival";
                arrMsg["agent_id"] = agentIdForRequest[reqData.requestId];
                arrMsg["t_access"] = reqData.walkingTimeToPickup;
                arrMsg["t_wait"] = reqData.depTime - reqData.arrTime; // todo: Inclusive walking time or exclusive?
                arrMsg["t_drive"] = reqData.arrTime - reqData.walkingTimeFromDropoff - reqData.depTime;
                arrMsg["t_egress"] = reqData.walkingTimeFromDropoff;
                arrMsg["car_id"] = reqData.vehicleId;
                arrList.push_back(arrMsg);
            }
            msg["list_arrivals"] = arrList;
            sendJsonMsg(msg);
        }

        void sendOffer(const Offer &offer) {
            nlohmann::json msg;
            msg["type"] = "offer";
            msg["agent_id"] = agentIdForRequest[offer.requestId];
            msg["offer_id"] = offer.offerId;
            msg["t_access"] = offer.walkingTimeToPickup;
            msg["t_wait"] = offer.waitTime;
            msg["t_drive"] = offer.rideTime;
            msg["t_egress"] = offer.walkingTimeFromDropoff;
            msg["fare"] = offer.fare;
            sendJsonMsg(msg);
        }

        template<typename Offers>
        void sendAllFirstMileOffers(const Offers &offers) {
            assert(offers.size() >= 0);
            const int reqId = offers.begin()->reqId;
            assert(std::all_of(offers.begin(), offers.end(), [&](const auto &o) { return o.reqId == reqId; }));
            nlohmann::json msg;
            msg["type"] = "first_mile_offer";
            msg["agent_id"] = agentIdForRequest[reqId];
            nlohmann::json offerList = nlohmann::json::array();
            for (const auto &o: offers) {
                nlohmann::json oMsg;
                oMsg["offer_id"] = o.offerId;
                oMsg["destination"] = o.destination;
                oMsg["t_access"] = o.walkingTimeToPickup;
                oMsg["t_wait"] = o.waitTime;
                oMsg["t_drive"] = o.rideTime;
                oMsg["t_egress"] = o.walkingTimeFromDropoff;
                oMsg["fare"] = o.fare;
                oMsg["objective"] = 0.0f; // GW: objective has no useful meaning in mobiTopp
                offerList.push_back(oMsg);
            }
            msg["offers"] = offerList;
            sendJsonMsg(msg);
        }

        template<typename Offers, typename Transfers>
        void sendBestFirstMileOffer(const Offers &offers, const Transfers &transfers) {
            assert(offers.size() >= 0);
            const int reqId = offers.begin()->requestId;
            assert(std::all_of(offers.begin(), offers.end(), [&](const auto &o) { return o.requestId == reqId; }));

            // We choose the offer with the smallest total trip time
            auto minTotalTripTime = INFTY;
            Offer const *bestOffer = nullptr;
            assert(offers.size() == transfers.size());
            for (int i = 0; i < offers.size(); ++i) {
                const auto totalTripTime = offers[i].waitTime + offers[i].rideTime + offers[i].walkingTimeFromDropoff +
                                           transfers[i].timeFromTransferToDest; // todo: does waitTime include walking time?
                if (totalTripTime < minTotalTripTime) {
                    minTotalTripTime = totalTripTime;
                    bestOffer = &offers[i];
                }
            }

            nlohmann::json msg;
            msg["type"] = "first_mile_offer";
            msg["agent_id"] = agentIdForRequest[reqId];

            nlohmann::json offerList = nlohmann::json::array();
            nlohmann::json oMsg;
            if (bestOffer) {
                const auto &o = *bestOffer;
                oMsg["offer_id"] = o.offerId;
                oMsg["destination"] = o.destination;
                oMsg["t_access"] = o.walkingTimeToPickup;
                oMsg["t_wait"] = o.waitTime;
                oMsg["t_drive"] = o.rideTime;
                oMsg["t_egress"] = o.walkingTimeFromDropoff;
                oMsg["fare"] = o.fare;
                oMsg["objective"] = 0.0f; // GW: objective has no useful meaning in mobiTopp
                offerList.push_back(oMsg);
            }
            msg["offers"] = offerList;

            sendJsonMsg(msg);
        }

        template<typename Offers>
        void sendAllLastMileOffers(const Offers &offers) {
            assert(offers.size() >= 0);
            const int reqId = offers.begin()->reqId;
            assert(std::all_of(offers.begin(), offers.end(), [&](const auto &o) { return o.reqId == reqId; }));
            nlohmann::json msg;
            msg["type"] = "last_mile_offer";
            msg["agent_id"] = agentIdForRequest[reqId];
            nlohmann::json offerList = nlohmann::json::array();
            for (const auto &o: offers) {
                nlohmann::json oMsg;
                oMsg["offer_id"] = o.offerId;
                oMsg["origin"] = o.origin;
                oMsg["t_access"] = o.walkingTimeToPickup;
                oMsg["t_wait"] = o.waitTime;
                oMsg["t_drive"] = o.rideTime;
                oMsg["t_egress"] = o.walkingTimeFromDropoff;
                oMsg["fare"] = o.fare;
                oMsg["objective"] = 0.0f; // GW: objective has no useful meaning in mobiTopp
                offerList.push_back(oMsg);
            }
            msg["offers"] = offerList;
            sendJsonMsg(msg);
        }


        template<typename Offers, typename Transfers>
        void sendBestLastMileOffer(const Offers &offers, const Transfers &transfers) {
            assert(offers.size() >= 0);
            const int reqId = offers.begin()->requestId;
            assert(std::all_of(offers.begin(), offers.end(), [&](const auto &o) { return o.requestId == reqId; }));

            // We choose the offer with the smallest total trip time (equivalent to earliest arrival time)
            auto minArrTime = INFTY;
            Offer const *bestOffer = nullptr;
            assert(offers.size() == transfers.size());
            for (int i = 0; i < offers.size(); ++i) {
                const auto arrTime = transfers[i].minDepTimeAtTransfer + offers[i].waitTime + offers[i].rideTime +
                                     offers[i].walkingTimeFromDropoff;
                if (arrTime < minArrTime) {
                    minArrTime = arrTime;
                    bestOffer = &offers[i];
                }
            }

            nlohmann::json msg;
            msg["type"] = "last_mile_offer";
            msg["agent_id"] = agentIdForRequest[reqId];

            nlohmann::json offerList = nlohmann::json::array();
            if (bestOffer) {
                nlohmann::json oMsg;
                const auto &o = *bestOffer;
                oMsg["offer_id"] = o.offerId;
                oMsg["origin"] = o.origin;
                oMsg["t_access"] = o.walkingTimeToPickup;
                oMsg["t_wait"] = o.waitTime;
                oMsg["t_drive"] = o.rideTime;
                oMsg["t_egress"] = o.walkingTimeFromDropoff;
                oMsg["fare"] = o.fare;
                oMsg["objective"] = 0.0f; // GW: objective has no useful meaning in mobiTopp
                offerList.push_back(oMsg);
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


        FleetSimulationT &sim;
        socketio::BlockingSocketClient io;

        static constexpr bool VERBOSE = true;


        CommState state;
        int lastFleetControlTime = 0;

        std::vector<int> agentIdForRequest; // maps request IDs to agent IDs
    };

} // end of namespace
