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

#include <vector>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <nlohmann/json.hpp>
#include <netinet/in.h>
#include <iostream>
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Tools/StringHelpers.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "DataStructures/Queues/AddressableKHeap.h"
#include "Algorithms/KaRRi/EventSimulations/MobitoppErrors.h"
#include "Tools/SocketIO/SingleClientBlockingSocketServer.h"
#include "Algorithms/KaRRi/EventSimulations/MobitoppMessageHelpers.h"

class MockMobitoppRequests {

    static constexpr bool VERBOSE = true;

public:

    MockMobitoppRequests(const std::vector<karri::Request> &requests, const int port)
            : requests(requests),
              requestEvents(requests.size()),
              progressBar(requests.size(), true),
              io(port) {}

    void run() {
        try {
            init();
            mockSimulation();
        } catch (socketio::SocketError& e) {
            std::cerr << "Socket IO Error: " << e.what() << std::endl;
        } catch (MobitoppCommError &e) {
            std::cerr << e.what() << std::endl;
            sendError();
        } catch (MobitoppMessageParseError &e) {
            std::cerr << e.what() << std::endl;
            if (VERBOSE)
                std::cerr << "Message received: " << e.receivedMessage << std::endl;
            sendError();
        }
        shutDown();
    }

private:

    // Establish socket connection
    void init() {
        io.init();

        // Sort requests into event queue ordered by issuing time
        for (const auto &req: requests)
            requestEvents.insert(req.requestId, req.issuingTime);
    }

    // Mock messages that would be sent by mobiTopp
    void mockSimulation() {
        static constexpr bool initStatusCode = true;
        nlohmann::json initMsg;
        initMsg["type"] = "init_communication";
        initMsg["status_code"] = initStatusCode;
        sendJsonMsg(initMsg);
        verifyMessage(listenAndGetMsgJson(), "init_communication");

        int id, key;
        while (!requestEvents.empty()) {
            requestEvents.deleteMin(id, key);
            const auto &req = requests[id];
            assert(req.issuingTime == key);

            nlohmann::json controlMsg;
            controlMsg["type"] = "fleet_control_and_update";
            controlMsg["time"] = req.minDepTime;
            sendJsonMsg(controlMsg);
            verifyMessage(listenAndGetMsgJson(), "customers_arriving");

            nlohmann::json reqMsg;
            reqMsg["type"] = "request_offer";
            reqMsg["agent_id"] = req.requestId;
            reqMsg["origin"] = req.origin;
            reqMsg["destination"] = req.destination;
            reqMsg["time"] = req.minDepTime;
            reqMsg["nr_pax"] = req.numRiders;
            sendJsonMsg(reqMsg);
            const auto offerMsg = listenAndGetMsgJson();
            verifyMessage(offerMsg, "offer");
            if (!offerMsg.contains("offer_id"))
                throw MobitoppMessageParseError("Offer Message JSON does not contain 'offer_id' field.",
                                                offerMsg.dump());
            const int offerId = offerMsg["offer_id"];
            if (offerId <= 0)
                throw MobitoppCommError("Offer ID not greater than 0.");

            nlohmann::json bookMsg;
            bookMsg["type"] = "book_offer";
            bookMsg["agent_id"] = req.requestId;
            bookMsg["confirms_offer"] = offerId;
            sendJsonMsg(bookMsg);
            verifyMessage(listenAndGetMsgJson(), "confirm_booking");

            ++progressBar;
        }

        nlohmann::json endMsg;
        endMsg["type"] = "end_of_simulation";
        sendJsonMsg(endMsg);
    }

    void shutDown() {
        io.shutDown();
    }


    nlohmann::json listenAndGetMsgJson() {
        return parseMobitoppJsonMsg(io.receive());
    }

    void verifyMessage(const nlohmann::json& msg, const std::string& expectedType) {
        if (!checkMsgType(msg, expectedType))
            throw MobitoppMessageParseError("Message does not have expected type " + expectedType, msg.dump());

        verifyMessageKeys(msg, expectedType);
    }

    static bool checkMsgType(const nlohmann::json &msg, const std::string &type) {
        if (!msg.contains("type"))
            throw MobitoppMessageParseError("Message JSON does not contain 'type' field.", msg.dump());
        return msg["type"] == type;
    }

    void sendJsonMsg(const nlohmann::json &msg) {
        std::string msgStr = msg.dump();
        msgStr += '\n';
        io.send(msgStr);
    }


    void sendError() {
        nlohmann::json msg;
        msg["type"] = "mt_error";
        try {
            sendJsonMsg(msg);
        } catch (MobitoppCommError &e) {
            std::cerr << "Could not send error." << std::endl;
        }
    }

    std::unordered_map<std::string, std::vector<std::string>> requiredKeysForMsgType = {
            {"fs_error", {}},
            {"init_communication", {"status_code"}},
            {"customers_arriving", {"time", "list_arrivals"}},
            {"arrival", {"agent_id", "t_access", "t_wait", "t_drive", "t_egress", "car_id"}},
            {"offer", {"agent_id", "offer_id", "t_access", "t_wait", "t_drive", "t_egress", "fare"}},
            {"first_mile_offers", {"agent_id", "offers"}},
            {"first_mile_offer",
             {"offer_id", "destination", "t_access", "t_wait", "t_drive", "t_egress", "fare", "objective"}},
            {"last_mile_offers", {"agent_id", "offers"}},
            {"last_mile_offer",
             {"offer_id", "origin", "t_access", "t_wait", "t_drive", "t_egress", "fare", "objective"}},
            {"confirm_booking", {"agent_id", "offer_id"}}
    };

    void verifyMessageKeys(const nlohmann::json &msg, const std::string &type) {
        const auto requiredKeys = requiredKeysForMsgType[type];
        for (const auto &key: requiredKeys) {
            if (!msg.contains(key))
                throw MobitoppMessageParseError("Message of type '" + type + "' is missing key '" + key + "'.",
                                                msg.dump());
        }
    }


    const std::vector<karri::Request> &requests;
    AddressableQuadHeap requestEvents;
    ProgressBar progressBar;
    socketio::SingleClientBlockingSocketServer io;


};