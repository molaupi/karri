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

class MockMobitoppRequests {

    // todo: Read config file for potential other ports
    static constexpr int BUF_NUM_BYTES = 4096;

    // todo: Input argument for verbose
    static constexpr bool VERBOSE = true;

    struct MobitoppMessageParseError : public std::runtime_error {

        explicit MobitoppMessageParseError(const std::string &what, std::string receivedMessage = "<omitted>")
                : std::runtime_error(what), receivedMessage(std::move(receivedMessage)) {}

        std::string receivedMessage;
    };

    struct MobitoppCommError : public std::runtime_error {
        explicit MobitoppCommError(const std::string &what) : std::runtime_error(what) {}
    };

public:

    MockMobitoppRequests(const std::vector<karri::Request> &requests, const int port)
            : requests(requests),
              requestEvents(requests.size()),
              port(port),
              progressBar(requests.size(), true) {}

    void run() {
        try {
            init();
            mockSimulation();
        } catch (MobitoppCommError &e) {
            std::cerr << e.what() << std::endl;
            sendError();
        } catch (MobitoppMessageParseError &e) {
            std::cerr << e.what() << std::endl;
            if (VERBOSE)
                std::cerr << "Message received: " << e.receivedMessage << std::endl;
            sendError();
        }
        close(connected_fd);
        close(listening_for_clients_fd);
    }

private:

    // Establish socket connection
    void init() {

        struct sockaddr_in address;
        socklen_t addrlen = sizeof(address);

        // Creating socket file descriptor
        if ((listening_for_clients_fd = socket(AF_LOCAL, SOCK_STREAM, 0)) < 0) {
            throw MobitoppCommError("socket failed.");
        }

        address.sin_family = AF_LOCAL;
        address.sin_addr.s_addr = htonl(INADDR_ANY); // address is localhost
        address.sin_port = htons(port); // todo: host to network byte order necessary?

        // Forcefully attaching socket to the port
        if (bind(listening_for_clients_fd, (struct sockaddr *) &address, addrlen)
            < 0) {
            throw MobitoppCommError("Binding to socket failed.");
        }
        if (listen(listening_for_clients_fd, 1) < 0) {
            throw MobitoppCommError("Listen to socket connections failed.");
        }

        // Use first connection as client.
        struct sockaddr_in tain;
        int size = sizeof(tain);
        if ((connected_fd = accept(listening_for_clients_fd, (struct sockaddr *) &tain, (socklen_t *) &size)) < 0) {
            throw MobitoppCommError("Accepting connection from client failed.");
        }

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
            requestEvents.min(id, key);
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


    nlohmann::json listenAndGetMsgJson() {
        ssize_t numBytesRead = read(connected_fd, (void *) buf.data(), BUF_NUM_BYTES - 1);
        return getMsgJsonFromBuf(numBytesRead);
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

    nlohmann::json getMsgJsonFromBuf(const ssize_t numBytesRead) {
        std::string msg = buf.data();
        unused(numBytesRead);
//        if (msg.size() != numBytesRead) {
//            throw MobitoppMessageParseError("Parsed message has size " + std::to_string(msg.size()) + " but " +
//                                            std::to_string(numBytesRead) + " bytes were read.", msg);
//        }
        if (!endsWith(msg, "\n")) {
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

    void sendJsonMsg(const nlohmann::json &msg) {
        std::string msgStr = msg.dump();
        msgStr += '\n';
        strcpy(buf.data(), msgStr.c_str());
        ssize_t bytesSent = write(connected_fd, (void *) buf.data(), msgStr.size() + 1);
        if (bytesSent != msgStr.size() + 1)
            throw MobitoppCommError(
                    "Tried sending " + std::to_string(msgStr.size() + 1) + " bytes but could only send " +
                    std::to_string(bytesSent) + " .");
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
    const int port;
    ProgressBar progressBar;

    int listening_for_clients_fd, connected_fd;
    std::array<char, BUF_NUM_BYTES> buf;


};