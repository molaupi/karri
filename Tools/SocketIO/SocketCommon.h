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

#include <stdexcept>
#include <array>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <netinet/in.h>
#include <iostream>
#include "SocketCommon.h"
#include "Tools/Workarounds.h"

namespace socketio {

    // Number of bytes that can be read in one read via sockets.
    static constexpr int BUFFER_SIZE = 8192;

    struct SocketError : public std::runtime_error {
        explicit SocketError(const std::string &what) : std::runtime_error(what) {}
    };



    // Abstract base class for socket based communication of strings.
    // Each message is a C-style string terminated by '\0' byte.
    class StringSocketCommunicator {
    public:

        StringSocketCommunicator(const int port) : port(port) {}

        ~StringSocketCommunicator() {
            close(fd);
        }

        // Server and client establish socket file descriptor fd in their implementation of init().
        virtual void init() = 0;

        void sendString(const std::string &msg) {
            if (msg.size() + 1 > BUFFER_SIZE)
                throw SocketError("Message size of " + std::to_string(msg.size()) +
                                  " plus '\\0' terminator exceeds buffer size of " + std::to_string(BUFFER_SIZE));

            ssize_t bytesSent = send(fd, (void *) msg.c_str(), msg.size() + 1, MSG_NOSIGNAL);

            if (bytesSent < 0)
                throw SocketError("Communication partner has unexpectedly shut down.");

            if (bytesSent != msg.size() + 1)
                throw SocketError("Tried sending " + std::to_string(msg.size() + 1) + " bytes but could only send " +
                                  std::to_string(bytesSent) + " .");
        }

        // Receive the next string.
        //
        // If any string can be read, the first string is removed from the socket queue, the string is written to msg,
        // and true is returned.
        // If communication has been shut down by communication partner, false is returned and socket queue is
        // left unchanged (so EOF remains in queue and regular shutdown can be performed).
        //
        // Throws SocketError for any errors during communication.
        bool receiveString(std::string& msg) {

            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd, &readfds);
            struct timeval tv;
            tv.tv_sec = 60; // waiting for message times out after 60 seconds (resets when part of a message comes in)
            tv.tv_usec = 0;

            int rv;
            int lenOfMsg = 0; // length of received string (excluding terminator symbol)
            while ((rv = select(fd + 1, &readfds, NULL, NULL, &tv))) {
                if (rv < 0)
                    throw SocketError("Error during select. No message could be received.");
                LIGHT_KASSERT(FD_ISSET(fd, &readfds));

                // Peek at queue, reading contents without removing from queue yet.
                ssize_t numBytesRead = recv(fd, (void *) buf.data(), BUFFER_SIZE - 1, MSG_PEEK);

                // If 0 bytes were received, communication partner has shut down communication.
                // Leave EOF in queue and return false.
                if (numBytesRead == 0)
                    return false;

                // If queue contains at least one string terminator, store number of characters in first string and
                // break, allowing read of that first string.
                while (lenOfMsg < numBytesRead && buf[lenOfMsg] != '\0') {
                    ++lenOfMsg;
                }
                if (buf[lenOfMsg] == '\0')
                    break;
            }
            if (rv == 0)
                throw SocketError("Waiting for message timed out after" + std::to_string(tv.tv_sec) +  " seconds.");

            // Actually read message, removing first string from queue.
            ssize_t numBytesRead = recv(fd, (void *) buf.data(), lenOfMsg + 1, 0);
            if (numBytesRead != lenOfMsg + 1)
                throw SocketError("Receiving message of length " + std::to_string(lenOfMsg + 1) + " failed.");
            LIGHT_KASSERT(buf[lenOfMsg] == '\0');
            msg = buf.data();
            return true;
        }

        void shutDown() {
            close(fd);
        }

        // Receive messages (and do nothing with them) until partner shuts down.
        void waitForPartnerShutdown() {
            while (recv(fd, (void*) buf.data(), BUFFER_SIZE - 1, 0)) {}
        }

    protected:

        const int port;
        int fd;

    private:

        std::array<char, BUFFER_SIZE> buf;
    };


}