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



    class SocketCommunicator {
    public:

        SocketCommunicator(const int port) : port(port) {}

        // Server and client establish socket file descriptor fd in their implementation of init().
        virtual void init() = 0;

        void send(const std::string &msg) {
            if (msg.size() + 1 > BUFFER_SIZE)
                throw SocketError("Message size of " + std::to_string(msg.size()) +
                                  " plus '\\0' terminator exceeds buffer size of " + std::to_string(BUFFER_SIZE));

            ssize_t bytesSent = write(fd, (void *) msg.c_str(), msg.size() + 1);

            if (bytesSent != msg.size() + 1)
                throw SocketError("Tried sending " + std::to_string(msg.size() + 1) + " bytes but could only send " +
                                  std::to_string(bytesSent) + " .");
        }

        std::string receive() {
            ssize_t numBytesRead = read(fd, (void *) buf.data(), BUFFER_SIZE - 1);
            if (numBytesRead < 0)
                throw SocketError("Receiving message failed.");
            std::string msg = buf.data();
            return msg;
        }

        void shutDown() {
            close(fd);
        }

    protected:
        const int port;
        int fd;

    private:
        std::array<char, BUFFER_SIZE> buf;
    };


}