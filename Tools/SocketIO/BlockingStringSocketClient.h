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
#include <unistd.h>
#include <utility>
#include <netinet/in.h>
#include <iostream>
#include "SocketCommon.h"

namespace socketio {

    class BlockingStringSocketClient : public StringSocketCommunicator {

    public:
        explicit BlockingStringSocketClient(const int port) : StringSocketCommunicator(port) {}

        void init() override {
            struct sockaddr_in address;

            if ((fd = socket(AF_LOCAL, SOCK_STREAM, 0)) < 0) {
                throw SocketError("Socket creation error.");
            }

            static constexpr int yes = 1;
            if (setsockopt(fd, SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes) == -1) {
                throw SocketError("setsockopt for reusage of address failed.");
            }
#ifdef SO_REUSEPORT
            if (setsockopt(fd, SOL_SOCKET,SO_REUSEPORT,&yes,sizeof yes) == -1) {
                throw SocketError("setsockopt for reusage of ports failed.");
            }
#endif

            address.sin_family = AF_LOCAL;
            address.sin_addr.s_addr = htonl(INADDR_ANY); // address is localhost
            address.sin_port = htons(port);

            if (connect(fd, (struct sockaddr *) &address, sizeof(address)) < 0) {
                throw SocketError("Socket connection to server failed.");
            }
            std::cout << "Connected to server on addr " << address.sin_addr.s_addr << " and port " << address.sin_port << std::endl;


        }
    };

} // socketio

