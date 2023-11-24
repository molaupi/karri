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

    class SingleClientBlockingSocketServer : public SocketCommunicator {

    public:

        explicit SingleClientBlockingSocketServer(const int port) : SocketCommunicator(port), listening_for_clients_fd(-1) {}

        void init() override {
            struct sockaddr_in address;
            socklen_t addrlen = sizeof(address);

            // Creating socket file descriptor
            if ((listening_for_clients_fd = socket(AF_LOCAL, SOCK_STREAM, 0)) < 0) {
                throw SocketError("Establishing listening socket failed.");
            }

            address.sin_family = AF_LOCAL;
            address.sin_addr.s_addr = htonl(INADDR_ANY); // address is localhost
            address.sin_port = htons(port); // todo: host to network byte order necessary?

            // Forcefully attaching socket to the port
            if (bind(listening_for_clients_fd, (struct sockaddr *) &address, addrlen)
                < 0) {
                throw SocketError("Binding to socket failed.");
            }
            if (listen(listening_for_clients_fd, 1) < 0) {
                throw SocketError("Listen to socket connections failed.");
            }

            // Use first connection as client.
            struct sockaddr_in tain;
            int size = sizeof(tain);
            if ((fd = accept(listening_for_clients_fd, (struct sockaddr *) &tain, (socklen_t *) &size)) < 0) {
                throw SocketError("Accepting connection from client failed.");
            }

            close(listening_for_clients_fd);
        }

    private:

        int listening_for_clients_fd;

    };

}


