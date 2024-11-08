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

    class SingleClientBlockingStringSocketServer : public StringSocketCommunicator {

    public:

        explicit SingleClientBlockingStringSocketServer(const int port) : StringSocketCommunicator(port), listening_for_clients_fd(-1) {}

        ~SingleClientBlockingStringSocketServer() {
            close(listening_for_clients_fd);
        }

        void init() override {
            struct sockaddr_in address;
            socklen_t addrlen = sizeof(address);

            // Creating socket file descriptor
            if ((listening_for_clients_fd = socket(AF_LOCAL, SOCK_STREAM, 0)) < 0) {
                throw SocketError("Establishing listening socket failed.");
            }

            // Allow reusing of previously used ports
            static constexpr int yes = 1;
            if (setsockopt(listening_for_clients_fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes) == -1) {
                throw SocketError("setsockopt for reusage of address failed.");
            }
#ifdef SO_REUSEPORT
            if (setsockopt(listening_for_clients_fd,SOL_SOCKET,SO_REUSEPORT,&yes,sizeof yes) == -1) {
                throw SocketError("setsockopt for reusage of ports failed.");
            }
#endif

            address.sin_family = AF_LOCAL;
            address.sin_addr.s_addr = htonl(INADDR_ANY); // address is localhost
            address.sin_port = htons(port);

            // Forcefully attaching socket to the port
            if ((bind(listening_for_clients_fd, (struct sockaddr *) &address, addrlen)) < 0) {
                if (errno == EADDRINUSE)
                    throw SocketError("Binding to socket failed with error EADDRINUSE.");
                if (errno == EINVAL)
                    throw SocketError("Binding to socket failed with error EINVAL.");
                throw SocketError("Binding to socket failed with error code " + std::to_string(errno) + ".");
            }

            if (listen(listening_for_clients_fd, 1) < 0) {
                throw SocketError("Listen to socket connections failed.");
            }

            std::cout << "Listening to client connection on addr " << address.sin_addr.s_addr << " and port " << address.sin_port << "..." << std::flush;
            // Use first connection as client.
            struct sockaddr_in tain;
            int size = sizeof(tain);
            if ((fd = accept(listening_for_clients_fd, (struct sockaddr *) &tain, (socklen_t *) &size)) < 0) {
                throw SocketError("Accepting connection from client failed.");
            }

            // Allow reusing of previously used ports
            if (setsockopt(fd, SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes) == -1) {
                throw SocketError("setsockopt for reusage of address failed.");
            }
#ifdef SO_REUSEPORT
            if (setsockopt(fd, SOL_SOCKET,SO_REUSEPORT,&yes,sizeof yes) == -1) {
                throw SocketError("setsockopt for reusage of ports failed.");
            }
#endif
            std::cout << " done." << std::endl;

            close(listening_for_clients_fd);
        }

    private:

        int listening_for_clients_fd;

    };

}


