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

#include "Tools/Constants.h"
#include "Assignment.h"
#include "PDLoc.h"

namespace karri {

    // Models a request with an ID, an origin location, a destination location and the earliest possible departure time.
    struct Request {
        int requestId = INVALID_ID;
        int origin = INVALID_EDGE;
        int destination = INVALID_EDGE;
        int numRiders = INFTY;

        int issuingTime = INFTY; // Time at which request is issued
        int minDepTime = INFTY; // Earliest possible departure time (prebooking if > issuingTime)
    };




    // Models a potential transfer to a different transportation method when taxi sharing is used for the first mile,
    // i.e. the trip from a fixed origin to the transfer point.
    struct TransferAfterFirstMile {
        int loc = INVALID_EDGE;
        int timeFromTransferToDest = INFTY; // Remaining travel time from transfer using other means of transport.
    };

    // Models part of an intermodal request where taxi sharing is used for the first mile, i.e. the trip from a fixed
    // origin to a point of transfer to a different transportation method. Contains multiple possible transfer points
    // but a taxi sharing offer for only one of the points is expected in return.
    struct FirstMileRequest {
        int requestId = INVALID_ID;
        int origin = INVALID_EDGE;
        std::vector<TransferAfterFirstMile> transfers = {};
        int numRiders = INFTY;

        int issuingTime = INFTY; // Time at which request is issued
        int minDepTime = INFTY; // Earliest possible departure time (prebooking if > issuingTime)
    };

    // Models a potential transfer to a different transportation method when taxi sharing is used for the last mile,
    // i.e. the trip from a transfer point to a fixed destination.
    struct TransferBeforeLastMile {
        int loc = INVALID_EDGE;
        int minDepTimeAtTransfer = INFTY; // Earliest possible departure time at transfer (defined by arrival using other means of transport)
    };

    // Models part of an intermodal request where taxi sharing is used for the last mile, i.e. the trip from a point of
    // transfer to a fixed destination. Contains multiple possible transfer points but a taxi sharing offer for only
    // one of the points is expected in return.
    struct LastMileRequest {
        int requestId = INVALID_ID;
        std::vector<TransferBeforeLastMile> transfers = {};
        int destination = INVALID_EDGE;
        int numRiders = INFTY;

        int issuingTime = INFTY; // Time at which request is issued
    };

} // end namespace