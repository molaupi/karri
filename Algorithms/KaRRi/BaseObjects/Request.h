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

namespace karri {

    // Models a request with an ID, an origin location, a destination location and the earliest possible departure time.
    struct Request {
        int requestId = INVALID_ID;
        int origin = INVALID_EDGE;
        int destination = INVALID_EDGE;
        int requestTime = INFTY;
    };

    // Models a location used for a pickup or dropoff with an ID (should be counted separately for pickups and dropoffs), a
    // location, a walking distance, and optional driving distances to and from the associated origin or destination
    // location.
    struct PDLoc {

        int id = INVALID_ID; // Should be counted separately for pickups and dropoffs
        int loc = INVALID_EDGE; // Location in road network
        int psgLoc = INVALID_EDGE; // Location in passenger road network
        int walkingDist = INFTY; // Walking time from origin to this pickup or from this dropoff to destination.

        int vehDistToCenter = INFTY; // Vehicle driving time from this pickup/dropoff to the origin/destination.
        int vehDistFromCenter = INFTY; // Vehicle driving time from origin/destination to this pickup/dropoff.
    };


    enum PDLocType : std::int8_t {
        PICKUP,
        DROPOFF,
        INVALID_PD_LOC_TYPE
    };

}