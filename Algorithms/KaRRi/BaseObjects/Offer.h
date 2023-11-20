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

#include "Assignment.h"
#include "Tools/Constants.h"


namespace karri {
    struct Offer {
        int offerId = INVALID_ID;
        int offerTime = INFTY; // time at which offer is made (generally equals time at which request is received)

        // Information on request
        int requestId = INVALID_ID;
        int origin = INVALID_EDGE; // chosen by KaRRi for last mile request, otherwise fixed by mobiTopp request
        int destination = INVALID_EDGE; // chosen by KaRRi for first mile request, otherwise fixed by mobiTopp request
        int numRiders = INFTY;
        int issuingTime = INFTY; // time at which request is issued
        int minDepTimeAtOrigin = INFTY; // earliest possible departure time (may be later than issuingTime)

        int directODDistance = INFTY; // computed by KaRRi

        // Information on assignment
        int vehicleId = INVALID_ID;
        int pickupLoc = INVALID_EDGE; // pickup meeting point chosen by KaRRi
        int walkingTimeToPickup = INFTY;
        int dropoffLoc = INVALID_EDGE; // dropoff meeting point chosen by KaRRi
        int walkingTimeFromDropoff = INFTY;

        int pickupStopIdx = INVALID_INDEX; // Pickup is inserted at or after stop with index pickupStopIdx in route of vehicle
        int dropoffStopIdx = INVALID_INDEX; // Dropoff is inserted at or after stop with index dropoffStopIdx in route of vehicle

        int distToPickup = 0; // Distance from previous stop to pickup
        int distFromPickup = 0; // Distance from pickup to next stop (or 0 if pickupStopIdx == dropoffStopIdx)
        int distToDropoff = 0; // Distance from previous stop to dropoff (or from pickup to dropoff if pickupStopIdx == dropoffStopIdx)
        int distFromDropoff = 0; // Distance from dropoff to next stop (or 0 if there is no next stop)

        int waitTime = INFTY;
        int rideTime = INFTY;
        int fare = INFTY;

        int cost = INFTY; // Cost according to KaRRi cost function.
    };
}