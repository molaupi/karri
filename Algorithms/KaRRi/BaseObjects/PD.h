/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"

namespace karri {

    // Type of the pickup / dropoff (ordinary, before next stop, after last stop)
    enum PDType {
        ORD,
        BNS,
        ALS
    };


    struct PD {

        PD() {}
        PD(const Vehicle *veh) : vehicle(veh) {}

        // Type of the pickup / dropoff
        PDType type;
        // Vehicle that does the pickup / dropoff
        const Vehicle *vehicle = nullptr;
        // Index after which the pickup / dropoff will be performed
        int pdIdx;
        // Walking distance from origin to pickup / dropoff to destination 
        int walkingDistance;
        // Detour of vehicle to get to the pickup / dropoff
        int detourToPD;
        // Detour of vehicle to get from pickup / dropoff to next scheduled stop
        int detourFromPD;
        // The id of the actual PDLoc
        int pdLocId;

    };

    using Pickup = PD;
    using Dropoff = PD;
    
}


