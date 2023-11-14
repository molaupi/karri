//
// Created by tim on 09.11.23.
//

#pragma once

#include <cassert>

#include "Vehicle.h"
#include "Request.h"
#include "Assignment.h"

namespace karri {
    struct Insertion {
        Assignment &asgn;
        int requestId;
        int requestTime;
        int passengerArrAtPickup;
        int maxDepTimeAtPickup;
        int maxArrTimeAtDropoff;
        bool enteredCar = false;
    };
}