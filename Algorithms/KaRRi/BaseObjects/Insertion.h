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
        Assignment *asgn = nullptr;
        int requestId = INVALID_ID;
        int requestTime = -1;
        int passengerArrAtPickup = -1;
        int maxDepTimeAtPickup = -1;
        int maxArrTimeAtDropoff = -1;
        bool enteredCar = false;
    };
}