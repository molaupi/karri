#pragma once

#include <vector>
#include "Assignment.h"

namespace karri {
    struct PickupInfo {
        PDLoc location;
        int vehicleId;

        int requestTime;

        int schedDepTime;   //schedDepTime[start + 1] in RouteState
        int maxArrTime;     //maxArrTime[start + 1] in RouteState

        int schedArrTime;
        int numOfPeoplePickedUp;
        int numOfPeopleDroppedoff;

        std::vector<int> pickedupRequests;
    };
}