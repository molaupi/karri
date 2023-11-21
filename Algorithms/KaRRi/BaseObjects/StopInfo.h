#pragma once

#include <vector>
#include <set>
#include "Assignment.h"

namespace karri {
    struct StopInfo {
        PDLoc location;
        int vehicleId = INVALID_ID;

        int schedDepTime = -1;
        int maxArrTime = -1;
        int schedArrTime = -1;

        std::vector<int> pickedupRequests;
        std::vector<int> droppedoffRequests;

        std::set<int> dropOffIds; // stopIds (in RouteState) of dropoffs for requests that are picked up at this stop

        int insertIndex = -1;  //Index where the Stopp should be inserted

        int maxArrTimeAtDropoff = INFTY; // The minimal maxArrTimeAtDropoff of all requests that are dropped off at this stop

        bool isFixed = false;



        StopInfo(PDLoc loc, const int vehId, const int schedDepTime, const int maxArrTime, const int schedArrTime):
                location(loc),
                vehicleId(vehId),
                schedDepTime(schedDepTime),
                maxArrTime(maxArrTime),
                schedArrTime(schedArrTime) {
        }

        StopInfo()= default;
    };
}