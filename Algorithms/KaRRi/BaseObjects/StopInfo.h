#pragma once

#include <vector>
#include <set>
#include "Assignment.h"

namespace karri {
    struct StopInfo {
        PDLoc location;
        int vehicleId;

        int schedDepTime;   //schedDepTime[start + 1] in RouteState
        int maxArrTime;     //maxArrTime[start + 1] in RouteState

        int schedArrTime;
        int numOfPeoplePickedUp;
        int numOfPeopleDroppedoff;

        std::vector<int> pickedupRequests;
        std::vector<int> droppedoffRequests;

        std::set<int> dropOffIds; // stopIds (in RouteState) of dropoffs for requests that are picked up at this stop

        int insertIndex;  //Index after which the Stopp should be inserted  (always 0 for pickups)

        int maxArrTimeAtDropoff; // The minimal maxArrTimeAtDropoff of all requests that are dropped off at this stop

        bool isFixed = false;


        StopInfo(PDLoc loc, const int vehId, const int schedDepTime, const int maxArrTime, const int schedArrTime, const int maxArrTimeAtDropoff):
                location(loc),
                vehicleId(vehId),
                schedDepTime(schedDepTime),
                maxArrTime(maxArrTime),
                schedArrTime(schedArrTime),
                maxArrTimeAtDropoff(maxArrTimeAtDropoff) {
        }
    };
}