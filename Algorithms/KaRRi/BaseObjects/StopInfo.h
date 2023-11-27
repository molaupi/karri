#pragma once

#include <vector>
#include <set>
#include "Assignment.h"

namespace karri {
    struct StopInfo {
        int location;
        int vehicleId = INVALID_ID;

        int schedDepTime = -1;
        int maxArrTime = -1;
        int schedArrTime = -1;

        std::vector<std::pair<int, int>> pickedupReqAndDropoff; // Stores (unsaved) pairs of requests (that are picked up at this stop) and their dropoff stopIds

        int insertIndex = -1;  //Index where the stop should be inserted

        int maxArrTimeAtDropoff = INFTY;



        StopInfo(int loc, const int vehId, const int schedDepTime, const int maxArrTime, const int schedArrTime):
                location(loc),
                vehicleId(vehId),
                schedDepTime(schedDepTime),
                maxArrTime(maxArrTime),
                schedArrTime(schedArrTime) {
        }

        StopInfo()= default;
    };
}