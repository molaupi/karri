//
// Created by tim on 14.11.23.
//

#pragma once

#include "Assignment.h"

namespace karri {
    struct Insertion {
        //Assignment info
        int vehicleId = INVALID_ID;
        PDLoc pickup;
        PDLoc dropoff;

        int pickupStopIdx = INVALID_INDEX; // Pickup is inserted at or after stop with index pickupStopIdx in route of vehicle
        int dropoffStopIdx = INVALID_INDEX; // Dropoff is inserted at or after stop with index dropoffStopIdx in route of vehicle

        int distToPickup = 0; // Distance from previous stop to pickup
        int distFromPickup = 0; // Distance from pickup to next stop (or 0 if pickupStopIdx == dropoffStopIdx)
        int distToDropoff = 0; // Distance from previous stop to dropoff (or from pickup to dropoff if pickupStopIdx == dropoffStopIdx)
        int distFromDropoff = 0; // Distance from dropoff to next stop (or 0 if there is no next stop)

        //StopId of dropoff in RouteState
        int dropoffId = INVALID_ID;

        // RequestState info
        int requestId = INVALID_ID;
        int requestTime = -1;
        int passengerArrAtPickup = -1;
        int maxDepTimeAtPickup = -1;
        int maxArrTimeAtDropoff = -1;

        bool enteredCar = false;


        Insertion(int vId, const PDLoc& pick, const PDLoc& drop, int pickIdx, int dropIdx,
                  int reqId, int reqTime, int passArrAtPick, int maxDepAtPick, int maxArrAtDrop)
                : vehicleId(vId), pickup(pick), dropoff(drop),
                  pickupStopIdx(pickIdx), dropoffStopIdx(dropIdx),
                  requestId(reqId), requestTime(reqTime),
                  passengerArrAtPickup(passArrAtPick), maxDepTimeAtPickup(maxDepAtPick), maxArrTimeAtDropoff(maxArrAtDrop),
                  enteredCar(false) {}
    };
}