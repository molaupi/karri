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

#include <cassert>

#include "Vehicle.h"
#include "Request.h"

namespace karri {


    // Models the assignment of a request to the route of a vehicle, specifying the vehicle, the pickup and dropoff
    // locations, the stop indices after which to insert the pickup and dropoff, as well as the distances from the
    // previous stop to either PD-location and from either PD-location to the next stop.
    struct Assignment {

        explicit Assignment(
                const Vehicle *vehicle = nullptr,
                const PDLoc *pickup = nullptr,
                const PDLoc *dropoff = nullptr,
                const int pickupStopIdx = 0,
                const int dropoffStopIdx = 0) noexcept
                : vehicle(vehicle),
                  pickup(pickup),
                  dropoff(dropoff),
                  pickupStopIdx(pickupStopIdx),
                  dropoffStopIdx(dropoffStopIdx) {
            assert(pickupStopIdx >= 0);
            assert(dropoffStopIdx >= 0);
        }

        Assignment(const Vehicle *vehicle,
                   const PDLoc *pickup,
                   const PDLoc *dropoff,
                   const int pickupStopIdx, const int dropoffStopIdx, const int distToPickup,
                   const int distFromPickup, const int distToDropoff, const int distFromDropoff)
                : vehicle(vehicle),
                  pickup(pickup),
                  dropoff(dropoff),
                  pickupStopIdx(pickupStopIdx),
                  dropoffStopIdx(dropoffStopIdx),
                  distToPickup(distToPickup),
                  distFromPickup(distFromPickup),
                  distToDropoff(distToDropoff),
                  distFromDropoff(distFromDropoff) {
            assert(pickupStopIdx >= 0);
            assert(dropoffStopIdx >= 0);
            assert(distToPickup >= 0);
            assert(distFromPickup >= 0);
            assert(distToDropoff >= 0);
            assert(distFromDropoff >= 0);
        }

        const Vehicle *vehicle = nullptr;
        const PDLoc *pickup = nullptr;
        const PDLoc *dropoff = nullptr;

        int pickupStopIdx = INVALID_INDEX; // Pickup is inserted at or after stop with index pickupStopIdx in route of vehicle
        int dropoffStopIdx = INVALID_INDEX; // Dropoff is inserted at or after stop with index dropoffStopIdx in route of vehicle

        int distToPickup = 0; // Distance from previous stop to pickup
        int distFromPickup = 0; // Distance from pickup to next stop (or 0 if pickupStopIdx == dropoffStopIdx)
        int distToDropoff = 0; // Distance from previous stop to dropoff (or from pickup to dropoff if pickupStopIdx == dropoffStopIdx)
        int distFromDropoff = 0; // Distance from dropoff to next stop (or 0 if there is no next stop)

        // Returns true if the assignment is valid, i.e., vehicle, pickup, and dropoff are set.
        bool isValid() const {
            return vehicle && pickup && dropoff;
        }
    };

    // Criterion to make decision between two assignments with the same cost deterministic.
    // Returns true if asgn1 should be preferred over asgn2.
    static inline bool
    breakCostTie(const Assignment &asgn1, const Assignment &asgn2) {
        // If vehicle, pickup, or dropoff are not set, the assignment is invalid.
        if (!asgn1.vehicle || !asgn1.pickup || !asgn1.dropoff) return false;
        if (!asgn2.vehicle || !asgn2.pickup || !asgn2.dropoff) return true;

        if (asgn1.vehicle->vehicleId < asgn2.vehicle->vehicleId) return true;
        if (asgn1.vehicle->vehicleId > asgn2.vehicle->vehicleId) return false;
        if (asgn1.pickupStopIdx < asgn2.pickupStopIdx) return true;
        if (asgn1.pickupStopIdx > asgn2.pickupStopIdx) return false;
        if (asgn1.dropoffStopIdx < asgn2.dropoffStopIdx) return true;
        if (asgn1.dropoffStopIdx > asgn2.dropoffStopIdx) return false;
        if (asgn1.pickup->walkingDist < asgn2.pickup->walkingDist) return true;
        if (asgn1.pickup->walkingDist > asgn2.pickup->walkingDist) return false;
        if (asgn1.dropoff->walkingDist < asgn2.dropoff->walkingDist) return true;
        if (asgn1.dropoff->walkingDist > asgn2.dropoff->walkingDist) return false;
        if (asgn1.pickup->id < asgn2.pickup->id) return true;
        if (asgn1.pickup->id > asgn2.pickup->id) return false;
        if (asgn1.dropoff->id < asgn2.dropoff->id) return true;
        if (asgn1.dropoff->id > asgn2.dropoff->id) return false;
        if (asgn1.distToPickup < asgn2.distToPickup) return true;
        if (asgn1.distToPickup > asgn2.distToPickup) return false;
        if (asgn1.distToDropoff < asgn2.distToDropoff) return true;
        if (asgn1.distToDropoff > asgn2.distToDropoff) return false;
        if (asgn1.distFromPickup < asgn2.distFromPickup) return true;
        if (asgn1.distFromPickup > asgn2.distFromPickup) return false;
        return asgn1.distFromDropoff < asgn2.distFromDropoff;

    }
}