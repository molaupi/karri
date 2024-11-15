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

#include <kassert/kassert.hpp>

#include "Vehicle.h"
#include "Request.h"

namespace karri {

    // Models the assignment of a request to the route of a vehicle, specifying the vehicle, the pickup and dropoff
    // locations, the stop indices after which to insert the pickup and dropoff, as well as the distances from the
    // previous stop to either PD-location and from either PD-location to the next stop.
    struct Assignment {

        // Models a single leg of a rider's trip with a vehicle of a specified type, the locations where the vehicle
        // picks up and drops off the rider, as well as information on travel times and vehicle detour costs.
        struct Leg {

            // Construct potentially partial (invalid) leg with distances set to 0.
            explicit Leg(Vehicle const *vehicle = nullptr,
                         int pickupLoc = INVALID_INDEX,
                         int dropoffLoc = INVALID_INDEX,
                         int pickupStopIdx = 0,
                         int dropoffStopIdx = 0) :
                    vehicle(vehicle),
                    pickupLoc(pickupLoc),
                    dropoffLoc(dropoffLoc),
                    pickupStopIdx(pickupStopIdx),
                    dropoffStopIdx(dropoffStopIdx) {}


            Leg(Vehicle const *vehicle,
                int pickupLoc,
                int dropoffLoc,
                const int pickupStopIdx, const int dropoffStopIdx,
                const int travelTimeToPickup, const int travelTimeFromPickup,
                const int travelTimeToDropoff, const int travelTimeFromDropoff,
                const int detourCostToPickup, const int detourCostFromPickup,
                const int detourCostToDropoff, const int detourCostFromDropoff) :
                    vehicle(vehicle),
                    pickupLoc(pickupLoc),
                    dropoffLoc(dropoffLoc),
                    pickupStopIdx(pickupStopIdx),
                    dropoffStopIdx(dropoffStopIdx),
                    travelTimeToPickup(travelTimeToPickup),
                    travelTimeFromPickup(travelTimeFromPickup),
                    travelTimeToDropoff(travelTimeToDropoff),
                    travelTimeFromDropoff(travelTimeFromDropoff),
                    detourCostToPickup(detourCostToPickup),
                    detourCostFromPickup(detourCostFromPickup),
                    detourCostToDropoff(detourCostToDropoff),
                    detourCostFromDropoff(detourCostFromDropoff) {
                KASSERT(pickupStopIdx >= 0);
                KASSERT(dropoffStopIdx >= 0);
                KASSERT(travelTimeToPickup >= 0);
                KASSERT(travelTimeFromPickup >= 0);
                KASSERT(travelTimeToDropoff >= 0);
                KASSERT(travelTimeFromDropoff >= 0);
                KASSERT(detourCostToPickup >= 0);
                KASSERT(detourCostFromPickup >= 0);
                KASSERT(detourCostToDropoff >= 0);
                KASSERT(detourCostFromDropoff >= 0);
            }

            Vehicle const *vehicle = nullptr;
            int pickupLoc = INVALID_EDGE;
            int dropoffLoc = INVALID_EDGE;

            int pickupStopIdx = INVALID_INDEX; // Pickup is inserted at or after stop with index pickupStopIdx in route of vehicle
            int dropoffStopIdx = INVALID_INDEX; // Dropoff is inserted at or after stop with index dropoffStopIdx in route of vehicle

            // Travel times used for rider trip times and related constraints
            int travelTimeToPickup = 0; // Free-flow travel time from previous stop to pickup
            int travelTimeFromPickup = 0; // Free-flow travel time from pickup to next stop (or 0 if pickupStopIdx == dropoffStopIdx)
            int travelTimeToDropoff = 0; // Free-flow travel time from previous stop to dropoff (or from pickup to dropoff if pickupStopIdx == dropoffStopIdx)
            int travelTimeFromDropoff = 0; // Free-flow travel time from dropoff to next stop (or 0 if there is no next stop)

            // Traversal costs for detours of vehicles used for vehicle related costs
            int detourCostToPickup = 0; // Detour cost from previous stop to pickup
            int detourCostFromPickup = 0; // Detour cost from pickup to next stop (or 0 if pickupStopIdx == dropoffStopIdx)
            int detourCostToDropoff = 0; // Detour cost from previous stop to dropoff (or from pickup to dropoff if pickupStopIdx == dropoffStopIdx)
            int detourCostFromDropoff = 0; // Detour cost from dropoff to next stop (or 0 if there is no next stop)

            // Leg is considered valid if vehicle, pickup location, dropoff location, pickup stop index, and dropoff
            // stop idx are set.
            bool valid() const {
                return vehicle && pickupLoc != INVALID_EDGE && dropoffLoc != INVALID_EDGE &&
                       pickupStopIdx != INVALID_INDEX && dropoffStopIdx != INVALID_INDEX;
            }

            // Arbitrary order of leg objects used for deterministic tie breaking between assignments with same cost.
            // Returns true if leg1 is considered smaller than leg2.
            static inline bool
            arbitraryCmp(const Leg &leg1, const Leg &leg2) {
                if (!leg2.valid()) return true;
                if (!leg1.valid()) return false;

                if (leg1.vehicle->vehicleId < leg2.vehicle->vehicleId) return true;
                if (leg2.vehicle->vehicleId < leg1.vehicle->vehicleId) return false;

                if (leg1.pickupLoc < leg2.pickupLoc) return true;
                if (leg2.pickupLoc < leg1.pickupLoc) return false;

                if (leg1.dropoffLoc < leg2.dropoffLoc) return true;
                if (leg2.dropoffLoc < leg1.dropoffLoc) return false;

                if (leg1.pickupStopIdx < leg2.pickupStopIdx) return true;
                if (leg2.pickupStopIdx < leg1.pickupStopIdx) return false;

                if (leg1.dropoffStopIdx < leg2.dropoffStopIdx) return true;
                if (leg2.dropoffStopIdx < leg1.dropoffStopIdx) return false;

                if (leg1.travelTimeToPickup < leg2.travelTimeToPickup) return true;
                if (leg2.travelTimeToPickup < leg1.travelTimeToPickup) return false;

                if (leg1.travelTimeFromPickup < leg2.travelTimeFromPickup) return true;
                if (leg2.travelTimeFromPickup < leg1.travelTimeFromPickup) return false;

                if (leg1.travelTimeToDropoff < leg2.travelTimeToDropoff) return true;
                if (leg2.travelTimeToDropoff < leg1.travelTimeToDropoff) return false;

                if (leg1.travelTimeFromDropoff < leg2.travelTimeFromDropoff) return true;
                if (leg2.travelTimeFromDropoff < leg1.travelTimeFromDropoff) return false;

                if (leg1.detourCostToPickup < leg2.detourCostToPickup) return true;
                if (leg2.detourCostToPickup < leg1.detourCostToPickup) return false;

                if (leg1.detourCostFromPickup < leg2.detourCostFromPickup) return true;
                if (leg2.detourCostFromPickup < leg1.detourCostFromPickup) return false;

                if (leg1.detourCostToDropoff < leg2.detourCostToDropoff) return true;
                if (leg2.detourCostToDropoff < leg1.detourCostToDropoff) return false;

                if (leg1.detourCostFromDropoff < leg2.detourCostFromDropoff) return true;
                if (leg2.detourCostFromDropoff < leg1.detourCostFromDropoff) return false;

                return true;
            }
        };

        // Create empty assignment.
        explicit Assignment() : pickup(nullptr), dropoff(nullptr), legs() {}

        // Create assignment with given pickup, dropoff, and legs.
        Assignment(PDLoc const *pickup,
                   PDLoc const *dropoff,
                   std::vector<Leg> &&legs = {})
                : pickup(pickup),
                  dropoff(dropoff),
                  legs(legs) {
            KASSERT(pickup);
            KASSERT(dropoff);
        }

        // Create assignment with given pickup, dropoff, and legs.
        Assignment(PDLoc const *pickup,
                   PDLoc const *dropoff,
                   const std::vector<Leg> &legs)
                : pickup(pickup),
                  dropoff(dropoff),
                  legs(legs) {
            KASSERT(pickup);
            KASSERT(dropoff);
        }

        // Assignment is considered valid if there is at least one leg, all legs are valid, and pickup/dropoff
        // locations line up.
        bool valid() const {
            if (!pickup || !dropoff) return false;
            if (legs.empty()) return false;

            int lastTransfer = pickup->loc;
            for (const auto& l : legs) {
                if (!l.valid() || l.pickupLoc != lastTransfer) return false;
                lastTransfer = l.dropoffLoc;
            }
            return lastTransfer == dropoff->loc;
        }

        PDLoc const *pickup = nullptr; // Pickup location that rider walks to at beginning of trip
        PDLoc const *dropoff = nullptr; // Dropoff location that rider walks from at end of trip

        std::vector<Leg> legs;
    };

    // Criterion to make decision between two assignments with the same cost deterministic.
    // Returns true if asgn1 should be preferred over asgn2.
    static inline bool
    breakCostTie(const Assignment &asgn1, const Assignment &asgn2) {
        // Assignments with no legs are considered invalid:
        if (asgn2.legs.empty()) return true;
        if (asgn1.legs.empty()) return false;

        // Assignments with invalid legs are considered invalid.
        for (const auto &l: asgn2.legs)
            if (!l.valid()) return true;
        for (const auto &l : asgn1.legs)
            if (!l.valid()) return false;

        // Prefer assignment with smaller pickup walking distance
        if (asgn1.pickup->walkingDist < asgn2.pickup->walkingDist) return true;
        if (asgn1.pickup->walkingDist > asgn2.pickup->walkingDist) return false;

        // Prefer assignment with smaller dropoff walking distance
        if (asgn1.dropoff->walkingDist < asgn2.dropoff->walkingDist) return true;
        if (asgn1.dropoff->walkingDist > asgn2.dropoff->walkingDist) return false;

        // Prefer assignment with fewer legs
        if (asgn1.legs.size() < asgn2.legs.size()) return true;
        if (asgn2.legs.size() < asgn1.legs.size()) return false;

        // The following are arbitrary decision criteria:

        // Prefer assignment with smaller pickup ID
        if (asgn1.pickup->id < asgn2.pickup->id) return true;
        if (asgn1.pickup->id > asgn2.pickup->id) return false;

        // Prefer assignment with smaller dropoff ID
        if (asgn1.dropoff->id < asgn2.dropoff->id) return true;
        if (asgn1.dropoff->id > asgn2.dropoff->id) return false;


        // Compare legs pairwise until one leg wins in arbitrary comparison
        for (int i = 0; i < asgn1.legs.size(); ++i) {
            if (Assignment::Leg::arbitraryCmp(asgn1.legs[i], asgn2.legs[i])) return true;
            if (Assignment::Leg::arbitraryCmp(asgn2.legs[i], asgn1.legs[i])) return false;
        }

        // Assignments are identical.
        return true;
    }
}