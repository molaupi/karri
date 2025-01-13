/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"

namespace karri {

    // A facility for computing the cost of an assignment of a request.
    template<typename CostFunctionT>
    class CostCalculator {


    public:

        // Calculates the objective value for a given assignment.
        template<bool checkHardConstraints, typename RequestContext>
        int calcBase(const Assignment &asgn, const RequestContext &context) const {
            using namespace time_utils;
            KASSERT(asgn.pickup && asgn.dropoff);
            if (!asgn.pickup || !asgn.dropoff)
                return INFTY;

            // Check hard constraints of vehicle on each leg.
            int riderArrTimeAtPickupOfLeg = context.getPassengerArrAtPickup(asgn.pickup->id);
            for (const auto &leg: asgn.legs) {
                if (leg.travelTimeToPickup == INFTY || leg.travelTimeFromPickup == INFTY ||
                    leg.travelTimeToDropoff == INFTY || leg.travelTimeFromDropoff == INFTY ||
                    leg.detourCostToPickup == INFTY || leg.detourCostFromPickup == INFTY ||
                    leg.detourCostToDropoff == INFTY || leg.detourCostFromDropoff == INFTY)
                    return INFTY;

                if constexpr (checkHardConstraints) {
                    const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(leg, riderArrTimeAtPickupOfLeg, context,
                                                                                routeState);
                    const auto initialPickupDetour = calcInitialPickupDetour(leg, actualDepTimeAtPickup, context,
                                                                             routeState);
                    const bool dropoffAtExistingStop = isDropoffAtExistingStop(leg, routeState);

                    if (isAnyHardConstraintViolated(leg, initialPickupDetour, dropoffAtExistingStop, context))
                        return INFTY;

                    riderArrTimeAtPickupOfLeg = getArrTimeAtDropoff(actualDepTimeAtPickup, leg, initialPickupDetour,
                                                                    dropoffAtExistingStop, routeState);
                }
            }


            // Compute vehicle cost for each leg:
            int totalVehCost = 0;
            for (const auto &leg: asgn.legs) {

                // TODO: Implement vehicle cost dependent on vehicle type

            }

            // Compute trip cost for new rider:

            // TODO

            // Compute added trip cost for existing riders on each leg:
            int existingRidersAddedTripCost = 0;
            // TODO
        }

        template<typename RequestContext>
        inline bool
        isAnyHardConstraintViolated(const Assignment::Leg &leg, const int initialPickupDetour,
                                    const bool dropoffAtExistingStop,
                                    const RequestContext &context) {
            using namespace time_utils;

            if (!leg.vehicle)
                return false;

            const auto vehId = leg.vehicle->vehicleId;
            const auto endServiceTime = leg.vehicle->endOfServiceTime;
            const auto numStops = routeState.numStopsOf(vehId);
            const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
            const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
            const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);
            const auto &occupancies = routeState.occupanciesFor(vehId);

            const auto initialDropoffDetour = calcInitialDropoffDetour(leg, dropoffAtExistingStop, routeState);
            const auto detourRightAfterDropoff = calcDetourRightAfterDropoff(leg, initialPickupDetour,
                                                                             initialDropoffDetour, routeState);
            const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(leg.vehicle->vehicleId,
                                                                                        leg.dropoffStopIdx,
                                                                                        numStops - 1,
                                                                                        detourRightAfterDropoff,
                                                                                        routeState);

            // If departure time at the last stop (which may be the time of issuing this request if the vehicle is currently
            // idling) is moved past the end of the service time by the total detour, the assignment violates the service
            // time constraint.
            if (std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
                endServiceTime)
                return true;

            // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
            // assignment is ok.
            if (leg.pickupStopIdx + 1 == numStops)
                return false;

            // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
            // arrival time, this assignment violates some trip time or wait time hard constraint.
            if (leg.pickupStopIdx != leg.dropoffStopIdx && initialPickupDetour != 0 &&
                minArrTimes[leg.pickupStopIdx + 1] + initialPickupDetour > maxArrTimes[leg.pickupStopIdx + 1])
                return true;

            // If somewhere between pickup and dropoff the vehicle is already full, we cannot insert another passenger.
            for (int i = leg.pickupStopIdx; i < leg.dropoffStopIdx; ++i)
                if (occupancies[i] + context.originalRequest.numRiders > leg.vehicle->capacity)
                    return true;
            if (!dropoffAtExistingStop &&
                occupancies[leg.dropoffStopIdx] + context.originalRequest.numRiders > leg.vehicle->capacity)
                return true;

            // If the dropoff is inserted at/after the last stop, the service time constraint is kept and the pickup does
            // not violate the trip time or wait time constraints, the assignment is ok.
            if (leg.dropoffStopIdx + 1 == numStops)
                return false;

            // If the total detour moves the planned arrival time at the stop after the dropoff past the latest permissible
            // arrival time, this assignment violates some trip time or wait time constraint.
            if (detourRightAfterDropoff != 0 &&
                minArrTimes[leg.dropoffStopIdx + 1] + detourRightAfterDropoff > maxArrTimes[leg.dropoffStopIdx + 1])
                return true;

            return false;
        }


    private:

        const RouteState &routeState;

    };

} // karri

