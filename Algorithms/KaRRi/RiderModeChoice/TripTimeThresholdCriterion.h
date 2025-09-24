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

#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "TransportMode.h"

namespace karri::mode_choice {

    // Models mode choice of a rider given a possible taxi-sharing assignment.
    // Simple implementation based on trip times: Takes smaller trip time between walking and taxi sharing. If this
    // trip time is larger than a threshold (relative to direct car distance), a direct car trip is chosen instead.
    class TripTimeThresholdCriterion {

    public:

        TripTimeThresholdCriterion(const RouteState& routeState) : routeState(routeState) {}

        template<typename AsgnFinderResponseT>
        TransportMode chooseMode(const Request &req, const AsgnFinderResponseT &resp) const {
            using namespace time_utils;

            const auto &bestAsgn = resp.getBestAssignment();
            int taxiTripTime = INFTY;
            if (bestAsgn.vehicle) {
                const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, resp, routeState);
                const auto initialPickupDetour = calcInitialPickupDetour(bestAsgn, depTimeAtPickup, resp, routeState);
                const auto dropoffAtExistingStop = isDropoffAtExistingStop(bestAsgn, routeState);
                const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, bestAsgn, initialPickupDetour,
                                                                  dropoffAtExistingStop, routeState);
                taxiTripTime = arrTimeAtDropoff + bestAsgn.dropoff.walkingDist - req.requestTime;
            }

            const auto walkTripTime = resp.odWalkingDist;
            KASSERT(walkTripTime < INFTY);

            const auto pedTaxiMinTime = std::min(taxiTripTime, walkTripTime);
            const auto directCarTime = resp.originalReqDirectDist;
            const auto threshold = InputConfig::getInstance().epsilon * directCarTime + InputConfig::getInstance().phi;


            // epsilon = 0 is a special value, indicating that there is no threshold, i.e. rider should only walk or take taxi.
            if (InputConfig::getInstance().epsilon == 0.0 || pedTaxiMinTime <= threshold) {
                if (taxiTripTime < walkTripTime)
                    return TransportMode::Taxi;
                return TransportMode::Ped;
            }

            return TransportMode::Car;
        }

    private:

        const RouteState &routeState;

    };

} // karri::rider_acceptance

