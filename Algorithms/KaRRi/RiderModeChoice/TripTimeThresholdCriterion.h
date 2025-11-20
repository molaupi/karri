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

#include "TransportMode.h"
#include "ModeChoiceInput.h"

namespace karri::mode_choice {

    // Models mode choice of a rider given a possible taxi-sharing assignment.
    // Simple implementation based on trip times: Takes smaller trip time between walking and taxi sharing. If this
    // trip time is larger than a threshold (relative to direct car distance), a direct car trip is chosen instead.
    // This criterion will never choose public transport.
    class TripTimeThresholdCriterion {

    public:

        TripTimeThresholdCriterion() = default;

        TransportMode apply(ModeChoiceInput input) {
            using namespace time_utils;

            const auto taxiTripTime = input.taxiTravelTime + input.taxiWaitTime + input.taxiAccEgrTime;
            const auto pedTaxiMinTime = std::min(taxiTripTime, input.walkTravelTime);
            const auto threshold = InputConfig::getInstance().epsilon * input.privateCarTravelTime + InputConfig::getInstance().phi;


            // epsilon = 0 is a special value, indicating that there is no threshold, i.e. rider should only walk or take taxi.
            if (InputConfig::getInstance().epsilon == 0.0 || pedTaxiMinTime <= threshold) {
                if (taxiTripTime < input.walkTravelTime)
                    return TransportMode::Taxi;
                return TransportMode::Ped;
            }

            return TransportMode::Car;
        }

    private:

    };

} // karri::rider_acceptance

