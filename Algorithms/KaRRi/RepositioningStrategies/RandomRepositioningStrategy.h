/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <random>
#include <vector>
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Tools/Constants.h"


namespace karri::RepositioningStrategies {

    // Strategy for randomly choosing which idle vehicle should reposition and where.
    // Repositioning targets are chosen from previously seen request origins,
    // weighted by frequency.
    class RandomRepositioningStrategy {

    public:
        static constexpr bool USE_DETERMINISTIC = true;

        explicit RandomRepositioningStrategy(const Fleet &)
            : seenOriginLocations(),
              gen(USE_DETERMINISTIC ? 0 : std::random_device{}()) {}

        // Notify the strategy about an incoming request.
        // Tracks the origin location for future repositioning target selection.
        void notifyRequestIncoming(const Request &request) {
            seenOriginLocations.push_back(request.origin);
        }

        // Pick an idle vehicle and a repositioning target location.
        // Returns a pair of (vehicle ID, target location).
        // Returns (INVALID_ID, INVALID_EDGE) if no valid choice can be made.
        std::pair<int, int> pickRepositioningVehicleAndTarget(const RouteState &routeState) const {
            // Choose a random idle vehicle
            const auto& idle = routeState.getIdleVehicles();
            if (idle.size() == 0 || seenOriginLocations.empty()) {
                return {INVALID_ID, INVALID_EDGE};
            }

            std::uniform_int_distribution<size_t> vehDis(0, idle.size() - 1);
            const int vehId = *(idle.begin() + vehDis(gen));

            // Choose a repositioning target from previously seen origins
            std::uniform_int_distribution<size_t> targetLocDis(0, seenOriginLocations.size() - 1);
            const int target = seenOriginLocations[targetLocDis(gen)];

            return {vehId, target};
        }

    private:
        // Track seen origin locations of requests for target selection
        std::vector<int> seenOriginLocations;

        // Random number generator
        mutable std::mt19937 gen;
    };

}

