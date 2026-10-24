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
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RiderModeChoice/TransportMode.h"
#include "Tools/Constants.h"
#include "Tools/Workarounds.h"
#include "DataStructures/Queues/AddressableFIFOQueue.h"
#include "DataStructures/Utilities/RunningQuantile.h"

namespace karri::RepositioningStrategies {

    // Strategy for randomly choosing which idle vehicle should reposition and where.
    // Repositioning targets are chosen from previously seen request origins,
    // weighted by frequency.
    class RandomRepositioningStrategy {

    public:
        static constexpr bool USE_DETERMINISTIC = true;

        // Seen origin locations time out as valid repositioning targets after SEEN_ORIGIN_TIMEOUT time steps.
        static constexpr int SEEN_ORIGIN_TIMEOUT = KARRI_REPOSITIONING_SEEN_ORIGIN_TIMEOUT_SECONDS * 10; // time steps are tenths of seconds

        explicit RandomRepositioningStrategy(const Fleet &)
            : seenOriginLocations(),
              gen(USE_DETERMINISTIC ? 0 : std::random_device{}()) {}

        // Notify the strategy about a request that has been processed by the dispatcher and the chosen mode.
        // Returns true if repositioning should be started now or false otherwise.
        bool notifyRequestProcessed(const Request &request, const mode_choice::TransportMode mode, const int directOdDist, const int tripTime) {
            unused(mode);

            // Ignore trips with invalid trip time
            if (tripTime >= INFTY)
                return false;

            // Short distance requests do not partake
            static constexpr int IGNORE_SHORT_DISTANCE_REQUESTS_THRESHOLD = 6000; // 10 min
            if (directOdDist <= IGNORE_SHORT_DISTANCE_REQUESTS_THRESHOLD)
                return false;

            const auto score = static_cast<double>(tripTime) / directOdDist;

            // Add delay of request to running quantile
            runningScoreQuantile.addValue(score);

            // If the score of this request is in the 90th percentile of requests processed so far, we consider it a
            // bad assignment and we allow repositioning to the requests origin location to improve the assignment for
            // future requests in the vicinity.
            if (score < runningScoreQuantile.getQuantile())
                return false;
            seenOriginLocations.emplace_back(request.origin, request.requestTime);

            // Start repositioning if this was a bad assignment.
            return true;
        }

        // Notify the strategy that a vehicle has become idle. Unused since idle vehicles are read directly
        // from the RouteState in pickRepositioningVehicleAndTarget().
        void notifyBecameIdle(const int) {}

        // Notify the strategy that a vehicle is no longer idle. Unused since idle vehicles are read directly
        // from the RouteState in pickRepositioningVehicleAndTarget().
        void notifyBecameNonIdle(const int) {}

        // Pick an idle vehicle and a repositioning target location.
        // Returns a pair of (vehicle ID, target location).
        // Returns (INVALID_ID, INVALID_EDGE) if no valid choice can be made.
        std::pair<int, int> pickRepositioningVehicleAndTarget(const RouteState &routeState, const int now) {

            // Remove timed out origin locations
            int i = 0;
            while (i < seenOriginLocations.size() && seenOriginLocations[i].second + SEEN_ORIGIN_TIMEOUT <= now)
                ++i;
            seenOriginLocations.erase(seenOriginLocations.begin(), seenOriginLocations.begin() + i);

            // Do not perform repositioning if there are no idle vehicles or no relevant target locations
            const auto& idle = routeState.getIdleVehicles();
            if (idle.size() == 0 || seenOriginLocations.empty()) {
                return {INVALID_ID, INVALID_EDGE};
            }

            // Choose a random idle vehicle
            std::uniform_int_distribution<size_t> vehDis(0, idle.size() - 1);
            const int vehId = *(idle.begin() + vehDis(gen));

            // Choose a repositioning target from previously seen origins
            std::uniform_int_distribution<size_t> targetLocDis(0, seenOriginLocations.size() - 1);
            const int target = seenOriginLocations[targetLocDis(gen)].first;

            return {vehId, target};
        }

    private:
        RunningQuantile<0.9, double> runningScoreQuantile;

        // Track seen origin locations and their request time for target selection
        std::vector<std::pair<int, int>> seenOriginLocations;

        // Random number generator
        mutable std::mt19937 gen;
    };

}
