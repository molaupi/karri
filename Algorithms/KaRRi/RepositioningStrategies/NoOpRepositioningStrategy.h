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

    // Strategy that never triggers repositioning and when prompted returns invalid vehicle and target.
    class NoOpRepositioningStrategy {

    public:

        NoOpRepositioningStrategy() = default;

        // Notify the strategy about a request that has been processed by the dispatcher and the chosen mode.
        // Returns true if repositioning should be started now or false otherwise.
        bool notifyRequestProcessed(const Request &, const mode_choice::TransportMode, const int, const int) {
            return false;
        }

        // Notify the strategy that a vehicle has become idle. Appends it to the back of the idle queue.
        void notifyBecameIdle(const int) {}

        // Notify the strategy that a vehicle is no longer idle. Removes it from the idle queue, wherever it
        // currently is.
        void notifyBecameNonIdle(const int ) {}

        // Pick the idle vehicle that has been idle the longest and a repositioning target location.
        // Returns a pair of (vehicle ID, target location).
        // Returns (INVALID_ID, INVALID_EDGE) if no valid choice can be made.
        std::pair<int, int> pickRepositioningVehicleAndTarget(const RouteState &, const int) {
            return {INVALID_ID, INVALID_EDGE};
        }
    };

}
