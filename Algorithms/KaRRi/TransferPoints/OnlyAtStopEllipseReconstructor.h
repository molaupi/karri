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

#include <vector>
#include "EdgeEllipseContainer.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Tools/Timer.h"

namespace karri {

    // Provides same interface as full EllipseReconstructor but only returns locations of stops themselves as
    // potential transfer points. This is useful for heuristics that limit transfers to existing stops.
    class OnlyAtStopEllipseReconstructor {

    public:
        OnlyAtStopEllipseReconstructor(const RouteState& routeState) : routeState(routeState) {}

        EdgeEllipseContainer computeEllipses(const std::vector<int> &stopIds, stats::EllipseReconstructionStats& stats) {
            Timer timer;

            EdgeEllipseContainer container;
            container.idxOfStop.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);

            if (stopIds.empty())
                return container;

            int nextUniqueStop = 0;
            for (const auto &stopId: stopIds) {
                if (container.idxOfStop[stopId] != INVALID_INDEX)
                    continue;

                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopPos = routeState.stopPositionOf(stopId);
                const auto numStops = routeState.numStopsOf(vehId);

                // Add stop to container
                container.idxOfStop[stopId] = nextUniqueStop++;
                auto& ellipse = container.edgeEllipses.emplace_back();
                const auto stopLocations = routeState.stopLocationsFor(vehId);
                const auto lengthOfLegToNext = stopPos == numStops - 1? 0 : time_utils::calcLengthOfLegStartingAt(stopPos, vehId, routeState);
                ellipse.emplace_back(stopLocations[stopPos], 0, lengthOfLegToNext);
            }


            stats.withoutLeewaySearchTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.withoutLeewayNumEllipses += stopIds.size();
            stats.withoutLeewaySumSizesEllipses += nextUniqueStop;

            return container;
        }

    private:
        const RouteState &routeState;
    };

} // karri

