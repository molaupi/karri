/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "OrdinaryStopsAtLocations.h"

namespace karri {

    template<
            typename InputGraphT,
            typename LastStopsAtVerticesT>
    class PDLocsAtExistingStopsFinder {


        // Info about a PD loc that coincides with an existing stop of a vehicle.
        struct PDLocAtExistingStop {
            int pdId = INVALID_ID;
            int vehId = INVALID_ID;
            int stopIndex = INVALID_INDEX;
        };

    public:

        PDLocsAtExistingStopsFinder(const InputGraphT& inputGraph,
                                    const OrdinaryStopsAtLocations& ordinaryStopsAtLocations,
                                    const LastStopsAtVerticesT& lastStopsAtVertices,
                                    const RouteState& routeState) :
                                    inputGraph(inputGraph),
                                    ordinaryStopsAtLocations(ordinaryStopsAtLocations),
                                    lastStopsAtVertices(lastStopsAtVertices),
                                    routeState(routeState) {}

        template<PDLocType type, typename PDLocsT>
        std::vector<PDLocAtExistingStop>
        findPDLocsAtExistingStops(const PDLocsT &pdLocs,
                                  stats::EllipticBCHPerformanceStats& stats) const {
            Timer timer;
            std::vector<PDLocAtExistingStop> res;

            for (const auto &pdLoc: pdLocs) {
                for (const auto& stopId : ordinaryStopsAtLocations.getStopsAtLocation(pdLoc.loc)) {
                    const auto vehId = routeState.vehicleIdOf(stopId);
                    const auto stopIdx = routeState.stopPositionOf(stopId);
                    LIGHT_KASSERT(routeState.stopLocationsFor(vehId)[stopIdx] == pdLoc.loc);
                    res.push_back({pdLoc.id, vehId, stopIdx});
                }

                if constexpr (type == DROPOFF) {
                    // Additionally find dropoffs that coincide with the last stop:
                    const auto head = inputGraph.edgeHead(pdLoc.loc);
                    for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(head)) {
                        const auto numStops = routeState.numStopsOf(vehId);
                        if (numStops == 1)
                            continue; // If vehicle has only one stop, insertions with dropoff at last stop are not possible
                        if (routeState.stopLocationsFor(vehId)[numStops - 1] == pdLoc.loc) {
                            res.push_back({pdLoc.id, vehId, numStops - 1});
                        }
                    }
                }
            }

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationTime += time;

            return res;
        }

    private:

        const InputGraphT& inputGraph;
        const OrdinaryStopsAtLocations& ordinaryStopsAtLocations;
        const LastStopsAtVerticesT& lastStopsAtVertices;
        const RouteState& routeState;

    };

} // end namespace karri

