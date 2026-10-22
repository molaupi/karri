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

#include <algorithm>
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleLocation.h"
#include "Algorithms/KaRRi/RouteState.h"

namespace karri {

// Determines the current location of a vehicle at a given point in time by reconstructing the path from its previous
// stop to its next stop and traversing this path to find the road segment along which the vehicle is currently
// travelling.
    template<typename InputGraphT, typename CHEnvT>
    class VehicleLocator {

    public:
        VehicleLocator(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteState &routeState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  chQuery(chEnv.template getFullCHQuery<>()),
                  unpacker(ch),
                  routeState(routeState),
                  path() {}


        VehicleLocation computeCurrentLocation(const Vehicle &veh, const int now) {
            const auto &vehId = veh.vehicleId;
            assert(routeState.numStopsOf(vehId) > 0);


            const auto prevOrCurLoc = routeState.stopLocationsFor(vehId)[0];
            const auto schedDepTimes = routeState.schedDepTimesFor(vehId);
            const auto schedArrTimes = routeState.schedArrTimesFor(vehId);

            // If the vehicle is technically idle but currently repositioning, its next location is the repositioning
            // target instead of the next stop.
            const bool isRepositioning = routeState.isRepositioning(vehId);

            // Vehicle before the start of its service time is at its initial location and can leave at the start of its
            // service time at the earliest
            if (veh.startOfServiceTime >= now) {
                return {prevOrCurLoc, veh.startOfServiceTime};
            }

            // If vehicle is idling, it is at its stop 0 and can leave now
            if (routeState.numStopsOf(vehId) == 1 && !isRepositioning) {
                return {prevOrCurLoc, now};
            }

            // If vehicle is currently stopping, it can leave at its scheduled departure time.
            if (schedDepTimes[0] >= now) {
                return {prevOrCurLoc, schedDepTimes[0]};
            }

            // For a repositioning vehicle, find the current edge on the stored repositioning path.
            if (isRepositioning) {
                const auto &reposPath = routeState.getRepositioningPath(vehId);
                const auto &reposPathTimes = routeState.getRepositioningPathTimes(vehId);
                const auto it = std::lower_bound(reposPathTimes.begin(), reposPathTimes.end(), now);
                if (it == reposPathTimes.end())
                    return {reposPath.back(), reposPathTimes.back()};
                const auto idx = it - reposPathTimes.begin();
                return {reposPath[idx], reposPathTimes[idx]};
            }

            const auto nextLoc = routeState.stopLocationsFor(vehId)[1];
            const int arrTimeAtNext = schedArrTimes[1];

            // Is vehicle already at next location?
            if (arrTimeAtNext - inputGraph.travelTime(nextLoc) < now) {
                return {nextLoc, arrTimeAtNext};
            }

            // Reconstruct path that vehicle is taking:
            // Attention: Depending on whether CHs or CCHs are used, the CH query can deliver different shortest paths.
            // This can lead to different current locations of the vehicle which can lead to different distances to a
            // pickup.
            // Usually, one would expect this difference to be small but in very bad edge cases, a vehicle may have
            // already entered a tunnel or stretch of highway without exits on one shortest path where it hasn't on
            // the other shortest path leading to the first vehicle location potentially making a much larger detour
            // to the pickup.
            // (This sounds like a pathologically rare case, but it actually happens on the Berlin-1pct input.)
            chQuery.run(ch.rank(inputGraph.edgeHead(prevOrCurLoc)), ch.rank(inputGraph.edgeTail(nextLoc)));
            KASSERT(schedDepTimes[0] + chQuery.getDistance() + inputGraph.travelTime(nextLoc) == arrTimeAtNext);

            path.clear();
            unpacker.unpackUpDownPath(chQuery.getUpEdgePath(), chQuery.getDownEdgePath(), path);


            int depTimeAtCurEdge = schedDepTimes[0];
            for (const auto &curEdge: path) {
                depTimeAtCurEdge += inputGraph.travelTime(curEdge);
                if (depTimeAtCurEdge >= now) {
                    return {curEdge, depTimeAtCurEdge};
                }
            }

            assert(false);
            return {};
        }


    private:

        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        CHPathUnpacker unpacker;
        const RouteState &routeState;

        std::vector<int> path;


    };
}