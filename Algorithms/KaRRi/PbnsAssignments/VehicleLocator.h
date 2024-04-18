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

#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleLocation.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Parallel/atomic_wrapper.h"
#include "DataStructures/Containers/Parallel/ThreadSafeSubset.h"
#include "Tools/Timer.h"

#include <tbb/enumerable_thread_specific.h>

namespace karri {

// Determines the current location of a vehicle at a given point in time by reconstructing the path from its previous
// stop to its next stop and traversing this path to find the road segment along which the vehicle is currently
// travelling.
    template<typename InputGraphT, typename CHEnvT>
    class VehicleLocator {

    public:

        static constexpr VehicleLocation INVALID_LOC = {INVALID_EDGE, -1};

        VehicleLocator(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteState &routeState,
                       const int fleetSize)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  chQuery([&]() { return chEnv.template getFullCHQuery<>(); }),
                  unpacker(ch),
                  routeState(routeState),
                  fleetSize(fleetSize),
                  path(),
                  currentTime(-1),
                  currentVehicleLocations(fleetSize, INVALID_LOC),
                  vehLocks(fleetSize, SpinLock()),
                  vehiclesWithKnownLocation(fleetSize),
                  totalLocatingVehiclesTimeForRequest(0) {}

        void locateVehicle(const Vehicle &vehicle) {
            const auto &vehId = vehicle.vehicleId;
            assert(vehId >= 0 && vehId < fleetSize);

            if (vehiclesWithKnownLocation.contains(vehId))
                return;

            SpinLock &curVehLock = vehLocks[vehId];
            curVehLock.lock();

            if (currentVehicleLocations[vehId] != INVALID_LOC) {
                curVehLock.unlock();
                return;
            }

            assert(currentVehicleLocations[vehId] == INVALID_LOC);

            Timer timer;
            const auto curLoc = computeCurrentLocation(vehicle, currentTime);
            totalLocatingVehiclesTimeForRequest.add_fetch(timer.elapsed<std::chrono::nanoseconds>(),
                                                          std::memory_order_relaxed);

            currentVehicleLocations[vehId] = curLoc;
            vehiclesWithKnownLocation.insert(vehId);
            curVehLock.unlock();

        }

        void init(const int time) {

            // Already initialized for current time
            if (currentTime == time)
                return;

            currentTime = time;
            for (const auto &vehId: vehiclesWithKnownLocation) {
                currentVehicleLocations[vehId] = INVALID_LOC;
            }
            vehiclesWithKnownLocation.clear();

            totalLocatingVehiclesTimeForRequest.store(0, std::memory_order_seq_cst);
        }

        const ThreadSafeSubset& getVehiclesWithKnownLocation() const {
            return vehiclesWithKnownLocation;
        }

        const VehicleLocation& getVehicleLocation(const int vehId) const {
            assert(vehiclesWithKnownLocation.contains(vehId));
            return currentVehicleLocations[vehId];
        }

        int64_t getTotalLocatingVehiclesTimeForRequest() const {
            return totalLocatingVehiclesTimeForRequest.load(std::memory_order_seq_cst);
        }

    private:

        VehicleLocation computeCurrentLocation(const Vehicle &veh, const int now) {
            const auto &vehId = veh.vehicleId;
            assert(routeState.numStopsOf(vehId) > 0);


            const auto prevOrCurLoc = routeState.stopLocationsFor(vehId)[0];
            const auto &schedDepTimes = routeState.schedDepTimesFor(vehId);
            const auto &schedArrTimes = routeState.schedArrTimesFor(vehId);

            // Vehicle before the start of its service time is at its initial location and can leave at the start of its
            // service time at the earliest
            if (veh.startOfServiceTime >= now) {
                return {prevOrCurLoc, veh.startOfServiceTime};
            }

            // If vehicle is idling, it is at its stop 0 and can leave now
            if (routeState.numStopsOf(vehId) == 1) {
                return {prevOrCurLoc, now};
            }

            // If vehicle is currently stopping, it can leave at its scheduled departure time.
            if (schedDepTimes[0] >= now) {
                return {prevOrCurLoc, schedDepTimes[0]};
            }

            const auto nextLoc = routeState.stopLocationsFor(vehId)[1];

            // Is vehicle already at stop 1?
            if (schedArrTimes[1] - inputGraph.travelTime(nextLoc) < now) {
                return {nextLoc, schedArrTimes[1]};
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
            FullQuery &localCHQuery = chQuery.local();
            localCHQuery.run(ch.rank(inputGraph.edgeHead(prevOrCurLoc)), ch.rank(inputGraph.edgeTail(nextLoc)));
            assert(schedDepTimes[0] + localCHQuery.getDistance() + inputGraph.travelTime(nextLoc) == schedArrTimes[1]);

            std::vector<int> &localPath = path.local();
            localPath.clear();
            unpacker.local().unpackUpDownPath(localCHQuery.getUpEdgePath(), localCHQuery.getDownEdgePath(), localPath);


            int depTimeAtCurEdge = schedDepTimes[0];
            for (const auto &curEdge: localPath) {
                depTimeAtCurEdge += inputGraph.travelTime(curEdge);
                if (depTimeAtCurEdge >= now) {
                    return {curEdge, depTimeAtCurEdge};
                }
            }

            assert(false);
            return {};
        }


    private:
        using FullQuery = typename CHEnvT::template FullCHQuery<>;
        const InputGraphT &inputGraph;
        const CH &ch;
        tbb::enumerable_thread_specific<FullQuery> chQuery;
        tbb::enumerable_thread_specific<CHPathUnpacker> unpacker;
        const RouteState &routeState;
        const int fleetSize;

        tbb::enumerable_thread_specific<std::vector<int>> path;

        int currentTime;

        std::vector<VehicleLocation> currentVehicleLocations;
        std::vector<SpinLock> vehLocks;
        ThreadSafeSubset vehiclesWithKnownLocation;


        CAtomic<int64_t> totalLocatingVehiclesTimeForRequest;

    };
}