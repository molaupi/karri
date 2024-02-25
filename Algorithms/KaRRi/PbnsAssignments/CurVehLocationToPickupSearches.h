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

#include <type_traits>
#include "Tools/Timer.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleLocation.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

#include "tbb/concurrent_vector.h"
#include "tbb/enumerable_thread_specific.h"
#include "Parallel/atomic_wrapper.h"
#include "DataStructures/Containers/ThreadSafeSubset.h"


namespace karri {


    template<typename InputGraphT, typename VehicleLocatorT, typename CHEnvT, typename LabelSetT>
    class CurVehLocationToPickupSearches {

    private:


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        static constexpr VehicleLocation INVALID_LOC = {INVALID_EDGE, -1};

        static constexpr int unknownDist = INFTY + 1;

        using FullQuery = typename CHEnvT::template FullCHQuery<>;


    public:

        CurVehLocationToPickupSearches(const InputGraphT &graph,
                                  VehicleLocatorT &locator,
                                  const CHEnvT &chEnv,
                                  const RouteState &routeState,
                                  RequestState &requestState,
                                  const int fleetSize)
                : inputGraph(graph),
                  vehicleLocator(locator),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  requestState(requestState),
                  fleetSize(fleetSize),
                  distances(),
                  currentVehicleLocations(fleetSize, INVALID_LOC),
                  prevNumPickups(0),
                  vehiclesWithKnownLocation(fleetSize),
                  vehLocks(fleetSize, SpinLock()),
                  currentTime(-1),
                  chQuery([&]() { return chEnv.template getFullCHQuery<>(); }),
                  totalLocatingVehiclesTimeForRequest(0),
                  totalVehicleToPickupSearchTimeForRequest(0),
                  totalNumCHSearchesRunForRequest(0) {}

        void initialize(const int now) {
            currentTime = now;

            clearDistances();
            totalLocatingVehiclesTimeForRequest.store(0);
            totalVehicleToPickupSearchTimeForRequest.store(0);
            totalNumCHSearchesRunForRequest.store(0);
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return distances[idx] != unknownDist;
        }

        int getDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return distances[idx];
        }

//        bool knowsCurrentLocationOf(const int vehId) const {
//            assert(vehId >= 0 && vehId < fleetSize);
//
//
//            return vehiclesWithKnownLocation.contains(vehId) && currentVehicleLocations[vehId] != INVALID_LOC;
//        }

        const VehicleLocation &getCurrentLocationOf(const int vehId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            return currentVehicleLocations[vehId];
        }

        // Computes the exact distances via a given vehicle to a specific pickup
        void computeExactDistancesVia(const Vehicle &vehicle, const unsigned int pickupId, const int distFromPrevStopToPickup) {

            assert(routeState.numStopsOf(vehicle.vehicleId) > 1);

           if (!vehiclesWithKnownLocation.contains(vehicle.vehicleId)) {
            locateVehicle(vehicle);
           }
            const auto &vehLocation = currentVehicleLocations[vehicle.vehicleId];
            assert(vehLocation != INVALID_LOC);

            if (vehLocation.location == routeState.stopLocationsFor(vehicle.vehicleId)[0]) {
                fillDistancesForVehicleAtPrevStop(vehicle, pickupId, distFromPrevStopToPickup);
                prevNumPickups = requestState.numPickups();
                return;
            }

            const auto distToCurLoc = vehLocation.depTimeAtHead - routeState.schedDepTimesFor(vehicle.vehicleId)[0];

            int numChSearchesRun = 0;
            Timer timer;

            if (!knowsDistance(vehicle.vehicleId, pickupId)) {
                const auto pickupLocation = requestState.pickups[pickupId].loc;
                const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                if (vehLocation.location == pickupLocation) {
                    distances[idx] = distToCurLoc;
                } else {
                    FullQuery &localCHQuery = chQuery.local();
                    localCHQuery.run(ch.rank(inputGraph.edgeHead(vehLocation.location)), ch.rank(inputGraph.edgeTail(pickupLocation)));
                    distances[idx] = distToCurLoc + localCHQuery.getDistance() + inputGraph.travelTime(pickupLocation);
                    ++numChSearchesRun;
                }
            }

            prevNumPickups = requestState.numPickups();

            totalVehicleToPickupSearchTimeForRequest.add_fetch(timer.elapsed<std::chrono::nanoseconds>(), std::memory_order_relaxed);
            totalNumCHSearchesRunForRequest.add_fetch(numChSearchesRun, std::memory_order_relaxed);
        }


        int64_t getTotalLocatingVehiclesTimeForRequest() const {
            return totalLocatingVehiclesTimeForRequest;
        }

        int64_t getTotalVehicleToPickupSearchTimeForRequest() const {
            return totalVehicleToPickupSearchTimeForRequest;
        }

        int64_t getTotalNumCHSearchesRunForRequest() const {
            return totalNumCHSearchesRunForRequest;
        }

    private:

        void clearDistances() {

            // Clear the distances for every vehicle for which we computed the current location:
            for (const auto &vehId: vehiclesWithKnownLocation) {
                const int start = vehId * prevNumPickups;
                const int end = start + prevNumPickups;
                std::fill(distances.begin() + start, distances.begin() + end, unknownDist);
                currentVehicleLocations[vehId] = INVALID_LOC;
            }
            assert(std::all_of(distances.begin(), distances.end(), [&](const auto &d) { return d == unknownDist; }));
            assert(std::all_of(currentVehicleLocations.begin(), currentVehicleLocations.end(),
                               [&](const auto &l) { return l == INVALID_LOC; }));
            vehiclesWithKnownLocation.clear();

            const int numDistances = requestState.numPickups() * fleetSize;
            if (numDistances > distances.size()) {
                const int diff = numDistances - distances.size();
                distances.insert(distances.end(), diff, unknownDist);
            }
        }

        void locateVehicle(const Vehicle &vehicle) {
            const auto &vehId = vehicle.vehicleId;

            assert(vehId >= 0 && vehId < fleetSize);

            SpinLock &curVehLock = vehLocks[vehId];
            curVehLock.lock();
            
            if (currentVehicleLocations[vehId] != INVALID_LOC) {
                curVehLock.unlock();
                return;
            }

            assert(currentVehicleLocations[vehId] == INVALID_LOC);

            Timer timer;
            const auto curLoc = vehicleLocator.computeCurrentLocation(vehicle, currentTime);
            totalLocatingVehiclesTimeForRequest.add_fetch(timer.elapsed<std::chrono::nanoseconds>(), std::memory_order_relaxed);

            currentVehicleLocations[vehId] = curLoc;
            vehiclesWithKnownLocation.insert(vehId);

            curVehLock.unlock();
        }

        void fillDistancesForVehicleAtPrevStop(const Vehicle &vehicle, const unsigned int pickupId, const int distFromPrevStopToPickup) {
            const auto &stopLocations = routeState.stopLocationsFor(vehicle.vehicleId);
            if (stopLocations[0] != requestState.pickups[pickupId].loc) {
                const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                distances[idx] = distFromPrevStopToPickup;
            } else {
                const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                distances[idx] = 0;
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        const CH &ch;
        const RouteState &routeState;
        RequestState &requestState;
        const int fleetSize;


        std::vector<int> distances;
        std::vector<VehicleLocation> currentVehicleLocations;
        int prevNumPickups;
        ThreadSafeSubset vehiclesWithKnownLocation;

        std::vector<SpinLock> vehLocks; 
        
        int currentTime;

        tbb::enumerable_thread_specific<FullQuery> chQuery;

        CAtomic<int64_t> totalLocatingVehiclesTimeForRequest;
        CAtomic<int64_t> totalVehicleToPickupSearchTimeForRequest;
        CAtomic<int64_t> totalNumCHSearchesRunForRequest;


    };

}