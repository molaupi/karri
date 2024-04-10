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
#include "DataStructures/Containers/Parallel/ThreadSafeSubset.h"


namespace karri {


    template<typename InputGraphT, typename VehicleLocatorT, typename CHEnvT>
    class OnTheFlyVehLocToPickupSearches {

    private:
        using FullQuery = typename CHEnvT::template FullCHQuery<>;


    public:

        OnTheFlyVehLocToPickupSearches(const InputGraphT &graph,
                                       const CHEnvT &chEnv,
                                       const RouteState &routeState,
                                       RequestState &requestState,
                                       VehicleLocatorT &locator,
                                       VehLocToPickupDistances &distances)
                : inputGraph(graph),
                  vehicleLocator(locator),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  requestState(requestState),
                  distances(distances),
                  chQuery([&]() { return chEnv.template getFullCHQuery<>(); }),
                  totalVehicleToPickupSearchTimeForRequest(0),
                  totalNumCHSearchesRunForRequest(0) {}

        void initialize() {
            distances.init();
            vehicleLocator.init(requestState.originalRequest.requestTime);

            totalVehicleToPickupSearchTimeForRequest.store(0, std::memory_order_seq_cst);
            totalNumCHSearchesRunForRequest.store(0, std::memory_order_seq_cst);
        }

        // Computes the exact distances via a given vehicle to a specific pickup
        void computeExactDistancesVia(const Vehicle &vehicle, const unsigned int pickupId,
                                      const int distFromPrevStopToPickup) {

            assert(routeState.numStopsOf(vehicle.vehicleId) > 1);

            // Make sure vehicle location is known
            vehicleLocator.locateVehicle(vehicle);
            const auto &vehLocation = vehicleLocator.getVehicleLocation(vehicle.vehicleId);

            if (vehLocation.location == routeState.stopLocationsFor(vehicle.vehicleId)[0]) {
                fillDistancesForVehicleAtPrevStop(vehicle, pickupId, distFromPrevStopToPickup);
                return;
            }

            const auto distToCurLoc = vehLocation.depTimeAtHead - routeState.schedDepTimesFor(vehicle.vehicleId)[0];

            int numChSearchesRun = 0;
            Timer timer;

            if (!distances.knowsDistance(vehicle.vehicleId, pickupId)) {
                const auto pickupLocation = requestState.pickups[pickupId].loc;
                if (vehLocation.location == pickupLocation) {
                    distances.updateDistance(vehicle.vehicleId, pickupId, distToCurLoc);
                } else {
                    FullQuery &localCHQuery = chQuery.local();
                    localCHQuery.run(ch.rank(inputGraph.edgeHead(vehLocation.location)),
                                     ch.rank(inputGraph.edgeTail(pickupLocation)));
                    const int newDist =
                            distToCurLoc + localCHQuery.getDistance() + inputGraph.travelTime(pickupLocation);
                    assert(!distances.knowsDistance(vehicle.vehicleId, pickupId) ||
                           distances.getDistance(vehicle.vehicleId, pickupId) == newDist);
                    distances.updateDistance(vehicle.vehicleId, pickupId, newDist);
                    ++numChSearchesRun;
                }
            }

            totalVehicleToPickupSearchTimeForRequest.add_fetch(timer.elapsed<std::chrono::nanoseconds>(),
                                                               std::memory_order_relaxed);
            totalNumCHSearchesRunForRequest.add_fetch(numChSearchesRun, std::memory_order_relaxed);
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) {
            return distances.knowsDistance(vehId, pickupId);
        }

        int getDistance(const int vehId, const unsigned int pickupId) {
            return distances.getDistance(vehId, pickupId);
        }

        int64_t getTotalLocatingVehiclesTimeForRequest() const {
            return vehicleLocator.getTotalLocatingVehiclesTimeForRequest();
        }

        int64_t getTotalVehicleToPickupSearchTimeForRequest() const {
            return totalVehicleToPickupSearchTimeForRequest;
        }

        int64_t getTotalNumCHSearchesRunForRequest() const {
            return totalNumCHSearchesRunForRequest;
        }

    private:


        void fillDistancesForVehicleAtPrevStop(const Vehicle &vehicle, const unsigned int pickupId,
                                               const int distFromPrevStopToPickup) {
            const auto &stopLocations = routeState.stopLocationsFor(vehicle.vehicleId);
            if (stopLocations[0] != requestState.pickups[pickupId].loc) {
                distances.updateDistance(vehicle.vehicleId, pickupId, distFromPrevStopToPickup);
            } else {
                distances.updateDistance(vehicle.vehicleId, pickupId, 0);
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        const CH &ch;
        const RouteState &routeState;
        RequestState &requestState;


        VehLocToPickupDistances &distances;

        tbb::enumerable_thread_specific<FullQuery> chQuery;

        CAtomic<int64_t> totalVehicleToPickupSearchTimeForRequest;
        CAtomic<int64_t> totalNumCHSearchesRunForRequest;


    };

}