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
#include "Algorithms/Buckets/BucketEntry.h"

namespace karri {


    template<typename InputGraphT,
            typename VehicleLocatorT,
            typename EllipticPickupDistancesT,
            typename CHEnvT,
            typename LabelSetT>
    class ThreadSafeBCHVehicleLocToPickupSearches {

    private:


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        static constexpr VehicleLocation INVALID_LOC = {INVALID_EDGE, -1};

        static constexpr int unknownDist = INFTY + 1;

        // In bucket entries, .targetId refers to vehicle ID, and distance refers to the sum of the distance from
        // stop 0 to the vehicle's current location and the upwards distance from the current location to the vertex.
        using BucketContainer = DynamicBucketContainer<BucketEntry>;


        struct StopWhenMaxDistExceeded {

            explicit StopWhenMaxDistExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return !anySet(~(maxDist < distToV));
            }


        private:
            const int &maxDist;
        };

        struct WriteVehicleDistLabel {
            explicit WriteVehicleDistLabel(ThreadSafeBCHVehicleLocToPickupSearches &searches) : searches(searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                searches.vehicleBuckets.insert(v, BucketEntry(searches.curVehId, distToV[0]));
                return false;
            }

        private:

            ThreadSafeBCHVehicleLocToPickupSearches &searches;
        };

        struct ScanVehicleEntries {
            explicit ScanVehicleEntries(ThreadSafeBCHVehicleLocToPickupSearches &searches,
                                        std::array<int, K>& localCurPickupIds)
                                        : searches(searches),
                                          localCurPickupIds(localCurPickupIds) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                for (const auto& e : searches.vehicleBuckets.getBucketOf(v)) {
                    const auto &vehId = e.targetId;
                    const auto &distFromVehLocToV = e.distToTarget;
                    assert(distFromVehLocToV < INFTY);
                    DistanceLabel dists = distToV + DistanceLabel(distFromVehLocToV);
                    for (int i = 0; i < K; ++i) {
                        searches.updateDistance(vehId, localCurPickupIds[i], dists[i]);
                    }
                }
                return false;
            }

        private:

            ThreadSafeBCHVehicleLocToPickupSearches &searches;
            std::array<int, K>& localCurPickupIds;
        };

        using WriteVehLabelsSearch = typename CHEnvT::template UpwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>;
        using FindDistancesSearch = typename CHEnvT::template UpwardSearch<ScanVehicleEntries, StopWhenMaxDistExceeded, LabelSetT>;


    public:

        ThreadSafeBCHVehicleLocToPickupSearches(const InputGraphT &graph,
                                  VehicleLocatorT &locator,
                                  const EllipticPickupDistancesT& ellipticPickupDistances,
                                  const CHEnvT &chEnv,
                                  const Fleet& fleet,
                                  const RouteState &routeState,
                                  RequestState &requestState)
                : inputGraph(graph),
                  vehicleLocator(locator),
                  ellipticPickupDistances(ellipticPickupDistances),
                  ch(chEnv.getCH()),
                  fleet(fleet),
                  routeState(routeState),
                  requestState(requestState),
                  distances(),
                  currentVehicleLocations(fleet.size(), INVALID_LOC),
                  prevNumPickups(0),
                  vehiclesWithKnownLocation(),
                  writeVehLabelsSearch(
                          chEnv.template getForwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>(
                                  WriteVehicleDistLabel(*this), StopWhenMaxDistExceeded(curLeeway))),
                  pickupQueries( [&] {
                      return chEnv.template getReverseSearch<ScanVehicleEntries, StopWhenMaxDistExceeded, LabelSetT>(
                                  ScanVehicleEntries(*this, curPickupIds.local()),
                                  StopWhenMaxDistExceeded(maxLeewayOfMarkedVehicles));}),
                  curPickupIds(),
                  currentTime(-1),
                  vehicleBuckets(inputGraph.numVertices()) {}

        void initialize(const int now) {
            currentTime = now;

            clearDistances();

            curLeeway = INFTY;
            maxLeewayOfMarkedVehicles = 0;

            totalLocatingVehiclesTimeForRequest = 0;
            totalVehicleToPickupSearchTimeForRequest = 0;
            totalNumCHSearchesRunForRequest = 0;
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleet.size());
            assert(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return distances[idx] != unknownDist;
        }

        int getDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleet.size());
            assert(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return distances[idx];
        }

        bool knowsCurrentLocationOf(const int vehId) const {
            assert(vehId >= 0 && vehId < fleet.size());
            return currentVehicleLocations[vehId] != INVALID_LOC;
        }

        const VehicleLocation &getCurrentLocationOf(const int vehId) const {
            assert(vehId >= 0 && vehId < fleet.size());
            return currentVehicleLocations[vehId];
        }

        // Construct bucket entries for all marked vehicles.
        template<typename VehicleIdsT, typename PickupIdsT>
        void buildBucketsForMarkedVehiclesSequential(const VehicleIdsT& markedVehicles, const PickupIdsT& markedPickups) {
            Timer timer;

            for (const auto& vehId : markedVehicles) {
                const auto& veh = fleet[vehId];
                if (!knowsCurrentLocationOf(vehId)) {
                    currentVehicleLocations[vehId] = locateVehicle(veh);
                    vehiclesWithKnownLocation.push_back(vehId);
                }
                const auto &vehLocation = currentVehicleLocations[vehId];
                assert(vehLocation != INVALID_LOC);

                // Skip vehicles that are at their first stop:
                if (vehLocation.location == routeState.stopLocationsFor(vehId)[0]) {
                    fillDistancesForVehicleAtPrevStop(veh, markedPickups);
                    continue;
                }

                const auto distToCurLoc = vehLocation.depTimeAtHead - routeState.schedDepTimesFor(vehId)[0];

                // If any marked pickups are at the current vehicle location, store the distance
                for (const auto& pickupId : markedPickups) {
                    if (requestState.pickups[pickupId].loc == vehLocation.location)
                        updateDistance(vehId, pickupId, distToCurLoc);
                }

                const auto source = ch.rank(inputGraph.edgeHead(vehLocation.location));
                curLeeway = routeState.leewayOfLegStartingAt(routeState.stopIdsFor(vehId)[0]);
                curVehId = vehId;
                writeVehLabelsSearch.runWithOffset(source, distToCurLoc);

                maxLeewayOfMarkedVehicles = std::max(maxLeewayOfMarkedVehicles, curLeeway);
            }

            totalVehicleToPickupSearchTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
        }

        // Compute distances from vehicles for which buckets have been constructed using
        // buildBucketsForMarkedVehiclesSequential() to all marked pickups in parallel.
        template<typename PickupIdsT>
        void computeDistancesForMarkedPickupsParallel(const PickupIdsT& markedPickups) {
            Timer timer;

            tbb::parallel_for(0, markedPickups.size(), K, [&](const int firstPickupIndex) {
                std::array<int, K> targets;
                std::array<int, K> targetOffsets;
                auto& localCurPickupIds = curPickupIds.local();

                for (int i = 0; i < K; ++i) {
                    if (firstPickupIndex + i >= markedPickups.size()) {
                        targets[i] = targets[0];
                        targetOffsets[i] = targetOffsets[0];
                        localCurPickupIds[i] = localCurPickupIds[0];
                        continue;
                    }
                    const auto& pickup = requestState.pickups[*(markedPickups.begin() + firstPickupIndex + i)];
                    targets[i] = ch.rank(inputGraph.edgeTail(pickup.loc));
                    targetOffsets[i] = inputGraph.travelTime(pickup.loc);
                    localCurPickupIds[i] = pickup.id;
                }


                auto& query = pickupQueries.local();
                query.runWithOffset(targets, targetOffsets);
            });
            prevNumPickups = requestState.numPickups();

            totalVehicleToPickupSearchTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
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

        void updateDistance(const int vehId, const int pickupId, const int newDist) {
            const int idx = vehId * requestState.numPickups() + pickupId;
            auto& dist = distances[idx];
            dist = std::min(dist, newDist);
        }

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
            vehicleBuckets.clear();

            const int numDistances = requestState.numPickups() * fleet.size();
            if (numDistances > distances.size()) {
                const int diff = numDistances - distances.size();
                distances.insert(distances.end(), diff, unknownDist);
            }
        }

        VehicleLocation locateVehicle(const Vehicle &vehicle) {
            Timer timer;
            const auto curLoc = vehicleLocator.computeCurrentLocation(vehicle, currentTime);
            totalLocatingVehiclesTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
            return curLoc;
        }

        template<typename PickupIdsT>
        void fillDistancesForVehicleAtPrevStop(const Vehicle &vehicle, const PickupIdsT& markedPickups) {
            const auto &stopLocations = routeState.stopLocationsFor(vehicle.vehicleId);
            const auto &stopIds = routeState.stopIdsFor(vehicle.vehicleId);
            for (const auto &pickupId : markedPickups) {
                if (stopLocations[0] != requestState.pickups[pickupId].loc) {
                    const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                    distances[idx] = ellipticPickupDistances.getDistanceFromStopToPDLoc(stopIds[0], pickupId);
                } else {
                    const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                    distances[idx] = 0;
                }
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        const EllipticPickupDistancesT& ellipticPickupDistances;
        const CH &ch;
        const Fleet& fleet;
        const RouteState &routeState;
        RequestState &requestState;


        std::vector<int> distances;
        std::vector<VehicleLocation> currentVehicleLocations;
        int prevNumPickups;
        std::vector<int> vehiclesWithKnownLocation;

        WriteVehLabelsSearch writeVehLabelsSearch;
        int curVehId;
        int curLeeway;


        tbb::enumerable_thread_specific<FindDistancesSearch> pickupQueries;
        tbb::enumerable_thread_specific<std::array<int, K>> curPickupIds;
        int maxLeewayOfMarkedVehicles;

        int currentTime;

        int64_t totalLocatingVehiclesTimeForRequest;
        int64_t totalVehicleToPickupSearchTimeForRequest;
        int64_t totalNumCHSearchesRunForRequest;


        BucketContainer vehicleBuckets;
    };

}