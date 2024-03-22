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
#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"

namespace karri {


    template<typename InputGraphT,
            typename VehicleLocatorT,
            typename CHEnvT,
            typename LabelSetT>
    class BatchedVehLocToPickupSearches {

    private:


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

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
            explicit WriteVehicleDistLabel(BatchedVehLocToPickupSearches &searches) : searches(searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                searches.vehicleBuckets.insert(v, BucketEntry(searches.curVehId, distToV[0]));
                searches.vehicleBucketsSearchSpace.push_back(v);
                return false;
            }

        private:

            BatchedVehLocToPickupSearches &searches;
        };

        struct ScanVehicleEntries {
            explicit ScanVehicleEntries(BatchedVehLocToPickupSearches &searches,
                                        std::array<int, K> &localCurPickupIds)
                    : searches(searches),
                      localCurPickupIds(localCurPickupIds) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                for (const auto &e: searches.vehicleBuckets.getBucketOf(v)) {
                    const auto &vehId = e.targetId;
                    const auto &distFromVehLocToV = e.distToTarget;
                    assert(distFromVehLocToV < INFTY);
                    DistanceLabel dists = distToV + DistanceLabel(distFromVehLocToV);
                    for (int i = 0; i < K; ++i) {
                        searches.distances.updateDistance(vehId, localCurPickupIds[i], dists[i]);
                    }
                }
                return false;
            }

        private:

            BatchedVehLocToPickupSearches &searches;
            std::array<int, K> &localCurPickupIds;
        };

        using WriteVehLabelsSearch = typename CHEnvT::template UpwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>;
        using FindDistancesSearch = typename CHEnvT::template UpwardSearch<ScanVehicleEntries, StopWhenMaxDistExceeded, LabelSetT>;


    public:

        BatchedVehLocToPickupSearches(const InputGraphT &graph,
                                      const RelevantPDLocs &relevantPickupsBns,
                                      const CHEnvT &chEnv,
                                      const Fleet &fleet,
                                      const RouteState &routeState,
                                      RequestState &requestState,
                                      VehicleLocatorT &locator,
                                      VehLocToPickupDistances &distances)
                : inputGraph(graph),
                  vehicleLocator(locator),
                  relevantPickupsBns(relevantPickupsBns),
                  ch(chEnv.getCH()),
                  fleet(fleet),
                  routeState(routeState),
                  requestState(requestState),
                  distances(distances),
                  writeVehLabelsSearch(
                          chEnv.template getForwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>(
                                  WriteVehicleDistLabel(*this), StopWhenMaxDistExceeded(curLeeway))),
                  pickupQueries([&] {
                      return chEnv.template getReverseSearch<ScanVehicleEntries, StopWhenMaxDistExceeded, LabelSetT>(
                              ScanVehicleEntries(*this, curPickupIds.local()),
                              StopWhenMaxDistExceeded(maxLeewayOfMarkedVehicles));
                  }),
                  curPickupIds(),
                  vehicleBuckets(inputGraph.numVertices()) {}

        void initialize() {
            distances.init();
            vehicleLocator.init(requestState.originalRequest.requestTime);

            for (const auto& v : vehicleBucketsSearchSpace)
                vehicleBuckets.clearBucket(v);
            vehicleBucketsSearchSpace.clear();

            curLeeway = INFTY;
            maxLeewayOfMarkedVehicles = 0;

            totalLocatingVehiclesTimeForRequest = 0;
            totalVehicleToPickupSearchTimeForRequest = 0;
            totalNumCHSearchesRunForRequest = 0;
        }

        // Construct bucket entries for all marked vehicles.
        template<typename VehicleIdsT>
        void buildBucketsForMarkedVehiclesSequential(const VehicleIdsT &markedVehicles) {
            Timer timer;

            for (const auto &vehId: markedVehicles) {
                const auto &veh = fleet[vehId];
                vehicleLocator.locateVehicle(veh);
                const auto &vehLocation = vehicleLocator.getVehicleLocation(vehId);

                // Skip vehicles that are at their first stop:
                if (vehLocation.location == routeState.stopLocationsFor(vehId)[0]) {
                    fillDistancesForVehicleAtPrevStop(veh);
                    continue;
                }

                const auto distToCurLoc = vehLocation.depTimeAtHead - routeState.schedDepTimesFor(vehId)[0];

                // If any pickups are at the current vehicle location, store the distance to the current veh location
                for (int pickupId = 0; pickupId < requestState.numPickups(); ++pickupId) {
                    if (requestState.pickups[pickupId].loc == vehLocation.location)
                        distances.updateDistance(vehId, pickupId, distToCurLoc);
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
        void computeDistancesForMarkedPickupsParallel(const PickupIdsT &markedPickups) {
            Timer timer;

            tbb::parallel_for(0, markedPickups.size(), K, [&](const int firstPickupIndex) {
                std::array<int, K> targets = {};
                std::array<int, K> targetOffsets = {};
                auto &localCurPickupIds = curPickupIds.local();

                for (int i = 0; i < K; ++i) {
                    if (firstPickupIndex + i >= markedPickups.size()) {
                        targets[i] = targets[0];
                        targetOffsets[i] = targetOffsets[0];
                        localCurPickupIds[i] = localCurPickupIds[0];
                        continue;
                    }
                    const auto &pickup = requestState.pickups[*(markedPickups.begin() + firstPickupIndex + i)];
                    targets[i] = ch.rank(inputGraph.edgeTail(pickup.loc));
                    targetOffsets[i] = inputGraph.travelTime(pickup.loc);
                    localCurPickupIds[i] = pickup.id;
                }


                auto &query = pickupQueries.local();
                query.runWithOffset(targets, targetOffsets);
            });

            totalVehicleToPickupSearchTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
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

        void fillDistancesForVehicleAtPrevStop(const Vehicle &vehicle) {

            for (const auto& e : relevantPickupsBns.relevantSpotsFor(vehicle.vehicleId)) {
                distances.updateDistance(vehicle.vehicleId, e.pdId, e.distToPDLoc);
            }

            const auto &stopLocations = routeState.stopLocationsFor(vehicle.vehicleId);
            for (int pickupId = 0; pickupId < requestState.numPickups(); ++pickupId) {
                if (stopLocations[0] == requestState.pickups[pickupId].loc) {
                    distances.updateDistance(vehicle.vehicleId, pickupId, 0);
                }
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        const RelevantPDLocs &relevantPickupsBns;
        const CH &ch;
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;

        VehLocToPickupDistances &distances;

        std::vector<int> vehicleBucketsSearchSpace;

        WriteVehLabelsSearch writeVehLabelsSearch;
        int curVehId;
        int curLeeway;


        tbb::enumerable_thread_specific<FindDistancesSearch> pickupQueries;
        tbb::enumerable_thread_specific<std::array<int, K>> curPickupIds;
        int maxLeewayOfMarkedVehicles;

        int64_t totalLocatingVehiclesTimeForRequest;
        int64_t totalVehicleToPickupSearchTimeForRequest;
        int64_t totalNumCHSearchesRunForRequest;


        BucketContainer vehicleBuckets;
    };

}