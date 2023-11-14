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
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Timer.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT, bool SORTED_BUCKETS, typename RouteStateT>
    class LastStopBucketsEnvironment {


        using Entry = BucketEntry;

        struct IsDistSmaller {
            bool operator()(const Entry &e1, const Entry &e2) const {
                // In last stop bucket entries .targetId refers to the vehicle's id and .distToTarget is the distance from the
                // vehicles last stop.
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:
        static constexpr bool SORTED_BY_DIST = SORTED_BUCKETS;

        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                SortedBucketContainer<Entry, IsDistSmaller>,
                DynamicBucketContainer<Entry>
        >;

    private:

        struct StopWhenDistanceExceeded {
            explicit StopWhenDistanceExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                return distToV[0] > maxDist;
            }

        private:
            const int &maxDist;
        };

        struct GenerateEntry {
            explicit GenerateEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0]);
                bucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
        };

        struct DeleteEntry {
            explicit DeleteEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited,
                                 int &entriesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited),
                      entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0]);
                bool removed = bucketContainer.remove(v, entry);
                ++verticesVisited;
                entriesVisited += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
            int &entriesVisited;
        };

    public:

        LastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteStateT &routeState,
                                   karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.upwardGraph()),
                  routeState(routeState),
                  bucketContainer(searchGraph.numVertices()),
                  entryGenSearch(
                          chEnv.getForwardSearch(GenerateEntry(bucketContainer, vehicleId, verticesVisitedInSearch),
                                                 StopWhenDistanceExceeded(maxDetourUntilEndOfServiceTime))),
                  entryDelSearch(chEnv.getForwardSearch(
                          DeleteEntry(bucketContainer, vehicleId, verticesVisitedInSearch, entriesVisitedInSearch))),
                  stats(stats) {}


        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }


        void generateBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex == routeState.numStopsOf(veh.vehicleId) - 1);

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            maxDetourUntilEndOfServiceTime =
                    veh.endOfServiceTime - routeState.schedDepTimesFor(vehicleId)[routeState.numStopsOf(vehicleId) - 1];
            verticesVisitedInSearch = 0;
            entryGenSearch.run(ch.rank(stopVertex));
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsGenerateEntriesTime += time;

//        bucketGenLogger << verticesVisitedInSearch << ',' << time << '\n';
        }

        void removeBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex >= 0);
            assert(stopIndex < routeState.numStopsOf(veh.vehicleId));

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            entriesVisitedInSearch = 0;
            maxDetourUntilEndOfServiceTime = INFTY;
            verticesVisitedInSearch = 0;
            entryDelSearch.run(ch.rank(stopVertex));
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsDeleteEntriesTime += time;

//        bucketDelLogger << verticesVisitedInSearch << ',' << entriesVisitedInSearch << ',' << time << '\n';
        }

    private:

        using GenerateEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateEntry, StopWhenDistanceExceeded>;
        using DeleteEntriesSearch = typename CHEnvT::template UpwardSearch<DeleteEntry>;


        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;
        const RouteStateT &routeState;

        BucketContainer bucketContainer;

        int vehicleId;
        int maxDetourUntilEndOfServiceTime;
        int entriesVisitedInSearch;
        int verticesVisitedInSearch;

        GenerateEntriesSearch entryGenSearch;
        DeleteEntriesSearch entryDelSearch;

        karri::stats::UpdatePerformanceStats &stats;

    };

    struct NoOpLastStopBucketsEnvironment {

        inline void generateBucketEntries(const Vehicle &, const int) {/* no op */}

        inline void removeBucketEntries(const Vehicle &, const int) {/* no op */}
    };
}