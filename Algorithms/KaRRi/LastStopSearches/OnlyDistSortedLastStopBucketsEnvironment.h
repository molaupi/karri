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
#include "Algorithms/Buckets/LastStopBucketContainer.h"
#include "LastStopBucketsSortingType.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class OnlyDistSortedLastStopBucketsEnvironment {


        // .targetId is vehicle ID
        // .distToTarget is dist from last stop to vertex
        using LastStopEntry = BucketEntry;

        struct CompareEntries {
            bool operator()(const LastStopEntry &e1, const LastStopEntry &e2) const {
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:
        static constexpr LastStopBucketsSortingType SORTING = ONLY_DIST;

        using BucketContainer = SortedBucketContainer<LastStopEntry, CompareEntries>;

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
                auto entry = LastStopEntry(curVehId, distToV[0]);
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

        OnlyDistSortedLastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                                 const RouteState &routeState,
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
                          DeleteEntry(bucketContainer, vehicleId, verticesVisitedInSearch,
                                      entriesVisitedInSearch))),
                  stats(stats) {}


        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }

        void generateIdleBucketEntries(const Vehicle &veh) {
            // No differentiation between idle and non-idle vehicles.
            generateBucketEntries(veh);
        }

        void generateNonIdleBucketEntries(const Vehicle &veh) {
            // No differentiation between idle and non-idle vehicles.
            generateBucketEntries(veh);
        }

        void updateBucketEntries(const Vehicle &, const int) {
            // No op since bucket updates are only needed for buckets that are sorted by arrival time at last stop
            // for non-idle vehicles.
        }

        void removeIdleBucketEntries(const Vehicle &veh, const int prevLastStopIdx) {
            // No differentiation between idle and non-idle vehicles.
            removeBucketEntries(veh, prevLastStopIdx);
        }

        void removeNonIdleBucketEntries(const Vehicle &veh, const int prevLastStopIdx) {
            // No differentiation between idle and non-idle vehicles.
            removeBucketEntries(veh, prevLastStopIdx);
        }

    private:

        void generateBucketEntries(const Vehicle &veh) {
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[numStops - 1];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            depTimeOfVehAtLastStop = routeState.schedDepTimesFor(veh.vehicleId)[numStops - 1];

            maxDetourUntilEndOfServiceTime = veh.endOfServiceTime - depTimeOfVehAtLastStop;
            verticesVisitedInSearch = 0;
            entryGenSearch.run(ch.rank(stopVertex));

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsGenerateEntriesTime += time;
        }

        void removeBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex >= 0);
            assert(stopIndex < routeState.numStopsOf(veh.vehicleId));

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);
            depTimeOfVehAtLastStop = routeState.schedDepTimesFor(veh.vehicleId)[numStops - 1];

            entriesVisitedInSearch = 0;
            maxDetourUntilEndOfServiceTime = INFTY;
            verticesVisitedInSearch = 0;
            entryDelSearch.run(ch.rank(stopVertex));
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsDeleteEntriesTime += time;
        }

        using GenerateEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateEntry, StopWhenDistanceExceeded>;
        using DeleteEntriesSearch = typename CHEnvT::template UpwardSearch<DeleteEntry>;


        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;
        const RouteState &routeState;

        BucketContainer bucketContainer;

        int vehicleId;
        int depTimeOfVehAtLastStop;
        int maxDetourUntilEndOfServiceTime;
        int entriesVisitedInSearch;
        int verticesVisitedInSearch;

        GenerateEntriesSearch entryGenSearch;
        DeleteEntriesSearch entryDelSearch;

        karri::stats::UpdatePerformanceStats &stats;

    };
}