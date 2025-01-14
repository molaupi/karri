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

namespace karri {

    // Last stop buckets sorted only by cost without distinguishing between idle and non-idle vehicles.
    // This is enough if we don't consider rider costs.
    template<typename InputGraphT, typename CHEnvT>
    class SimpleSortedLastStopBucketsEnvironment {

        struct LastStopEntry {
            int targetId;
            int distToTarget; // cost (SP distance according to traversal cost)
            int travelTimeToTarget; // travel time (not SP distance)
        };

        struct CompareEntries {
            bool operator()(const LastStopEntry &e1, const LastStopEntry &e2) const {
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:
        static constexpr bool SORTED = true;

        using BucketContainer = SortedBucketContainer<LastStopEntry, CompareEntries>;

    private:

        struct StopIfTravelTimeExceedsMaxDist {
            explicit StopIfTravelTimeExceedsMaxDist(const int &maxDist,
                                                    StampedDistanceLabelContainer<int> &travelTimes)
                    : maxDist(maxDist), travelTimes(travelTimes) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
                return travelTimes[v] > maxDist;
            }

        private:
            const int &maxDist;
            StampedDistanceLabelContainer<int> &travelTimes;
        };

        struct GenerateEntry {
            explicit GenerateEntry(BucketContainer &bucketContainer,
                                   StampedDistanceLabelContainer<int> &travelTimes,
                                   int &curVehId, int &verticesVisited)
                    : bucketContainer(bucketContainer), travelTimes(travelTimes),
                      curVehId(curVehId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &costToV, const DistLabelContT &) {
                auto entry = LastStopEntry(curVehId, costToV[0], travelTimes[v]);
                bucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            StampedDistanceLabelContainer<int> &travelTimes;
            int &curVehId;
            int &verticesVisited;
        };

        struct DeleteEntry {
            explicit DeleteEntry(BucketContainer &bucketContainer,
                                 StampedDistanceLabelContainer<int> &travelTimes,
                                 int &curVehId, int &verticesVisited, int &entriesVisited)
                    : bucketContainer(bucketContainer), travelTimes(travelTimes),
                      curVehId(curVehId), verticesVisited(verticesVisited),
                      entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0], travelTimes[v]);
                bool removed = bucketContainer.removeIdle(v, entry);
                ++verticesVisited;
                entriesVisited += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            BucketContainer &bucketContainer;
            StampedDistanceLabelContainer<int> &travelTimes;
            int &curVehId;
            int &verticesVisited;
            int &entriesVisited;
        };

        struct UpdateTravelTimeCallback {

            UpdateTravelTimeCallback(const typename CH::SearchGraph &searchGraph,
                                     StampedDistanceLabelContainer<int> &travelTimes) : searchGraph(searchGraph),
                                                                                        travelTimes(travelTimes) {}

            template<typename LabelMaskT, typename DistanceLabelContainerT>
            void operator()(const int v, const int w, const int e, const LabelMaskT &improved,
                            const DistanceLabelContainerT &) {
                if (improved[0])
                    travelTimes[w] = travelTimes[v] + searchGraph.travelTime(e);
            }

            const CH::SearchGraph &searchGraph;
            StampedDistanceLabelContainer<int> &travelTimes;
        };

    public:

        SimpleSortedLastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                               const RouteState &routeState,
                                               karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.upwardGraph()),
                  routeState(routeState),
                  bucketContainer(searchGraph.numVertices()),
                  travelTimes(inputGraph.numVertices()),
                  entryGenSearch(
                          chEnv.getForwardSearch(
                                  GenerateEntry(bucketContainer, travelTimes, vehicleId, verticesVisitedInSearch),
                                  StopIfTravelTimeExceedsMaxDist(maxDetourUntilEndOfServiceTime, travelTimes),
                                  UpdateTravelTimeCallback(ch.upwardGraph(), travelTimes))),
                  entryDelSearch(chEnv.getForwardSearch(
                                         DeleteEntry(bucketContainer, travelTimes, vehicleId, verticesVisitedInSearch,
                                                     entriesVisitedInSearch),
                                         {},
                                         UpdateTravelTimeCallback(ch.upwardGraph(), travelTimes))),
                  stats(stats) {}


        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }

        // Implement the last stops at vertices interface.
        std::vector<int> vehiclesWithLastStopAt(const int vertex) const {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            const auto rank = ch.rank(vertex);
            std::vector<int> vehIds;

            for (const auto &entry: bucketContainer.getBucketOf(rank)) {
                if (entry.distToTarget == 0)
                    vehIds.push_back(entry.targetId);
            }

            return vehIds;
        }

        void generateBucketEntries(const Vehicle &veh) {
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[numStops - 1];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            const int depTimeOfVehAtLastStop = routeState.schedDepTimesFor(veh.vehicleId)[numStops - 1];
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
            entriesVisitedInSearch = 0;
            maxDetourUntilEndOfServiceTime = INFTY;
            verticesVisitedInSearch = 0;
            entryDelSearch.run(ch.rank(stopVertex));
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsDeleteEntriesTime += time;
        }

    private:

        using GenerateEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateEntry, StopIfTravelTimeExceedsMaxDist, UpdateTravelTimeCallback>;
        using DeleteEntriesSearch = typename CHEnvT::template UpwardSearch<DeleteEntry, dij::NoCriterion, UpdateTravelTimeCallback>;

        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;
        const RouteState &routeState;

        BucketContainer bucketContainer;
        StampedDistanceLabelContainer<int> travelTimes;

        int vehicleId;
        int maxDetourUntilEndOfServiceTime;
        int entriesVisitedInSearch;
        int verticesVisitedInSearch;

        GenerateEntriesSearch entryGenSearch;
        DeleteEntriesSearch entryDelSearch;

        karri::stats::UpdatePerformanceStats &stats;

    };
}