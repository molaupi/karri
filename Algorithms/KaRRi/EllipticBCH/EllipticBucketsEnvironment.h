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
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "BucketEntryWithLeeway.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"

namespace karri {

    template<typename InputGraphT, typename TravelTimeCHEnvT, typename TraversalCostCHEnvT>
    class EllipticBucketsEnvironment {

    public:
        static constexpr bool SORTED_BY_REM_LEEWAY = false;

        struct Entry {
            int targetId = INVALID_ID;
            int distToTarget = INFTY; // traversal cost (shortest path)
            int travelTimeToTarget = INFTY; // travel time (not shortest path)

            friend bool operator==(const Entry& e1, const Entry& e2) {
                return e1.targetId == e2.targetId;
            }
        };
        using BucketContainer = DynamicBucketContainer<Entry>;

    private:

        struct StoreSearchSpace {
            explicit StoreSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &, const DistLabelContT &) const noexcept {
                searchSpace.push_back(v);
                return false;
            }

            std::vector<int> &searchSpace;
        };


        struct GenerateEntryIfInEllipse {
            explicit GenerateEntryIfInEllipse(EllipticBucketsEnvironment &env, BucketContainer& buckets)
                    : env(env), buckets(buckets) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {

                // Prune if not in ellipse
                if (!env.ellipse.contains(env.costCh.contractionOrder(v)))
                    return true;

                // Otherwise generate bucket entry
                Entry entry = {env.currentStopId, distToV[0], env.travelTimes[v]};
                buckets.insert(v, entry);
                ++env.numEntriesGenerated;
                return false;
            }

            EllipticBucketsEnvironment &env;
            BucketContainer & buckets;
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


        EllipticBucketsEnvironment(const InputGraphT &inputGraph, const TravelTimeCHEnvT &timeChEnv,
                                   const TraversalCostCHEnvT &costChEnv,
                                   const RouteState &routeState,
                                   karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph),
                  timeCh(timeChEnv.getCH()),
                  costCh(costChEnv.getCH()),
                  routeState(routeState),
                  travelTimeForwardSearchFromNewStop(
                          timeChEnv.getForwardTopologicalSearch(StoreSearchSpace(searchSpace))),
                  travelTimeReverseSearchFromNewStop(
                          timeChEnv.getReverseTopologicalSearch(StoreSearchSpace(searchSpace))),
                  travelTimeForwardSearchFromPrevStop(timeChEnv.getForwardSearch()),
                  travelTimeReverseSearchFromNextStop(timeChEnv.getReverseSearch()),
                  currentLeeway(INFTY),
                  searchSpace(),
                  descendentHasEntry(inputGraph.numVertices()),
                  deleteSearchSpace(inputGraph.numVertices()),
                  ellipse(inputGraph.numVertices()),
                  sourceBuckets(inputGraph.numVertices()),
                  targetBuckets(inputGraph.numVertices()),
                  travelTimes(inputGraph.numVertices()),
                  generateSourceBucketsSearch(costChEnv.getForwardSearch(GenerateEntryIfInEllipse(*this, sourceBuckets), {},
                                                                         UpdateTravelTimeCallback(costCh.upwardGraph(),
                                                                                                  travelTimes))),
                  generateTargetBucketsSearch(costChEnv.getReverseSearch(GenerateEntryIfInEllipse(*this, targetBuckets), {},
                                                                         UpdateTravelTimeCallback(
                                                                                 costCh.downwardGraph(), travelTimes))),
                  stats(stats) {}


        const BucketContainer &getSourceBuckets() const {
            return sourceBuckets;
        }

        const BucketContainer &getTargetBuckets() const {
            return targetBuckets;
        }

        void generateSourceBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(routeState.numStopsOf(veh.vehicleId) > stopIndex + 1);

            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(veh.vehicleId)[stopIndex + 1],
                                        routeState.schedDepTimesFor(veh.vehicleId)[stopIndex + 1]) -
                               routeState.schedDepTimesFor(veh.vehicleId)[stopIndex] -
                               InputConfig::getInstance().stopTime;

            if (leeway <= 0)
                return;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int nextStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex + 1];

            // Find ellipse
            currentLeeway = leeway;
            const int timeNewStopRoot = timeCh.rank(inputGraph.edgeHead(newStopLoc));
            const int timeNextStopRoot = timeCh.rank(inputGraph.edgeTail(nextStopLoc));
            const int timeNextStopOffset = inputGraph.travelTime(nextStopLoc);
            findEllipse(leeway, timeNewStopRoot, 0, travelTimeForwardSearchFromNewStop, timeCh.upwardGraph(),
                        timeNextStopRoot, timeNextStopOffset, travelTimeReverseSearchFromNextStop,
                        timeCh.downwardGraph());

            // Generate entries within ellipse:
            generateBucketEntriesInEllipse(stopId, costCh.rank(inputGraph.edgeHead(newStopLoc)), 0, 0,
                                          generateSourceBucketsSearch);
        }

        void generateTargetBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex > 0);

            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(veh.vehicleId)[stopIndex],
                                        routeState.schedDepTimesFor(veh.vehicleId)[stopIndex]) -
                               routeState.schedDepTimesFor(veh.vehicleId)[stopIndex - 1] -
                               InputConfig::getInstance().stopTime;
            if (leeway <= 0)
                return;

            const int newStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int prevStopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex - 1];

            // Find ellipse
            currentLeeway = leeway;
            const int timeNewStopRoot = timeCh.rank(inputGraph.edgeTail(newStopLoc));
            const int timeNewStopOffset = inputGraph.travelTime(newStopLoc);
            const int timePrevStopRoot = timeCh.rank(inputGraph.edgeHead(prevStopLoc));
            findEllipse(leeway, timeNewStopRoot, timeNewStopOffset, travelTimeReverseSearchFromNewStop,
                                  timeCh.downwardGraph(), timePrevStopRoot, 0, travelTimeForwardSearchFromPrevStop,
                                  timeCh.upwardGraph());

            // Generate entries within ellipse:
            const int costNewStopRoot = costCh.rank(inputGraph.edgeTail(newStopLoc));
            const int costNewStopCostOffset = inputGraph.traversalCost(newStopLoc);
            const int costNewStopTravelTimeOffset = inputGraph.travelTime(newStopLoc);
            generateBucketEntriesInEllipse(stopId, costNewStopRoot, costNewStopCostOffset, costNewStopTravelTimeOffset,
                                          generateTargetBucketsSearch);

        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle &) {
            // no op because buckets are not sorted
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle &) {
            // no op because buckets are not sorted
        }

        void deleteSourceBucketEntries(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = costCh.rank(inputGraph.edgeHead(stopLoc));
            deleteBucketEntries(stopId, root, costCh.upwardGraph(), sourceBuckets);
        }

        void deleteTargetBucketEntries(const Vehicle &veh, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(veh.vehicleId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const int root = costCh.rank(inputGraph.edgeTail(stopLoc));
            deleteBucketEntries(stopId, root, costCh.downwardGraph(), targetBuckets);
        }

    private:


        // Searches for inserting new stops: Topo search from/to new stop, regular CH searches to/from neighboring stops.
        using TravelTimeSearchFromNewStop = typename TravelTimeCHEnvT::template TopologicalUpwardSearch<StoreSearchSpace>;
        using TravelTimeSearchFromNeighbor = typename TravelTimeCHEnvT::template UpwardSearch<dij::NoCriterion>;

        using TraversalCostGenerateBucketEntriesSearch = typename TraversalCostCHEnvT::template UpwardSearch<GenerateEntryIfInEllipse, dij::NoCriterion, UpdateTravelTimeCallback>;

        void findEllipse(const int leeway,
                         const int newStopRoot, const int newStopOffSet, TravelTimeSearchFromNewStop &searchFromNewStop,
                         const CH::SearchGraph &newStopGraph,
                         const int neighborRoot, const int neighborOffset,
                         TravelTimeSearchFromNeighbor &searchFromNeighbor,
                         const CH::SearchGraph &neighborGraph) {
            Timer timer;

            ellipse.clear();

            // Run topological search from new stop and memorize search space:
            searchSpace.clear();
            searchFromNewStop.runWithOffset(newStopRoot, newStopOffSet);

            // Run reverse CH query from next stop:
            searchFromNeighbor.runWithOffset(neighborRoot, neighborOffset);

            for (auto it = searchSpace.crbegin(); it < searchSpace.crend(); ++it) {
                const auto v = *it;

                // Try to find witness for shortest path that has a higher ranked vertex:
                int minDistViaHigherPath = INFTY;
                FORALL_INCIDENT_EDGES(neighborGraph, v, e) {
                    const int higherVertex = neighborGraph.edgeHead(e);
                    const int distViaHigherVertex =
                            searchFromNewStop.getDistance(higherVertex) + neighborGraph.traversalCost(e);
                    minDistViaHigherPath = std::min(minDistViaHigherPath, distViaHigherVertex);
                }
                searchFromNewStop.distanceLabels[v].min(minDistViaHigherPath);


                // Propagate distances to neighbor down into search space:
                FORALL_INCIDENT_EDGES(newStopGraph, v, e) {
                    const int higherVertex = newStopGraph.edgeHead(e);
                    const int distViaHigherVertex =
                            newStopGraph.traversalCost(e) + searchFromNeighbor.getDistance(higherVertex);
                    searchFromNeighbor.distanceLabels[v].min(distViaHigherVertex);
                }

                // Check if vertex is in ellipse using distances to neighbor:
                const bool inEllipse = searchFromNewStop.getDistance(v) + searchFromNeighbor.getDistance(v) <= leeway;
                if (inEllipse || descendentHasEntry[v]) {
                    ellipse.insert(timeCh.contractionOrder(v));

                    // Always insert entries at every vertex on the branch, so we obtain a tree of entries that can be
                    // used in delete operations.
                    descendentHasEntry[searchFromNewStop.getParentVertex(v)] = true;
                    descendentHasEntry[v] = false;
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numVerticesInSearchSpace += searchSpace.size();
        }

        void generateBucketEntriesInEllipse(const int stopId, const int rootInCostCh, const int costOffset,
                                            const int travelTimeOffset,
                                            TraversalCostGenerateBucketEntriesSearch &search) {
            numEntriesGenerated = 0;
            currentStopId = stopId;

            Timer timer;

            travelTimes.init();
            travelTimes[rootInCostCh] = travelTimeOffset;

            search.runWithOffset(rootInCostCh, costOffset);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void
        deleteBucketEntries(const int stopId, const int root, const CH::SearchGraph &graph, BucketContainer &buckets) {
            int64_t numVerticesVisited = 0, numEntriesScanned = 0;
            Timer timer;
            deleteSearchSpace.clear();
            deleteSearchSpace.insert(root);
            for (auto it = deleteSearchSpace.begin(); it < deleteSearchSpace.end(); ++it) {
                const auto &v = *it;
                if (buckets.remove(v, stopId)) {
                    FORALL_INCIDENT_EDGES(graph, v, e) {
                        const auto w = graph.edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++numVerticesVisited;
                numEntriesScanned += buckets.getNumEntriesVisitedInLastUpdateOrRemove();
            }
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_delete_time += time;
            stats.elliptic_delete_numVerticesVisited += numVerticesVisited;
            stats.elliptic_delete_numEntriesScanned += numEntriesScanned;
        }


        const InputGraphT &inputGraph;
        const CH &timeCh;
        const CH &costCh;

        const RouteState &routeState;


        // Ellipse computation:
        TravelTimeSearchFromNewStop travelTimeForwardSearchFromNewStop;
        TravelTimeSearchFromNewStop travelTimeReverseSearchFromNewStop;
        TravelTimeSearchFromNeighbor travelTimeForwardSearchFromPrevStop;
        TravelTimeSearchFromNeighbor travelTimeReverseSearchFromNextStop;

        int currentLeeway;

        std::vector<int> searchSpace;

        BitVector descendentHasEntry;
        Subset deleteSearchSpace;

        Subset ellipse;

        // Bucket entry generation:
        BucketContainer sourceBuckets;
        BucketContainer targetBuckets;

        int currentStopId;
        StampedDistanceLabelContainer<int> travelTimes;
        TraversalCostGenerateBucketEntriesSearch generateSourceBucketsSearch;
        TraversalCostGenerateBucketEntriesSearch generateTargetBucketsSearch;

        int numEntriesGenerated;
        karri::stats::UpdatePerformanceStats &stats;
    };
}