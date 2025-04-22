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
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class EllipticSearchSpaces {

        struct StopWhenLeewayExceeded {
            explicit StopWhenLeewayExceeded(const int &currentLeeway) : currentLeeway(currentLeeway) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, const DistLabelT &distToV, const DistLabelContT &) const noexcept {
                return allSet(distToV > currentLeeway);
            }

            const int &currentLeeway;
        };

        struct StoreSearchSpace {
            explicit StoreSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &, const DistLabelContT &) const noexcept {
                searchSpace.push_back(v);
                return false;
            }

            std::vector<int> &searchSpace;
        };

        struct CountSubset {

            struct IndexAndCount {
                int index = INVALID_INDEX;
                int count = 0;
            };

            CountSubset(const int capacity) : indexAndCount(capacity), elements() {
                elements.reserve(capacity);
            }

            size_t size() const {
                return elements.size();
            }

            void clear() {
                for (const auto &i: elements) {
                    indexAndCount[i].index = INVALID_INDEX;
                    indexAndCount[i].count = 0;
                }
                elements.clear();
            }

            const std::vector<int> &getElements() const {
                return elements;
            }

            bool contains(const int& element) const {
                return indexAndCount[element].index != INVALID_INDEX;
            }

            void addOccurrence(const int i) {
                KASSERT(i >= 0 && i < indexAndCount.size());
                if (indexAndCount[i].index == INVALID_INDEX) {
                    indexAndCount[i].index = elements.size();
                    elements.push_back(i);
                }
                ++indexAndCount[i].count;
            }

            void removeOccurrence(const int i) {
                KASSERT(i >= 0 && i < indexAndCount.size());
                KASSERT(indexAndCount[i].count > 0);
                --indexAndCount[i].count;
                // If there are no more occurrences of i, remove it from the union:
                if (indexAndCount[i].count == 0) {
                    indexAndCount[elements.back()].index = indexAndCount[i].index;
                    elements[indexAndCount[i].index] = elements.back();
                    indexAndCount[i].index = INVALID_INDEX;
                    elements.pop_back();
                };
            }

        private:

            std::vector<IndexAndCount> indexAndCount;
            std::vector<int> elements;

        };


    public:


        EllipticSearchSpaces(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteState &routeState)
                : inputGraph(inputGraph), ch(chEnv.getCH()), routeState(routeState),
                  sourcePos(inputGraph.numVertices(), dynamic_ragged2d::ValueBlockPosition(0, 0)),
                  targetPos(inputGraph.numVertices(), dynamic_ragged2d::ValueBlockPosition(0, 0)),
                  sourceVerticesUnion(inputGraph.numVertices()),
                  targetVerticesUnion(inputGraph.numVertices()),
                  sourceEdgesUnion(ch.upwardGraph().numEdges()),
                  targetEdgesUnion(ch.downwardGraph().numEdges()),
                  forwardSearchFromNewStop(
                          chEnv.getForwardTopologicalSearch(StoreSearchSpace(searchSpaceForUpSearch),
                                                            StopWhenLeewayExceeded(currentLeeway))),
                  reverseSearchFromNewStop(
                          chEnv.getReverseTopologicalSearch(StoreSearchSpace(searchSpaceForUpSearch),
                                                            StopWhenLeewayExceeded(currentLeeway))),
                  forwardSearchFromPrevStop(chEnv.getForwardSearch({}, StopWhenLeewayExceeded(currentLeeway))),
                  reverseSearchFromNextStop(chEnv.getReverseSearch({}, StopWhenLeewayExceeded(currentLeeway))),
                  currentLeeway(INFTY),
                  searchSpaceForUpSearch(),
                  descendentHasEntry(inputGraph.numVertices()) {}

        const std::vector<int> &getUnionOfSourceSearchSpaceVertices() const {
            return sourceVerticesUnion.getElements();
        }

        const std::vector<int> &getUnionOfTargetSearchSpaceVertices() const {
            return targetVerticesUnion.getElements();
        }

        const CountSubset &getUnionOfSourceSearchSpaceEdges() const {
            return sourceEdgesUnion;
        }

        const CountSubset &getUnionOfTargetSearchSpaceEdges() const {
            return targetEdgesUnion;
        }

        void generateSourceSearchSpace(const int vehId, const int stopIndex,
                                       karri::stats::UpdatePerformanceStats &stats) {
            assert(routeState.numStopsOf(vehId) > stopIndex + 1);

            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(vehId)[stopIndex + 1],
                                        routeState.schedDepTimesFor(vehId)[stopIndex + 1]) -
                               routeState.schedDepTimesFor(vehId)[stopIndex] -
                               InputConfig::getInstance().stopTime;

            if (leeway <= 0)
                return;

            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeHead(newStopLoc));

            const int nextStopLoc = routeState.stopLocationsFor(vehId)[stopIndex + 1];
            const int nextStopRoot = ch.rank(inputGraph.edgeTail(nextStopLoc));
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);

            generateSearchSpace(stopId, leeway,
                                newStopRoot, 0, forwardSearchFromNewStop, ch.upwardGraph(),
                                nextStopRoot, nextStopOffset, reverseSearchFromNextStop, ch.downwardGraph(),
                                sourcePos, sourceSearchSpaceVertices, sourceSearchSpaceEdges, sourceVerticesUnion,
                                sourceEdgesUnion, stats);
        }

        void generateTargetSearchSpace(const int vehId, const int stopIndex,
                                       karri::stats::UpdatePerformanceStats &stats) {
            assert(stopIndex > 0);

            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int leeway = std::max(routeState.maxArrTimesFor(vehId)[stopIndex],
                                        routeState.schedDepTimesFor(vehId)[stopIndex]) -
                               routeState.schedDepTimesFor(vehId)[stopIndex - 1] -
                               InputConfig::getInstance().stopTime;
            if (leeway <= 0)
                return;

            currentLeeway = leeway;

            const int newStopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const int newStopRoot = ch.rank(inputGraph.edgeTail(newStopLoc));
            const int newStopOffset = inputGraph.travelTime(newStopLoc);

            const int prevStopLoc = routeState.stopLocationsFor(vehId)[stopIndex - 1];
            const int prevStopRoot = ch.rank(inputGraph.edgeHead(prevStopLoc));

            generateSearchSpace(stopId, leeway,
                                newStopRoot, newStopOffset, reverseSearchFromNewStop, ch.downwardGraph(),
                                prevStopRoot, 0, forwardSearchFromPrevStop, ch.upwardGraph(),
                                targetPos, targetSearchSpaceVertices, targetSearchSpaceEdges, targetVerticesUnion,
                                targetEdgesUnion, stats);
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle &) {
            // TODO: could remove current search spaces and recompute them for new leeways
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle &) {
            // TODO: could remove current search spaces and recompute them for new leeways
        }

        void deleteSourceSearchSpace(const int vehId, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            deleteSearchSpace(stopId, sourcePos, sourceSearchSpaceVertices, sourceSearchSpaceEdges, sourceVerticesUnion,
                              sourceEdgesUnion);
        }

        void deleteTargetSearchSpace(const int vehId, const int stopIndex) {
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            deleteSearchSpace(stopId, targetPos, targetSearchSpaceVertices, targetSearchSpaceEdges, targetVerticesUnion,
                              targetEdgesUnion);
        }

    private:


        // Searches for inserting new stops: Topo search from/to new stop, regular CH searches to/from neighboring stops.
        using SearchFromNewStop = typename CHEnvT::template TopologicalUpwardSearch<
                StoreSearchSpace, StopWhenLeewayExceeded>;
        using SearchFromNeighbor = typename CHEnvT::template UpwardSearch<
                dij::NoCriterion, StopWhenLeewayExceeded>;

        void generateSearchSpace(const int stopId, const int leeway,
                                 const int newStopRoot, const int newStopOffSet, SearchFromNewStop &searchFromNewStop,
                                 const CH::SearchGraph &newStopGraph,
                                 const int neighborRoot, const int neighborOffset,
                                 SearchFromNeighbor &searchFromNeighbor,
                                 const CH::SearchGraph &neighborGraph,
                                 std::vector<dynamic_ragged2d::ValueBlockPosition> &pos,
                                 std::vector<int> &searchSpaceVertices,
                                 std::vector<int> &searchSpaceEdges,
                                 CountSubset &verticesUnion,
                                 CountSubset &edgesUnion,
                                 stats::UpdatePerformanceStats &stats) {
            int64_t numEntriesGenerated = 0;
            Timer timer;

            // Run topological search from new stop and memorize search space:
            searchSpaceForUpSearch.clear();
            searchFromNewStop.runWithOffset(newStopRoot, newStopOffSet);

            // Run reverse CH query from next stop:
            searchFromNeighbor.runWithOffset(neighborRoot, neighborOffset);

            for (auto it = searchSpaceForUpSearch.crbegin(); it < searchSpaceForUpSearch.crend(); ++it) {
                const auto v = *it;

                // Try to find witness for shortest path that has a higher ranked vertex:
                int minDistViaHigherPath = INFTY;
                FORALL_INCIDENT_EDGES(neighborGraph, v, e) {
                    const int higherVertex = neighborGraph.edgeHead(e);
                    const int distViaHigherVertex =
                            searchFromNewStop.getDistance(higherVertex) + neighborGraph.traversalCost(e);
                    minDistViaHigherPath = std::min(minDistViaHigherPath, distViaHigherVertex);
                }
                const bool betterHigherPathExists = minDistViaHigherPath <= searchFromNewStop.getDistance(v);
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
                if ((!betterHigherPathExists && inEllipse) || descendentHasEntry[v]) {
                    const int idx = dynamic_ragged2d::insertion(stopId, v, pos, searchSpaceVertices, searchSpaceEdges);
                    const auto parentEdge = searchFromNewStop.getParentEdge(v);
                    searchSpaceEdges[idx] = parentEdge;
                    verticesUnion.addOccurrence(v);
                    if (parentEdge != INVALID_EDGE)
                        edgesUnion.addOccurrence(parentEdge);
                    ++numEntriesGenerated;

                    // Always insert entries at every vertex on the branch, so we obtain a tree of entries.
                    descendentHasEntry[searchFromNewStop.getParentVertex(v)] = true;
                    descendentHasEntry[v] = false;
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.elliptic_generate_time += time;
            stats.elliptic_generate_numVerticesInSearchSpace += searchSpaceForUpSearch.size();
            stats.elliptic_generate_numEntriesInserted += numEntriesGenerated;
        }

        void deleteSearchSpace(const int stopId,
                               std::vector<dynamic_ragged2d::ValueBlockPosition> &pos,
                               std::vector<int> &searchSpaceVertices,
                               std::vector<int> &searchSpaceEdges,
                               CountSubset &verticesUnion,
                               CountSubset &edgesUnion) {

            const auto start = pos[stopId].start;
            const auto end = pos[stopId].end;
            for (int i = start; i < end; ++i) {
                const auto v = searchSpaceVertices[i];
                verticesUnion.removeOccurrence(v);
                const auto e = searchSpaceEdges[i];
                KASSERT(i < end - 1 || e == INVALID_EDGE);
                if (i < end - 1) {
                    KASSERT(e != INVALID_EDGE);
                    edgesUnion.removeOccurrence(e);
                }
            }

            dynamic_ragged2d::removalOfAllCols(stopId, pos, searchSpaceVertices);
        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;

        std::vector<dynamic_ragged2d::ValueBlockPosition> sourcePos;
        std::vector<int> sourceSearchSpaceVertices;
        std::vector<int> sourceSearchSpaceEdges;

        std::vector<dynamic_ragged2d::ValueBlockPosition> targetPos;
        std::vector<int> targetSearchSpaceVertices;
        std::vector<int> targetSearchSpaceEdges;

        CountSubset sourceVerticesUnion;
        CountSubset targetVerticesUnion;

        CountSubset sourceEdgesUnion;
        CountSubset targetEdgesUnion;

        SearchFromNewStop forwardSearchFromNewStop;
        SearchFromNewStop reverseSearchFromNewStop;
        SearchFromNeighbor forwardSearchFromPrevStop;
        SearchFromNeighbor reverseSearchFromNextStop;
        int currentLeeway;

        std::vector<int> searchSpaceForUpSearch;
        BitVector descendentHasEntry;
    };
}