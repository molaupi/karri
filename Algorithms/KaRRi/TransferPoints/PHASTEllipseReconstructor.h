/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "EdgeEllipseContainer.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "VertexInEllipse.h"
#include "PHASTEllipseReconstructorQuery.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/Timer.h"

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename InputGraphT,
            typename CHEnvT,
            typename EllipticBucketsEnvironmentT,
            bool ParallelizeQuery,
            int TOP_VERTICES_DIVISOR,
            typename WeightT = TraversalCostAttribute,
            typename LabelSetT = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>>
    class PHASTEllipseReconstructor {

        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;
        static constexpr int K = LabelSetT::K;

        using Query = PHASTEllipseReconstructorQuery<EllipticBucketsEnvironmentT, LabelSetT, TOP_VERTICES_DIVISOR, WeightT>;


        using P2PLabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;
        using P2PQuery = typename CHEnvT::template FullCHQuery<P2PLabelSet>;

    public:

        PHASTEllipseReconstructor(const InputGraphT &inputGraph,
                                  const CHEnvT &chEnv,
                                  const Fleet &fleet,
                                  const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                  const RequestState &requestState,
                                  const RouteState &routeState,
                                  CostCalculator &calc)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  fleet(fleet),
                  requestState(requestState),
                  routeState(routeState),
                  calc(calc),
                  downGraph(chEnv.getCH().downwardGraph()),
                  upGraph(chEnv.getCH().upwardGraph()),
                  sweepVertexPermutation(),
                  inverseSweepVertexPermutation(),
                  firstIdxOfLevel(),
                  query(ch, downGraph, upGraph, sweepVertexPermutation, firstIdxOfLevel,
                        ellipticBucketsEnv, routeState),
                  distanceFromVertexToNextStopPerThread(
                          [&]() { return std::vector<int>(inputGraph.numVertices(), INFTY); }),
                  p2pQuery(chEnv.template getFullCHQuery<P2PLabelSet>()),
                  pathUnpacker(chEnv.getCH()) {
            KASSERT(downGraph.numVertices() == upGraph.numVertices());
            const int numVertices = downGraph.numVertices();

            // We use a vertex order by CH level, allowing parallelization of vertex scans within each level.
            auto levels = computeSharedLevelsInUpAndDownGraph(upGraph, downGraph);
            std::vector<int> inversePerm(numVertices);
            std::iota(inversePerm.begin(), inversePerm.end(), 0);
            std::sort(inversePerm.begin(), inversePerm.end(), [&](const auto a, const auto b) {
                return levels[a] > levels[b] || (levels[a] == levels[b] && a < b);
            });
            inverseSweepVertexPermutation.assign(inversePerm.begin(), inversePerm.end());
            KASSERT(inverseSweepVertexPermutation.validate());
            sweepVertexPermutation = inverseSweepVertexPermutation.getInversePermutation();

            if constexpr (ParallelizeQuery) {
                // We set a threshold for large levels, s.t. only the large levels at the end will be processed in
                // parallel, ensuring there is enough work to make it worth it.
                int curLevel = INFTY;
                for (int i = 0; i < numVertices; ++i) {
                    if (levels[inversePerm[i]] == curLevel)
                        continue;
                    curLevel = levels[inversePerm[i]];
                    firstIdxOfLevel.push_back(i);
                }
                firstIdxOfLevel.push_back(numVertices);
                static constexpr int LARGE_LEVEL_THRESHOLD = (1 << 7) * CACHE_LINE_SIZE / sizeof(int);
                int firstLargeLevelIdx = 0;
                while (firstLargeLevelIdx < firstIdxOfLevel.size() - 1 &&
                       firstIdxOfLevel[firstLargeLevelIdx + 1] - firstIdxOfLevel[firstLargeLevelIdx] <
                       LARGE_LEVEL_THRESHOLD) {
                    ++firstLargeLevelIdx;
                }
                firstIdxOfLevel.erase(firstIdxOfLevel.begin(), firstIdxOfLevel.begin() + firstLargeLevelIdx);
            } else {
                // If queries are run sequentially, we place all vertices in one level, that will be considered small,
                // s.t. all vertices will be processed sequentially in the query without the overhead for parallel
                // structures.
                firstIdxOfLevel.push_back(numVertices);
            }

            // Permute the search graphs by the chosen permutation.
            downGraph.permuteVertices(sweepVertexPermutation);
            upGraph.permuteVertices(sweepVertexPermutation);

            query.init(); // Initialize query for vertex order, levels
        }

        // Given a set of stop IDs for pickup vehicles and dropoff vehicles, this computes the transfer points between
        // any pair of stops in the two sets. The given stop IDs should indicate the first stop in a pair of stops of
        // the respective vehicle.
        EdgeEllipseContainer
        computeEllipses(const std::vector<int> &stopIds, stats::EllipseReconstructionStats &stats) {

            Timer totalTimer;
            Timer timer;

            EdgeEllipseContainer container;
            container.idxOfStop.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);

            if (stopIds.empty())
                return container;

            int numStops = 0;
            for (const auto &stopId: stopIds) {
                if (container.idxOfStop[stopId] == INVALID_INDEX) {
                    container.idxOfStop[stopId] = numStops++;
                }
            }

            // Compute ellipses
            const size_t numEllipses = stopIds.size();

            // Differentiate between stops with and without leeway since the ellipse of stops without leeway is just
            // the shortest path between the stops (i.e. no detour allowed).
            std::vector<int> indicesWithoutLeeway;
            std::vector<int> indicesWithLeeway;
            for (int i = 0; i < numEllipses; ++i) {
                const int stopPos = routeState.stopPositionOf(stopIds[i]);
                const int vehId = routeState.vehicleIdOf(stopIds[i]);
                if (stopPos == routeState.numStopsOf(vehId) - 1) {
                    indicesWithoutLeeway.push_back(i);
                    continue;
                }
                const int legLength = time_utils::calcLengthOfLegStartingAt(stopPos, vehId, routeState);
                const int leeway = routeState.leewayOfLegStartingAt(stopIds[i]);
                if (leeway == legLength) {
                    indicesWithoutLeeway.push_back(i);
                } else {
                    indicesWithLeeway.push_back(i);
                }
            }

            KASSERT(numEllipses >= 0);
            container.edgeEllipses.resize(numEllipses);

            const auto initTime = timer.elapsed<std::chrono::nanoseconds>();


            // Construct ellipses without leeway by running point-to-point CH queries.
            computeEdgeEllipsesWithoutLeeway(stopIds, indicesWithoutLeeway, container, stats);

            // Construct ellipses with leeway by running ellipse reconstructor queries
            computeEdgeEllipsesWithLeeway(stopIds, indicesWithLeeway, container, stats);

            KASSERT(sanityCheckEdgeEllipses(stopIds, container.edgeEllipses));

            for (const auto &edgeEllipse: container.edgeEllipses) {
                KASSERT(std::is_sorted(edgeEllipse.begin(), edgeEllipse.end(),
                                       [](const EdgeInEllipse &e1, const EdgeInEllipse &e2) {
                                           return e1.edge < e2.edge;
                                       }));
            }

            int64_t sumSizesEllipsesWithoutLeeway = 0;
            for (const auto &idx: indicesWithoutLeeway) {
                sumSizesEllipsesWithoutLeeway += container.edgeEllipses[idx].size();
            }
            int64_t sumSizesEllipsesWithLeeway = 0;
            for (const auto &idx: indicesWithLeeway) {
                sumSizesEllipsesWithLeeway += container.edgeEllipses[idx].size();
            }

            stats.withoutLeewayNumEllipses += indicesWithoutLeeway.size();
            stats.withoutLeewaySumSizesEllipses += sumSizesEllipsesWithoutLeeway;
            stats.withLeewayNumEllipses += indicesWithLeeway.size();
            stats.withLeewaySumSizesEllipses += sumSizesEllipsesWithLeeway;
            stats.initTime += initTime;
            stats.withLeewayQuery_initTime += queryStats.initTime;
            stats.withLeewayQuery_topoSearchTime += queryStats.topoSearchTime;
            stats.withLeewayQuery_postprocessTime += queryStats.postprocessTime;

            return container;
        }

    private:

        void computeEdgeEllipsesWithoutLeeway(const std::vector<int> &stopIds,
                                              const std::vector<int> &indicesWithoutLeeway,
                                              EdgeEllipseContainer &container,
                                              stats::EllipseReconstructionStats &stats) {
            const size_t numEllipsesWithoutLeeway = indicesWithoutLeeway.size();
            std::vector<int> edgePathInInputGraph;
            std::vector<VertexInEllipse> vertexEllipse;
            // TODO: parallelize this loop
            Timer timer;
            for (int i = 0; i < numEllipsesWithoutLeeway; ++i) {
                timer.restart();
                // Reconstruct shortest path between stops by running point-to-point CH query.
                // TODO: what if there is more than one shortest path?
                const int stopId = stopIds[indicesWithoutLeeway[i]];
                const auto stopIdx = routeState.stopPositionOf(stopId);
                const int vehId = routeState.vehicleIdOf(stopId);
                const int numStops = routeState.numStopsOf(vehId);
                const auto &stopLocs = routeState.stopLocationsFor(vehId);

                KASSERT(container.idxOfStop[stopIds[indicesWithoutLeeway[i]]] == indicesWithoutLeeway[i]);
                auto &edgeEllipse = container.edgeEllipses[indicesWithoutLeeway[i]];

                if (stopIdx == numStops - 1) {
                    // Special case for last stops: Last stops only contain themselves in their ellipse
                    KASSERT(edgeEllipse.empty());
                    edgeEllipse.emplace_back(stopLocs[stopIdx], 0, 0);
                    stats.withoutLeewaySearchTime += timer.elapsed<std::chrono::nanoseconds>();
                    continue;
                }

                KASSERT(routeState.leewayOfLegStartingAt(stopId) ==
                        time_utils::calcLengthOfLegStartingAt(stopIdx, routeState.vehicleIdOf(stopId), routeState));
                const auto source = ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]));
                const auto target = ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]));
                p2pQuery.run(source, target);
                KASSERT(p2pQuery.getDistance() + inputGraph.travelTime(stopLocs[stopIdx + 1]) ==
                        routeState.leewayOfLegStartingAt(stopId));
                edgePathInInputGraph.clear();
                pathUnpacker.unpackUpDownPath(p2pQuery.getUpEdgePath(), p2pQuery.getDownEdgePath(),
                                              edgePathInInputGraph);

                // Add all vertices along path to ellipse, and sort vertices. Then convert back to edges (thus, all
                // edges that shortcut the path with equal distance will be included).
                vertexEllipse.clear();
                vertexEllipse.reserve(edgePathInInputGraph.size() + 1);
                int distFromFirstStopToHead = 0;
                int distFromHeadToSecondStop =
                        p2pQuery.getDistance() + inputGraph.travelTime(stopLocs[stopIdx + 1]);
                vertexEllipse.emplace_back(inputGraph.edgeHead(stopLocs[stopIdx]), distFromFirstStopToHead,
                                           distFromHeadToSecondStop);
                for (const auto e: edgePathInInputGraph) {
                    distFromFirstStopToHead += inputGraph.travelTime(e);
                    distFromHeadToSecondStop -= inputGraph.travelTime(e);
                    const auto v = inputGraph.edgeHead(e);
                    vertexEllipse.emplace_back(v, distFromFirstStopToHead, distFromHeadToSecondStop);
                }
                KASSERT(vertexEllipse.front().vertex == inputGraph.edgeHead(stopLocs[stopIdx]));
                KASSERT(vertexEllipse.back().vertex == inputGraph.edgeTail(stopLocs[stopIdx + 1]));

                stats.withoutLeewaySearchTime += timer.elapsed<std::chrono::nanoseconds>();

                timer.restart();
                std::sort(vertexEllipse.begin(), vertexEllipse.end(),
                          [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                              return v1.vertex < v2.vertex;
                          });

                // Convert ellipse of vertices to ellipse of edges.
                convertVertexEllipseIntoEdgeEllipse(vertexEllipse, routeState.leewayOfLegStartingAt(stopId),
                                                    edgeEllipse, distanceFromVertexToNextStopPerThread.local());
                stats.withoutLeewayConvertToEdgesTime += timer.elapsed<std::chrono::nanoseconds>();
            }
        }

        void computeEdgeEllipsesWithLeeway(const std::vector<int> &stopIds,
                                           const std::vector<int> &indicesWithLeeway,
                                           EdgeEllipseContainer &container,
                                           stats::EllipseReconstructionStats &stats) {
            queryStats.reset();

            // Construct ellipses with leeway by running topological downward sweep in CH.
            Timer timer;
            const size_t numEllipsesWithLeeway = indicesWithLeeway.size();
            const size_t numBatchesWithLeeway = numEllipsesWithLeeway / K + (numEllipsesWithLeeway % K != 0);
            std::vector<std::vector<VertexInEllipse>> vertexEllipses(numEllipsesWithLeeway);
            for (int i = 0; i < numBatchesWithLeeway; ++i) {

                std::array<int, K> batchStopIds;
                DistanceLabel leeways;
                int numEllipsesInBatch = 0;
                for (int j = 0; j < K && i * K + j < numEllipsesWithLeeway; ++j) {
                    batchStopIds[j] = stopIds[indicesWithLeeway[i * K + j]];
                    leeways[j] = routeState.leewayOfLegStartingAt(batchStopIds[j]);
                    ++numEllipsesInBatch;
                }

                auto firstEllipseInBatch = vertexEllipses.begin() + i * K;
                query.run(batchStopIds, leeways, numEllipsesInBatch, firstEllipseInBatch, queryStats);
            }

            stats.withLeewaySearchTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.withLeewayQuery_numVerticesSettled += queryStats.numVerticesSettled;
            stats.withLeewayQuery_numEdgesRelaxed += queryStats.numEdgesRelaxed;

            // Convert vertex ellipses to edge ellipses
            timer.restart();
            tbb::parallel_for(0ul, numEllipsesWithLeeway, [&](const auto i) {
                auto &vertexEllipse = vertexEllipses[i];
                for (auto &v: vertexEllipse)
                    v.vertex = ch.contractionOrder(
                            inverseSweepVertexPermutation[v.vertex]); // Revert to vertex IDs in inputGraph
                std::sort(vertexEllipse.begin(), vertexEllipse.end(),
                          [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                              return v1.vertex < v2.vertex;
                          });
                const auto leeway = routeState.leewayOfLegStartingAt(stopIds[indicesWithLeeway[i]]);
                KASSERT(container.idxOfStop[stopIds[indicesWithLeeway[i]]] == indicesWithLeeway[i]);
                auto &edgeEllipse = container.edgeEllipses[indicesWithLeeway[i]];
                convertVertexEllipseIntoEdgeEllipse(vertexEllipse, leeway, edgeEllipse,
                                                    distanceFromVertexToNextStopPerThread.local());
            });
            stats.withLeewayConvertToEdgesTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        // Vertex ellipse must be given using original input graph vertex IDs.
        // Edges in output are edge IDs in original input graph.
        // Vertices should be sorted so that edges are sorted due to sorted inputGraph.
        void convertVertexEllipseIntoEdgeEllipse(const std::vector<VertexInEllipse> &vertexEllipse,
                                                 const int leeway,
                                                 std::vector<EdgeInEllipse> &edgeEllipse,
                                                 std::vector<int> &localDistanceFromVertexToNextStop) const {
            edgeEllipse.reserve(vertexEllipse.size() * 2);

            KASSERT(std::all_of(localDistanceFromVertexToNextStop.begin(), localDistanceFromVertexToNextStop.end(),
                                [](const int d) { return d == INFTY; }));

            for (const auto &vertexInEllipse: vertexEllipse)
                localDistanceFromVertexToNextStop[vertexInEllipse.vertex] = vertexInEllipse.distFromVertex;

            for (const auto &vertexInEllipse: vertexEllipse) {

                FORALL_INCIDENT_EDGES(inputGraph, vertexInEllipse.vertex, e) {

                    const int travelTime = inputGraph.travelTime(e);
                    const int edgeHead = inputGraph.edgeHead(e);
                    const int distToTail = vertexInEllipse.distToVertex;
                    const int distFromHead = localDistanceFromVertexToNextStop[edgeHead];

                    if (distToTail + travelTime + distFromHead <= leeway) {
                        edgeEllipse.emplace_back(e, distToTail, distFromHead);
                    }
                }
            }

            // Reset distances
            for (const auto &vertexInEllipse: vertexEllipse)
                localDistanceFromVertexToNextStop[vertexInEllipse.vertex] = INFTY;

            KASSERT(std::is_sorted(edgeEllipse.begin(), edgeEllipse.end(),
                                   [](const EdgeInEllipse &e1, const EdgeInEllipse &e2) {
                                       return e1.edge < e2.edge;
                                   }));
        }

        bool sanityCheckEdgeEllipses(const std::vector<int> &stopIds,
                                     const std::vector<std::vector<EdgeInEllipse>> &ellipses) {
            std::vector<int> edgePathInInputGraph;
            for (int i = 0; i < stopIds.size(); ++i) {
                const auto &ellipse = ellipses[i];

                // Make sure ellipse is sorted by increasing vertex ID
                KASSERT(std::is_sorted(ellipse.begin(), ellipse.end(),
                                       [](const EdgeInEllipse &e1, const EdgeInEllipse &e2) {
                                           return e1.edge < e2.edge;
                                       }));

                if constexpr (TOP_VERTICES_DIVISOR == 1) {

                    const auto stopId = stopIds[i];
                    const auto stopIdx = routeState.stopPositionOf(stopId);
                    const auto vehId = routeState.vehicleIdOf(stopId);
                    const auto stopLocs = routeState.stopLocationsFor(vehId);

                    // Make sure ellipse at least contains edges on shortest path

                    // If the stop is the last stop of the vehicle, the ellipse should only contain the stop itself.
                    if (stopIdx == routeState.numStopsOf(vehId) - 1) {
                        KASSERT(ellipse.size() == 1);
                        KASSERT(ellipse[0].edge == stopLocs[stopIdx]);
                        continue;
                    }

                    // If stopId is at position 0 in the route and the stop at position 1 has just been inserted as an
                    // intermediate stop, we never generated target entries for stop at position 0 so the ellipse will be
                    // empty. This is okay, since the vehicle is already at this intermediate stop so we can ignore the
                    // ellipse.
                    if (stopIdx == 0 &&
                        routeState.schedArrTimesFor(vehId)[1] == requestState.originalRequest.requestTime)
                        continue;

                    const auto source = ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]));
                    const auto target = ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]));
                    p2pQuery.run(source, target);
                    edgePathInInputGraph.clear();
                    pathUnpacker.unpackUpDownPath(p2pQuery.getUpEdgePath(), p2pQuery.getDownEdgePath(),
                                                  edgePathInInputGraph);

                    for (const auto &eInInputGraph: edgePathInInputGraph) {
                        if (std::find_if(ellipse.begin(), ellipse.end(), [&](const EdgeInEllipse &e) {
                            return e.edge == eInInputGraph;
                        }) == ellipse.end()) {
                            KASSERT(false);
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        // Takes CH search graphs ordered by increasing rank (as in CH) and assigns a level to each vertex, s.t.
        // level[v] > level[u] for all upward and reverse downward edges (u, v).
        static std::vector<int> computeSharedLevelsInUpAndDownGraph(const CH::SearchGraph &forwardUpGraph,
                                                                    const CH::SearchGraph &reverseDownGraph) {
            const auto numVertices = forwardUpGraph.numVertices();
            KASSERT(reverseDownGraph.numVertices() == numVertices);
            std::vector<int> level(numVertices, 0);

            // Traverse vertices in increasing order, updating levels of upper neighbors for each vertex
            for (int u = 0; u < numVertices; ++u) {
                FORALL_INCIDENT_EDGES(forwardUpGraph, u, e) {
                    const auto v = forwardUpGraph.edgeHead(e);
                    level[v] = std::max(level[v], level[u] + 1);
                }
                FORALL_INCIDENT_EDGES(reverseDownGraph, u, e) {
                    const auto v = reverseDownGraph.edgeHead(e);
                    level[v] = std::max(level[v], level[u] + 1);
                }
            }

            return level;
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const Fleet &fleet;
        const RequestState &requestState;
        const RouteState &routeState;
        CostCalculator& calc;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.

        Permutation sweepVertexPermutation; // Permutes vertex ranks in order to linearize top-down passes.
        Permutation inverseSweepVertexPermutation;
        std::vector<int> firstIdxOfLevel;

        // Permutation that maps edge IDs in permuted input graph to edge IDs in original input graph.
        Permutation ellipseEdgeIdsToOriginalEdgeIds;

        Query query;
        EllipseReconstructorStats queryStats;
        tbb::enumerable_thread_specific<std::vector<int>> distanceFromVertexToNextStopPerThread;

        P2PQuery p2pQuery;
        CHPathUnpacker pathUnpacker;
    };

} // karri