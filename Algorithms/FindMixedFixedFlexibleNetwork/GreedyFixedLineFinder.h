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

#include <vector>
#include "DataStructures/Containers/TimestampedVector.h"
#include "FixedLine.h"
#include "Request.h"
#include "DataStructures/Graph/Graph.h"
#include <kassert/kassert.hpp>
#include "Tools/custom_assertion_levels.h"
#include "DataStructures/Containers/Subset.h"
#include "PickupDropoffManager.h"
#include "OverlappingPaths.h"
#include "InputConfig.h"
#include "Tools/Timer.h"
#include "DataStructures/Containers/FastResetFlagArray.h"

#include <nlohmann/json.hpp>

namespace mixfix {

    // Given a set of OD-pairs and paths between each origin and destination in a road network, this facility
    // constructs a set of fixed lines with the goal of serving as many passengers as possible, i.e. s.t. the
    // number of OD-pairs (o,d) for which both o and d lie in the vicinity of at least one single line is maximized.
    //
    // This greedy algorithm is based on Andres Fielbaum, Javier Alonso-Mora, "Design of mixed fixed-flexible bus
    // public transport networks by tracking the paths of on-demand vehicles", Transportation Research Part C: Emerging
    // Technologies, 2024, https://doi.org/10.1016/j.trc.2024.104580.
    template<typename VehicleInputGraphT,
            typename PreliminaryPathsT,
            AvoidLoopsStrategy AvoidLoopsStrat,
            typename OverviewLoggerT = NullLogger>
    class GreedyFixedLineFinder {

        struct RunningTimePerLineStats {
            uint64_t constructLineTime = 0;
            uint64_t updatePathsTime = 0;
        };

    public:

        GreedyFixedLineFinder(const VehicleInputGraphT &inputGraph, const VehicleInputGraphT &reverseGraph,
                              const PickupDropoffInfo &pdInfo, const std::vector<Request> &requests)
                : inputGraph(inputGraph), reverseGraph(reverseGraph),
                  pdInfo(pdInfo), requests(requests), inputConfig(InputConfig::getInstance()),
                  residualFlow(inputGraph.numEdges(), 0),
                  lineOverviewLogger(LogManager<OverviewLoggerT>::getLogger("lines.csv",
                                                                            "line_id,"
                                                                            "initial_edge,"
                                                                            "max_flow,"
                                                                            "num_edges,"
                                                                            "vehicle_travel_time,"
                                                                            "num_subpaths_covered,"
                                                                            "num_rider_edges,"
                                                                            "sum_rider_travel_time,"
                                                                            "construct_line_time,"
                                                                            "update_paths_time\n")) {}

        // Finds fixed lines given preliminary rider paths.
        // Each path is expected to be a sequence of edges in the network.
        // Returns pairs of line and according total rider travel time covered by the line.
        std::vector<FixedLine> findFixedLines(PreliminaryPathsT &paths) {

            std::vector<FixedLine> lines;

            std::vector<int> overlapsWithLine;
            OverlappingPaths globalOverlaps;
            FixedLine line;

            Timer timer;
            while (!paths.empty()) {
                std::cout << "\n\n";

                RunningTimePerLineStats runningTimeStats;

                // Construct line:
                timer.restart();
                int initialEdge, maxFlowOnLine;
                line.clear();
                overlapsWithLine.clear();
                globalOverlaps.init(paths.getMaxPathId());
                findInitialEdge(paths, initialEdge, maxFlowOnLine);
                std::cout << "Initial edge " << initialEdge
                          << " (tail: " << inputGraph.osmNodeId(inputGraph.edgeTail(initialEdge))
                          << ", head: " << inputGraph.osmNodeId(inputGraph.edgeHead(initialEdge))
                          << ") with flow " << maxFlowOnLine << std::endl;
                if (maxFlowOnLine < inputConfig.minMaxFlowOnLine) {
                    std::cout << "Stopping line construction due to insufficient flow on initial edge." << std::endl;
                    break;
                }
                constructNextLine(paths, line, initialEdge, maxFlowOnLine, globalOverlaps, overlapsWithLine);
                runningTimeStats.constructLineTime = timer.elapsed<std::chrono::nanoseconds>();


                std::cout << "Found a line of length " << line.size() << " that overlaps " << overlapsWithLine.size()
                          << " paths." << std::endl;

                // Log line:
                logLine(line, lines.size(), initialEdge, maxFlowOnLine, overlapsWithLine, globalOverlaps, paths, runningTimeStats);

                // Remove paths that overlap with line:
                // TODO: add subpaths that are not covered back as new paths
                for (const auto& pathId : overlapsWithLine) {
                    paths.removePath(pathId);
                }

                // Add line
                lines.emplace_back(std::move(line));
            }

            return lines;
        }


    private:

        using Path = typename PreliminaryPathsT::Path;

        void findInitialEdge(const PreliminaryPathsT &paths, int &initialEdge, int &flowOnInitialEdge) {

            // flowOnInitialEdge may be larger than the number of paths if one paths visits maxFlowEdge multiple times.
            residualFlow.clear();
            flowOnInitialEdge = 0;
            initialEdge = INVALID_EDGE;
            for (const auto &path: paths) {
                for (const auto &e: path) {
                    ++residualFlow[e];
                    if (residualFlow[e] > flowOnInitialEdge) {
                        initialEdge = e;
                        flowOnInitialEdge = residualFlow[e];
                    }
                }
            }
        }

        // Greedily constructs new line from remaining paths. Returns line and max flow on line.
        void constructNextLine(PreliminaryPathsT &paths, FixedLine &line,
                               const int &initialEdge, const int &flowOnInitialEdge,
                               OverlappingPaths &globalOverlaps,
                               std::vector<int> &overlapsWithLine) {

            line = {initialEdge};
            const auto pickupsAtTail = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(initialEdge));
            const auto dropoffsAtHead = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(initialEdge));
            for (const auto &path: paths) {
                KASSERT(!globalOverlaps.hasKnownOverlap(path.getPathId()));
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == initialEdge) {
                        const bool possiblePickupReached = contains(pickupsAtTail.begin(), pickupsAtTail.end(),
                                                                    path.getPathId());
                        const bool possibleDropoffReached = contains(dropoffsAtHead.begin(), dropoffsAtHead.end(),
                                                                     path.getPathId());
                        globalOverlaps.initializeOverlap(path.getPathId(), i, i + 1, possiblePickupReached,
                                                         possibleDropoffReached);
                        overlapsWithLine.push_back(path.getPathId());
                        break; // If path overlaps with edge in multiple places, only consider first overlap
                    }
                }
            }
            const auto numOverlapsOnInitialEdge = overlapsWithLine.size();

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.

            // Extend forward first. All initial overlapping paths that do not have a reachable dropoff are active.
            std::vector<int> activeOverlaps;
            for (int i = 0; i < numOverlapsOnInitialEdge; ++i) {
                const auto &p = overlapsWithLine[i];
                if (!globalOverlaps.getOverlapFor(p).reachesEndOfPath)
                    activeOverlaps.push_back(p);
            }
            extendLineForwards(line, flowOnInitialEdge, globalOverlaps, overlapsWithLine, activeOverlaps, paths);


            // Extend backward. All initial overlapping paths that do not have a reachable pickup are active.
            activeOverlaps.clear();
            for (int i = 0; i < numOverlapsOnInitialEdge; ++i) {
                const auto &p = overlapsWithLine[i];
                if (!globalOverlaps.getOverlapFor(p).reachesBeginningOfPath)
                    activeOverlaps.push_back(p);
            }
            extendLineBackwards(line, flowOnInitialEdge, globalOverlaps, overlapsWithLine, activeOverlaps, paths);
        }

        static inline uint64_t computeScoreForOverlapLength(const uint64_t overlapLength) {
            const auto res = std::pow(overlapLength, InputConfig::getInstance().overlapScoreExponent);
            return static_cast<uint64_t>(res);
        }

        // Finds the highest scoring extension starting at given vertex.
        // Ignores edges in given blacklist.
        // Returns extension edge and flow on extension or {INVALID_EDGE, 0} if no viable extension exists.
        template<
                typename DoesExtensionContinueOverlapT,
                typename FindOverlapsStartingAtExtensionT>
        static void findMaxScoreExtension(int &extension,
                                          int &flowOnExtension,
                                          std::vector<std::pair<int, int>> &startingOverlaps,
                                          const VehicleInputGraphT &graph,
                                          const int v,
                                          const std::vector<int> &activeOverlaps,
                                          const DoesExtensionContinueOverlapT &doesExtensionContinueOverlap,
                                          const FindOverlapsStartingAtExtensionT &findOverlapsStartingAtExtension,
                                          const std::vector<int> &edgeBlackList = {}) {
            // Search for edge on which to extend line. Edge is chosen using a scoring system.
            extension = INVALID_EDGE;
            flowOnExtension = 0;
            uint64_t maxScore = 0;
            FORALL_INCIDENT_EDGES(graph, v, e) {
                if (contains(edgeBlackList.begin(), edgeBlackList.end(), e))
                    continue;

                uint64_t score = 0; // Each path that overlaps with the edge contributes to the score as described below
                int flow = 0; // Each path that overlaps with the edge contributes to the load with a value of one

                int eInForwGraph = graph.edgeId(e);

                // Paths already on line whose next edge is e increase the score by 1 + the square of the length of
                // the current overlap between the line and the path.
                for (const auto &pathId: activeOverlaps) {
                    int currentOverlapLength = -1;
                    if (doesExtensionContinueOverlap(pathId, eInForwGraph, currentOverlapLength)) {
                        const uint64_t overlapLength = 1 + currentOverlapLength;
                        score += computeScoreForOverlapLength(overlapLength);
                        ++flow;
                    }
                }


                // Paths that begin at e increase the score by 1.
                const auto startingOverlapsOnE = findOverlapsStartingAtExtension(eInForwGraph);
                score += startingOverlapsOnE.size();
                flow += startingOverlapsOnE.size();

                if (score > maxScore) {
                    maxScore = score;
                    extension = e;
                    flowOnExtension = flow;
                    startingOverlaps = startingOverlapsOnE;
                }
            }
        }

        template<
                typename DoesExtensionContinueOverlapT,
                typename FindOverlapsStartingAtExtensionT,
                typename ExtendOverlapIfPossibleT,
                typename CommitStartingOverlapsT
        >
        void extendLine(FixedLine &line,
                        const int &maxFlowOnLine,
                        const PreliminaryPathsT &paths,
                        const VehicleInputGraphT &graph,
                        std::vector<int> &activeOverlaps,
                        const DoesExtensionContinueOverlapT &doesExtensionContinueOverlap,
                        const FindOverlapsStartingAtExtensionT &findOverlapsStartingAtExtension,
                        const ExtendOverlapIfPossibleT &extendOverlapIfPossible,
                        const CommitStartingOverlapsT &commitStartingOverlaps) {

            // Begin extending
            int mostRecentUsefulVertex = graph.edgeHead(line.back());
            double flowDif = 0.0;
            std::vector<std::pair<int, int>> startingOverlaps; // Overlaps starting on extension [request ID, index of edge in path of request]
            while (flowDif < inputConfig.maxFlowRatioOnLine && !activeOverlaps.empty()) {
                const auto v = graph.edgeHead(line.back());

                int extension, flowOnExtension;
                startingOverlaps.clear();
                if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::NONE) {
                    // Allow loops on lines
                    findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, activeOverlaps,
                                          doesExtensionContinueOverlap, findOverlapsStartingAtExtension);
                } else if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::VERTEX
                                     || AvoidLoopsStrat == AvoidLoopsStrategy::VERTEX_ROLLBACK) {
                    // Stop extending if current vertex has been visited before
                    bool vertexVisitedBefore = false;
                    for (const auto &e: line) {
                        if (v == graph.edgeTail(e)) {
                            vertexVisitedBefore = true;
                            break;
                        }
                    }
                    if (vertexVisitedBefore) {
                        if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::VERTEX_ROLLBACK) {
                            // Immediately roll line back to last useful vertex.
                            while (graph.edgeHead(line.back()) != mostRecentUsefulVertex) {
                                line.pop_back();
                            }
                            KASSERT(!line.empty());
                        }
                        break;
                    }

                    findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, activeOverlaps,
                                          doesExtensionContinueOverlap, findOverlapsStartingAtExtension);

                } else if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::EDGE
                                     || AvoidLoopsStrat == AvoidLoopsStrategy::EDGE_ROLLBACK) {
                    // Stop extending if chosen extension edge has been visited before

                    findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, activeOverlaps,
                                          doesExtensionContinueOverlap, findOverlapsStartingAtExtension);

                    bool edgeVisitedBefore = false;
                    for (const auto &e: line) {
                        if (e == extension) {
                            edgeVisitedBefore = true;
                            break;
                        }
                    }

                    if (edgeVisitedBefore) {
                        if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::VERTEX_ROLLBACK) {
                            // Immediately roll line back to last useful vertex.
                            while (graph.edgeHead(line.back()) != mostRecentUsefulVertex) {
                                line.pop_back();
                            }
                            KASSERT(!line.empty());
                        }
                        extension = INVALID_EDGE;
                        break;
                    }
                } else { // inputConfig.avoidLoopsStrategy == AvoidLoopsStrategy::EDGE_ALTERNATIVE
                    // Choose extension that avoids loop if one exists.
                    std::vector<int> extensionBlackList;
                    extensionBlackList.reserve(graph.degree(v));
                    while (true) {

                        findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, activeOverlaps,
                                              doesExtensionContinueOverlap, findOverlapsStartingAtExtension);

                        // If no viable extension is available, stop extending line.
                        if (extension == INVALID_EDGE)
                            break;

                        KASSERT(flowOnExtension <= maxFlowOnLine, "Larger flow than max flow has been found!",
                                kassert::assert::light);

                        // If extension edge is already part of line, choose different extension (if possible)
                        bool extensionClosesLoop = false;
                        for (const auto &e: line) {
                            if (e == extension) {
                                extensionClosesLoop = true;
                                break;
                            }
                        }
                        if (!extensionClosesLoop)
                            break; // Viable extension has been found

                        // Blacklist extension and try again
                        extensionBlackList.push_back(extension);
                    }
                }


                // If no viable extension is available, stop extending line.
                if (extension == INVALID_EDGE)
                    break;

                KASSERT(flowOnExtension <= maxFlowOnLine, "Larger flow than max flow has been found!",
                        kassert::assert::light);

                // TODO update name of parameter?
                if (flowOnExtension < inputConfig.minMaxFlowOnLine)
                    break;

                // Fielbaum and Alonso-Mora give this formula for the flow difference. The given intention is to stop
                // extending once the difference in flow becomes too large since this would imply bad utilization of
                // vehicle capacity in parts of the line. However, a line is started on the edge with most flow so
                // flowOnExtension will become smaller over time which means flowDif also becomes smaller over time.
                // Thus, this stopping criterion is never met. Therefore, we use the formula below instead.
                // flowDif = static_cast<double>(flowOnExtension) /
                //           static_cast<double>(std::max(maxFlowOnLine, flowOnExtension));

                // If flow difference (i.e. difference between least and most flow) has become too large, stop extending
                flowDif = static_cast<double>(maxFlowOnLine) /
                          static_cast<double>(flowOnExtension);

                if (flowDif >= inputConfig.maxFlowRatioOnLine)
                    break; // Stop extending line

                const int extensionInForwGraph = graph.edgeId(extension);

                // Remove passengers whose paths do not overlap with extension
                int i = 0;
                while (i < activeOverlaps.size()) {
                    auto &o = activeOverlaps[i];
                    const auto &path = paths.getPathFor(o);
                    if (extendOverlapIfPossible(o, path, extensionInForwGraph, mostRecentUsefulVertex)) {
                        ++i;
                    } else {
                        // If overlapping path does not overlap with extension or a dropoff has been found,
                        // remove from overlapping paths.
                        std::swap(activeOverlaps[i], activeOverlaps.back());
                        activeOverlaps.pop_back();
                    }
                }

                // Add overlaps starting with extension
                commitStartingOverlaps(activeOverlaps, startingOverlaps, extensionInForwGraph);

                // Add extension
                line.push_back(extension);
            }
        }

        void extendLineForwards(FixedLine &line,
                                const int &maxFlowOnLine,
                                OverlappingPaths &globalOverlaps,
                                std::vector<int> &overlapsWithLine,
                                std::vector<int> &activeOverlaps,
                                const PreliminaryPathsT &paths) {


            // Hook function for checking if an extension continues an overlap with a path when deciding next extension.
            static auto doesExtensionContinueOverlap = [&globalOverlaps, &paths](const int &pathId,
                                                                                 const int extension,
                                                                                 int &currentOverlapLength) {
                const auto &o = globalOverlaps.getOverlapFor(pathId);
                const auto &path = paths.getPathFor(pathId);
                KASSERT(o.end < path.size());
                const bool extensionContinuesOverlap = path[o.end] == extension;
                if (extensionContinuesOverlap) {
                    currentOverlapLength = o.end - o.start;
                    return true;
                }
                currentOverlapLength = -1;
                return false;
            };

            // Hook function for finding new overlaps when deciding next extension.
            auto findOverlapsStartingAtExtension = [this, &globalOverlaps, &paths](const int extension) {

                std::vector<std::pair<int, int>> startingOverlaps;

                // There is a new overlap for request r if r can be picked up at the currently last vertex of the line
                // (the tail vertex of the extension edge) and the preliminary path of r visits this vertex.
                const auto &pickups = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension));
                for (const auto &pathId: pickups) {

                    // If request has already been answered, ignore it
                    if (!paths.hasPathFor(pathId))
                        continue;

                    // If we already know an earlier overlap of the path with the line, we only keep the earlier one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (globalOverlaps.hasKnownOverlap(pathId))
                        continue;

                    // Check if extension edge lies on the path for reqId:
                    const auto &path = paths.getPathFor(pathId);
                    int i = 0;
                    while (i < path.size() && path[i] != extension)
                        ++i;
                    if (i == path.size())
                        continue;

                    startingOverlaps.push_back({pathId, i});
                }

                return startingOverlaps;
            };

            // Hook function for extending an overlap if possible for a chosen extension.
            // Returns true if overlap has been extended and should be kept as overlapping or false if it
            // should be removed.
            const auto extendOverlapIfPossible = [this, &globalOverlaps](
                    const int pathId,
                    const Path &path,
                    const int extension,
                    int &mostRecentUsefulVertex) {
                auto &o = globalOverlaps.getOverlapFor(pathId);
                const int newReachedVertex = inputGraph.edgeHead(extension);
                const auto &dropoffsAtNewVertex = pdInfo.getPossibleDropoffsAt(newReachedVertex);
                KASSERT(o.end < path.size());
                if (path[o.end] != extension)
                    return false;
                ++o.end;
                if (!contains(dropoffsAtNewVertex.begin(), dropoffsAtNewVertex.end(), path.getPathId())) {
                    // If path continues on extension but no dropoff is reached yet, keep it as overlapping path
                    return true;
                }
                o.reachesEndOfPath = true;
                mostRecentUsefulVertex = newReachedVertex;
                return false;
            };

            // Hook function for starting an overlap if possible for a chosen extension.
            // Each starting overlap for the extension found by findStartingOverlapsAtExtension is added as an
            // overlapping path. Starting overlaps of requests for which the head of the extension is a valid dropoff
            // are instead added to fully covered paths.
            const auto commitStartingOverlaps = [this, &globalOverlaps, &overlapsWithLine](
                    std::vector<int> &activeOverlaps,
                    const std::vector<std::pair<int, int>> &startingOverlaps,
                    const int extension) {
                const auto &dropoffsAtNewVertex = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extension));

                for (const auto &[pathId, startEdgeIdx]: startingOverlaps) {
                    KASSERT(contains(pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension)).begin(),
                                     pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension)).end(),
                                     pathId),
                            "Starting overlap does not have a pickup at the tail of the extension.");
                    const bool reachesEndOfPath = contains(dropoffsAtNewVertex.begin(), dropoffsAtNewVertex.end(),
                                                           pathId);
                    globalOverlaps.initializeOverlap(pathId, startEdgeIdx, startEdgeIdx + 1, true, reachesEndOfPath);
                    overlapsWithLine.push_back(pathId);
                    if (!reachesEndOfPath)
                        activeOverlaps.push_back(pathId);
                }
            };

            // Execute line extension with the forward extension hooks
            extendLine(line, maxFlowOnLine, paths, inputGraph, activeOverlaps, doesExtensionContinueOverlap,
                       findOverlapsStartingAtExtension, extendOverlapIfPossible, commitStartingOverlaps);
        }


        void extendLineBackwards(FixedLine &line,
                                 const int &maxFlowOnLine,
                                 OverlappingPaths &globalOverlaps,
                                 std::vector<int> &overlapsWithLine,
                                 std::vector<int> &activeOverlaps,
                                 const PreliminaryPathsT &paths) {

            // Hook function for checking if an extension continues an overlap with a path when deciding next extension.
            static auto doesExtensionContinueOverlap = [&globalOverlaps, &paths](const int &pathId,
                                                                                 const int extensionInForw, int& currentOverlapLength) {
                const auto &o = globalOverlaps.getOverlapFor(pathId);
                const auto &path = paths.getPathFor(pathId);
                KASSERT(o.start > 0);
                const bool extensionContinuesOverlap = path[o.start - 1] == extensionInForw;
                if (extensionContinuesOverlap)  {
                    currentOverlapLength = o.end - o.start;
                    return true;
                }
                currentOverlapLength = -1;
                return false;
            };

            // Hook function for checking if an extension starts an overlap with a path when deciding next extension.
            static auto findOverlapsStartingAtExtension = [this, &globalOverlaps, &paths](const int extensionInForw) {

                std::vector<std::pair<int, int>> startingOverlaps;

                // There is a new overlap for request r if r can be dropped off at the currently last vertex of the line
                // (the head vertex of the extension edge) and the preliminary path of r visits this vertex.
                const auto &dropoffs = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extensionInForw));
                for (const auto &pathId: dropoffs) {

                    // If request has already been answered, ignore it
                    if (!paths.hasPathFor(pathId))
                        continue;

                    // If we already know a later overlap of the path with the line, we only keep the later one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (globalOverlaps.hasKnownOverlap(pathId))
                        continue;

                    // Check if extension edge lies on the path for reqId:
                    const auto &path = paths.getPathFor(pathId);
                    int i = path.size() - 1;
                    while (i >= 0 && path[i] != extensionInForw)
                        --i;
                    if (i == -1)
                        continue;

                    startingOverlaps.push_back({pathId, i});
                }

                return startingOverlaps;
            };

            // Hook function for extending an overlap if possible for a chosen extension.
            // Returns true if overlap has been extended and should be kept as overlapping or false if it
            // should be removed.
            const auto extendOverlapIfPossible = [this, &globalOverlaps](
                    const int pathId,
                    const Path &path,
                    const int extensionInForw,
                    int &mostRecentUsefulVertex) {
                auto &o = globalOverlaps.getOverlapFor(pathId);
                KASSERT(o.start > 0);
                const auto newReachedVertex = inputGraph.edgeTail(extensionInForw);
                const auto &pickupsAtNewVertex = pdInfo.getPossiblePickupsAt(newReachedVertex);
                if (path[o.start - 1] != extensionInForw)
                    return false;

                --o.start;
                if (!contains(pickupsAtNewVertex.begin(), pickupsAtNewVertex.end(), pathId)) {
                    // If path continues on extension but no possible pickup has been reached, keep it as overlapping path
                    return true;
                }
                o.reachesBeginningOfPath = true;
                mostRecentUsefulVertex = newReachedVertex;
                return false;
            };

            // Hook function for starting overlaps for a chosen extension.
            // If extension starts an overlap with the path, we do not know an overlap already, and the path is not
            // fully covered right away, the overlap is added to overlapping..
            const auto commitStartingOverlaps = [this, &globalOverlaps, &overlapsWithLine](
                    std::vector<int> &activeOverlaps,
                    const std::vector<std::pair<int, int>> &startingOverlaps,
                    const int extensionInForw) {
                const auto &pickupsAtNewVertex = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extensionInForw));

                for (const auto &[pathId, lastEdgeIdx]: startingOverlaps) {
                    KASSERT(contains(pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extensionInForw)).begin(),
                                     pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extensionInForw)).end(),
                                     pathId),
                            "Starting overlap does not have a dropoff at the head of the extension.");

                    const bool reachesBegOfPath = contains(pickupsAtNewVertex.begin(), pickupsAtNewVertex.end(),
                                                           pathId);
                    globalOverlaps.initializeOverlap(pathId, lastEdgeIdx, lastEdgeIdx + 1, reachesBegOfPath, true);
                    overlapsWithLine.push_back(pathId);
                    if (!reachesBegOfPath)
                        activeOverlaps.push_back(pathId);
                }
            };


            // Get reverse representation of line: Order of edges is reversed and edges are transformed to edges in
            // reverseGraph.
            std::reverse(line.begin(), line.end());
            for (auto &e: line) {
                bool found = false;
                FORALL_INCIDENT_EDGES(reverseGraph, inputGraph.edgeHead(e), eInRev) {
                    if (reverseGraph.edgeId(eInRev) == e) {
                        e = eInRev;
                        found = true;
                        break;
                    }
                }
                KASSERT(found, "No reverse edge found for " << e << "!", kassert::assert::light);
            }

            // Execute line extension with the backward extension hooks
            extendLine(line, maxFlowOnLine, paths, reverseGraph, activeOverlaps, doesExtensionContinueOverlap,
                       findOverlapsStartingAtExtension, extendOverlapIfPossible, commitStartingOverlaps);

            // Convert line back into forwards representation
            std::reverse(line.begin(), line.end());
            for (auto &e: line)
                e = reverseGraph.edgeId(e);
        }

//
//        struct Pickupable {
//            int requestId;
//            int pickupVertexIdx;
//            int walkingTime;
//        };
//
//        template<typename IsPassengerAlreadyServedT>
//        std::vector<ServedRequest>
//        findPassengersServableByLine(const FixedLine &line,
//                                     const std::vector<int> &verticesInLine,
//                                     const IsPassengerAlreadyServedT &isPaxAlreadyServedByOtherLine,
//                                     const std::vector<int> &,
//                                     BitVector &servedByThisLine,
//                                     std::vector<int> &pickupableIdx,
//                                     Subset &tripTimeViolators) const {
//            KASSERT(servedByThisLine.cardinality() == 0);
//            KASSERT(std::all_of(pickupableIdx.begin(), pickupableIdx.end(),
//                                [](const int idx) { return idx == INVALID_INDEX; }));
//
//            // We iterate over the vertices of the line from front to back, keeping track of a subset of requests P that
//            // may be picked up by the vehicle before the current vertex.
//            // When reaching vertex i, we perform the following:
//            //  1. Remove a request r from P if the detour of r has become too large with vertex i.
//            //  2. For every r in P, check whether r can be dropped off at vertex i. If so, r can be served by the line. Remove r from P.
//            //  3. Add all unserved requests that can be picked up at vertex i to P.
//            //
//            // There may be multiple possible pickup vertices on the same line for a request r. We always use the one with
//            // the smallest travel time to the current vertex i, i.e. walking distance + current in-vehicle distance.
//
//            // Build prefix sum of travel time along line for fast retrieval of travel time between any two vertices:
//            std::vector<int> ttPrefixSum(verticesInLine.size(), 0);
//            for (int i = 0; i < line.size(); ++i)
//                ttPrefixSum[i + 1] = ttPrefixSum[i] + inputGraph.travelTime(line[i]);
//
//            std::vector<ServedRequest> servableRequests;
//            std::vector<Pickupable> pickupables;
//            for (int i = 0; i < verticesInLine.size(); ++i) {
//                const auto &v = verticesInLine[i];
//
//                // 1. Remove pickup-able requests whose detour would grow too large with edge i:
////                removePickupablesWithExceededDetour(pickupables, pickupableIdx, ttPrefixSum, i, fullyCoveredPaths);
//
//                // 2. Check which pickup-able requests can be dropped off at edge i.
//                findPickupablesThatCanBeDroppedOff(pickupables, pickupableIdx, servableRequests, servedByThisLine,
//                                                   tripTimeViolators, ttPrefixSum, i, v);
//
//                // 3. Add unserved passengers that may be picked up here:
//                addNewPickupables(pickupables, pickupableIdx, servedByThisLine, isPaxAlreadyServedByOtherLine,
//                                  ttPrefixSum, i, v);
//            }
//
////            // If the line comprises a single loop, we try to find dropoffs for the remaining pickup-able passengers
////            // along the loop:
////            if (verticesInLine.front() == verticesInLine.back()) {
////                // The prefix sum no longer works when crossing the stitch of the loop. Thus, we explicitly advance
////                // from the stitch forward for every remaining pickup-able:
////                for (const auto &p: pickupables) {
////                    int tt = p.walkingTime + (ttPrefixSum[verticesInLine.size() - 1] - ttPrefixSum[p.pickupVertexIdx]);
////                    bool dropoffFound = false;
////                    for (int i = 1; !dropoffFound && i < p.pickupVertexIdx; ++i) {
////                        const int v = verticesInLine[i];
////                        tt += inputGraph.travelTime(line[i - 1]);
////                        if (tt > getMaxTravelTime(requests[p.requestId], inputConfig))
////                            break;
////
////                        const auto &dropoffsAtV = pdInfo.getPossibleDropoffsAt(v);
////                        const auto &dropoffWalkingTimes = pdInfo.getDropoffWalkingDistsAt(v);
////                        for (int j = 0; j < dropoffsAtV.size(); ++j) {
////                            const auto reqId = dropoffsAtV[j];
////                            if (reqId != p.requestId)
////                                continue;
////
////                            const auto totalTT = tt + dropoffWalkingTimes[j];
////                            if (totalTT <= getMaxTravelTime(requests[p.requestId], inputConfig)) {
////                                // Request can be served by line
////                                servedByThisLine[reqId] = true;
////                                servableRequests.push_back(
////                                        {reqId, p.pickupVertexIdx, p.walkingTime, i, dropoffWalkingTimes[j],
////                                         totalTT - p.walkingTime - dropoffWalkingTimes[j], 0.0});
////                                dropoffFound = true;
////                                break;
////                            }
////                        }
////                    }
////                }
////            }
//
//            for (const auto &p: pickupables) {
//                pickupableIdx[p.requestId] = INVALID_INDEX;
//            }
//
//
//
//            // Find travel time weighted average degree of sharing of each served request
//            std::vector<int> numOccupants(line.size(), 0);
//            for (const auto &served: servableRequests) {
//                for (int i = served.pickupVertexIdx; i < served.dropoffVertexIdx; ++i) {
//                    ++numOccupants[i];
//                }
//            }
//
//            for (auto &served: servableRequests) {
//                int64_t weightedNumOtherRiders = 0;
//                if (served.pickupVertexIdx < served.dropoffVertexIdx) {
//                    for (int i = served.pickupVertexIdx; i < served.dropoffVertexIdx; ++i) {
//                        weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
//                    }
//                } else {
//                    // For passengers crossing the stitch of a loop line:
//                    for (int i = served.pickupVertexIdx; i < verticesInLine.size(); ++i) {
//                        weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
//                    }
//                    for (int i = 0; i < served.dropoffVertexIdx; ++i) {
//                        weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
//                    }
//                }
////                int64_t totalTT = served.pickupWalkingTime + served.inVehicleTime + served.dropoffWalkingTime;
//                int64_t totalTT = served.inVehicleTime;
//                served.ttWeightedAvgSharing =
//                        static_cast<double>(weightedNumOtherRiders) / static_cast<double>(totalTT);
//            }
//
//            return servableRequests;
//        }
//
//        void removePickupablesWithExceededDetour(std::vector<Pickupable> &pickupables,
//                                                 std::vector<int> &pickupableIdx,
//                                                 const std::vector<int> &ttPrefixSum,
//                                                 const int vertexInLineIdx,
//                                                 const std::vector<int> &) const {
//            int k = 0;
//            while (k < pickupables.size()) {
//                auto &p = pickupables[k];
//                const auto tt = p.walkingTime + (ttPrefixSum[vertexInLineIdx] - ttPrefixSum[p.pickupVertexIdx]);
//                if (tt > getMaxTravelTime(requests[p.requestId], inputConfig)) {
//                    const int removedReqId = p.requestId;
//                    KASSERT(pickupableIdx[removedReqId] == k);
//                    p = pickupables.back();
//                    pickupableIdx[p.requestId] = k;
//                    pickupables.pop_back();
//                    pickupableIdx[removedReqId] = INVALID_INDEX;
//                    continue;
//                }
//                ++k;
//            }
//        }
//
//        void findPickupablesThatCanBeDroppedOff(std::vector<Pickupable> &pickupables,
//                                                std::vector<int> &pickupableIdx,
//                                                std::vector<ServedRequest> &servableRequests,
//                                                BitVector &servedByThisLine,
//                                                Subset &tripTimeViolators,
//                                                const std::vector<int> &ttPrefixSum,
//                                                const int vertexInLineIdx, const int v) const {
//            const auto &dropoffsAtV = pdInfo.getPossibleDropoffsAt(v);
//            const auto &dropoffWalkingTimes = pdInfo.getDropoffWalkingDistsAt(v);
//            for (int j = 0; j < dropoffsAtV.size(); ++j) {
//                const auto reqId = dropoffsAtV[j];
//                if (pickupableIdx[reqId] == INVALID_INDEX)
//                    continue;
//                auto &p = pickupables[pickupableIdx[reqId]];
//                KASSERT(p.requestId == reqId);
//                const auto inVehTime = ttPrefixSum[vertexInLineIdx] - ttPrefixSum[p.pickupVertexIdx];
//                const auto totalTT = p.walkingTime + inVehTime + dropoffWalkingTimes[j];
//                if (totalTT <= getMaxTravelTime(requests[p.requestId], inputConfig)) {
//                    // Request can be served by line
//                    servedByThisLine[reqId] = true;
//                    servableRequests.push_back(
//                            {reqId, p.pickupVertexIdx, p.walkingTime, vertexInLineIdx, dropoffWalkingTimes[j],
//                             inVehTime, 0.0});
//                    tripTimeViolators.remove(reqId);
//                    p = pickupables.back();
//                    pickupableIdx[p.requestId] = pickupableIdx[reqId];
//                    pickupables.pop_back();
//                    pickupableIdx[reqId] = INVALID_INDEX;
//                } else {
//                    tripTimeViolators.insert(reqId);
//                }
//            }
//        }
//
//        template<typename IsPassengerAlreadyServedT>
//        void addNewPickupables(std::vector<Pickupable> &pickupables,
//                               std::vector<int> &pickupableIdx,
//                               const BitVector &servedByThisLine,
//                               const IsPassengerAlreadyServedT &isPaxAlreadyServedByOtherLine,
//                               const std::vector<int> &ttPrefixSum,
//                               const int vertexInLineIdx, const int v) const {
//            const auto &pickupsAtV = pdInfo.getPossiblePickupsAt(v);
//            const auto &pickupWalkingTimes = pdInfo.getPickupWalkingDistsAt(v);
//            for (int j = 0; j < pickupsAtV.size(); ++j) {
//                const auto &reqId = pickupsAtV[j];
//                if (isPaxAlreadyServedByOtherLine(reqId) || servedByThisLine[reqId])
//                    continue;
//                const auto &walkingTime = pickupWalkingTimes[j];
//
//                // If request is not pickupable already, mark it as such
//                if (pickupableIdx[reqId] == INVALID_INDEX) {
//                    pickupableIdx[reqId] = pickupables.size();
//                    pickupables.push_back({reqId, vertexInLineIdx, walkingTime});
//                    continue;
//                }
//
//                // If request can already be picked up at earlier edge on line, only store vertex that provides
//                // better travel time (ignoring waiting times).
//                auto &p = pickupables[pickupableIdx[reqId]];
//                const auto ttExisting = p.walkingTime + ttPrefixSum[vertexInLineIdx] - ttPrefixSum[p.pickupVertexIdx];
//                const auto ttNew = walkingTime;
//                if (ttNew < ttExisting) {
//                    p = {reqId, vertexInLineIdx, walkingTime};
//                }
//            }
//        }

        void
        logLine(const FixedLine &line, const int lineId, const int initialEdge, const int maxFlow,
                const std::vector<int> &overlappingWithLine,
                const OverlappingPaths& globalOverlaps,
                const PreliminaryPathsT &paths,
                const RunningTimePerLineStats &runningTimeStats) {

            // Travel time for vehicle
            uint64_t vehicleTravelTime = 0;
            for (const auto &e: line)
                vehicleTravelTime += inputGraph.travelTime(e);

            // Sum of travel times and number of edges for passengers subpaths (overlaps)
            uint64_t sumRiderTravelTime = 0;
            uint64_t sumNumRiderEdges = 0;
            for (const auto &pathId: overlappingWithLine) {
                const auto &p = paths.getPathFor(pathId);
                const auto& o = globalOverlaps.getOverlapFor(pathId);
                for (int i = o.start; i < o.end; ++i) {
                    ++sumNumRiderEdges;
                    sumRiderTravelTime += inputGraph.travelTime(p[i]);
                }
            }


            lineOverviewLogger << lineId << ", "
                               << initialEdge << ","
                               << maxFlow << ","
                               << line.size() << ", "
                               << vehicleTravelTime << ", "
                               << overlappingWithLine.size() << ", "
                               << sumNumRiderEdges << ", "
                               << sumRiderTravelTime << ", "
                               << runningTimeStats.constructLineTime << ", "
                               << runningTimeStats.updatePathsTime << "\n";
        }


        const VehicleInputGraphT &inputGraph;
        const VehicleInputGraphT &reverseGraph;
        const PickupDropoffInfo &pdInfo;
        const std::vector<Request> &requests;
        const InputConfig &inputConfig;

        TimestampedVector<int> residualFlow;

//        // For every edge e, the IDs of not-yet-served requests whose paths start at e are stored in
//        // pathsStartingAtEdge[firstPathStartingAtEdge[e]..firstPathStartingAtEdge[e + 1]].
//        std::vector<int> firstPathStartingAtEdge;
//        std::vector<int> pathsStartingAtEdge;
//
//        // For every edge e, the IDs of not-yet-served requests whose paths end at e are stored in
//        // pathsEndingAtEdge[firstPathEndingAtEdge[e]..firstPathEndingAtEdge[e + 1]].
//        std::vector<int> firstPathEndingAtEdge;
//        std::vector<int> pathsEndingAtEdge;

        OverviewLoggerT &lineOverviewLogger;

    };

} // end namespace