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
#include "OverlappingPaths.h"
#include "InputConfig.h"
#include "Tools/Timer.h"
#include "DataStructures/Containers/FastResetFlagArray.h"
#include "Tools/Logging/NullLogger.h"
#include "Tools/Logging/LogManager.h"
#include "PathStartEndInfo.h"

#include <nlohmann/json.hpp>

namespace mixfix {

    // Given a set of OD-pairs and paths between each origin and destination in a road network, this facility
    // constructs a set of fixed lines with the goal of (partially) covering as many rider paths as possible.
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
            uint64_t choosePartiallyServedRidersTime = 0;
            uint64_t updatePathsTime = 0;
        };

    public:

        GreedyFixedLineFinder(const VehicleInputGraphT &inputGraph, const VehicleInputGraphT &reverseGraph,
                              PathStartEndInfo &pathStartEndInfo, const std::vector<Request> &requests)
                : inputGraph(inputGraph), reverseGraph(reverseGraph),
                  pathStartEndInfo(pathStartEndInfo), requests(requests), inputConfig(InputConfig::getInstance()),
                  lineStatsLogger(LogManager<OverviewLoggerT>::getLogger("lines.csv",
                                                                         "line_id,"
                                                                         "initial_edge,"
                                                                         "max_flow,"
                                                                         "num_edges,"
                                                                         "vehicle_travel_time,"
                                                                         "num_subpaths_covered,"
                                                                         "num_rider_edges,"
                                                                         "sum_rider_travel_time,"
                                                                         "construct_line_time,"
                                                                         "choose_partially_served_riders_time,"
                                                                         "update_paths_time\n")),
                  initialPathsCoverageLogger(LogManager<OverviewLoggerT>::getLogger("initial_paths_coverage.csv",
                                                                                    "initial_path_id,"
                                                                                    "num_legs,"
                                                                                    "num_gaps,"
                                                                                    "travel_time_not_covered\n")) {}

        // Finds fixed lines given preliminary rider paths.
        // Each path is expected to be a sequence of edges in the network.
        // Returns pairs of line and according total rider travel time covered by the line.
        std::vector<FixedLine>
        findFixedLines(PreliminaryPathsT &paths, const bool noVerboseLinesOutput) {

            // Compute total travel times for initial paths and initial flows:
            std::vector<int> residualFlow(inputGraph.numEdges(), 0);
            std::vector<int> initialPathTravelTimes(paths.getMaxPathId() + 1, 0);
            for (int i = 0; i <= paths.getMaxPathId(); ++i) {
                if (!paths.hasPathFor(i))
                    continue;
                for (const auto &e: paths.getPathFor(i)) {
                    ++residualFlow[e];
                    initialPathTravelTimes[i] += inputGraph.travelTime(e);
                }
            }

            std::vector<FixedLine> lines;

            std::vector<int> overlapsWithLine;
            OverlappingPaths globalOverlaps;
            FixedLine line;

            // Statistic for how many legs each initial path has, i.e. how many lines it was chosen to overlap with
            std::vector<int> initialPathsNumLegs(paths.getMaxPathId() + 1, 0);

            Timer timer;
            while (!paths.empty()) {
                RunningTimePerLineStats runningTimeStats;

                // Construct line:
                timer.restart();
                int initialEdge, maxFlowOnLine;
                line.clear();
                overlapsWithLine.clear();
                globalOverlaps.init(paths.getMaxPathId());
                findMaxFlowEdge(residualFlow, initialEdge, maxFlowOnLine);
                if (!noVerboseLinesOutput) {
                    std::cout << "\nInitial edge " << initialEdge
                              << " (tail: " << inputGraph.osmNodeId(inputGraph.edgeTail(initialEdge))
                              << ", head: " << inputGraph.osmNodeId(inputGraph.edgeHead(initialEdge))
                              << ") with flow " << maxFlowOnLine << std::endl;
                }
                if (maxFlowOnLine < inputConfig.minFlowOnLine) {
                    if (!noVerboseLinesOutput)
                        std::cout << "Stopping line construction due to insufficient flow on initial edge."
                                  << std::endl;
                    break;
                }
                constructNextLine(paths, line, initialEdge, maxFlowOnLine, globalOverlaps, overlapsWithLine);
                runningTimeStats.constructLineTime = timer.elapsed<std::chrono::nanoseconds>();

                timer.restart();

                // Greedily choose the overlaps served by the line s.t. the bus capacity is not exceeded anywhere along
                // the line. Sort the overlaps by the fraction of the original path covered by the line and choose each
                // overlap that does not violate the capacity in this order.
                std::vector<std::pair<int, float>> score(overlapsWithLine.size(), {INVALID_INDEX, 0.0f});
                for (int i = 0; i < overlapsWithLine.size(); ++i) {
                    const auto &o = globalOverlaps.getOverlapFor(overlapsWithLine[i]);
                    const auto &p = paths.getPathFor(overlapsWithLine[i]);
                    int overlapLength = 0;
                    const int effectiveStartOfOverlap = o.reachesBeginningOfPath ? 0 : o.start;
                    const int effectiveEndOfOverlap = o.reachesEndOfPath ? p.size() : o.end;
                    for (int j = effectiveStartOfOverlap; j < effectiveEndOfOverlap; ++j)
                        overlapLength += inputGraph.travelTime(p[j]);
                    score[i] = {i, static_cast<float>(overlapLength) /
                                   static_cast<float>(initialPathTravelTimes[p.getAncestorId()])};
                }

//                // Greedily choose the overlaps served by the line s.t. the bus capacity is not exceeded anywhere along
//                // the line. Sort the overlaps by the total rider travel time covered.
//                std::vector<std::pair<int, int>> score(overlapsWithLine.size(), {INVALID_INDEX, 0});
//                for (int i = 0; i < overlapsWithLine.size(); ++i) {
//                    const auto &o = globalOverlaps.getOverlapFor(overlapsWithLine[i]);
//                    const auto &p = paths.getPathFor(overlapsWithLine[i]);
//                    int overlapLength = 0;
//                    for (int j = o.start; j < o.end; ++j)
//                        overlapLength += inputGraph.travelTime(p[j]);
//                    score[i] = {i, overlapLength};
//                }


                std::sort(score.begin(), score.end(),
                          [&](const auto &s1, const auto &s2) { return s1.second > s2.second; });


                std::vector<int> chosenOverlaps;
                std::vector<int> occupancy(line.size(), 0);
                for (const auto &[i, s]: score) {
                    const auto &pathId = overlapsWithLine[i];
                    const auto &path = paths.getPathFor(pathId);
                    auto &o = globalOverlaps.getOverlapFor(pathId);

                    const int startInLine = findStartIdxOfOverlapInLine(line, path, o);
                    const int endInLine = startInLine + (o.end - o.start);

                    int j = startInLine;
                    for (; j < endInLine; ++j) {
                        if (occupancy[j] + 1 > inputConfig.capacity)
                            break;
                    }

                    // If capacity is not broken at any point, choose this overlap and add it to the occupancy.
                    if (j == endInLine) {
                        chosenOverlaps.push_back(pathId);
                        for (int k = startInLine; k < endInLine; ++k)
                            ++occupancy[k];
                        continue;
                    }

                    // Otherwise, reset the overlap so future lines can serve the path instead.
                    o.reset();
                }

                // Roll line back on both ends as long as edges are not used by any chosen overlaps
                int unusedFrontEnd = 0;
                while (unusedFrontEnd < line.size() && occupancy[unusedFrontEnd] == 0)
                    ++unusedFrontEnd;
                KASSERT(unusedFrontEnd < line.size());
                int unusedBackStart = line.size();
                while (unusedBackStart > 0 && occupancy[unusedBackStart - 1] == 0)
                    --unusedBackStart;
                KASSERT(unusedBackStart > 0);
                line.erase(line.begin(), line.begin() + unusedFrontEnd);
                line.erase(line.begin() + unusedBackStart, line.end());

                if (!noVerboseLinesOutput)
                    std::cout << "Found a line of length " << line.size() << " that overlaps "
                              << overlapsWithLine.size() << " paths and partially serves "
                              << chosenOverlaps.size() << " paths." << std::endl;

                runningTimeStats.choosePartiallyServedRidersTime = timer.elapsed<std::chrono::nanoseconds>();
                timer.restart();

                // For each chosen overlap, remove path and add potential subpaths.
                uint64_t sumRiderTravelTime = 0;
                uint64_t sumNumRiderEdges = 0;
                for (const auto &pathId: chosenOverlaps) {
                    const auto &fullPath = paths.getPathFor(pathId);
                    ++initialPathsNumLegs[fullPath.getAncestorId()];
                    const auto &o = globalOverlaps.getOverlapFor(pathId);
                    const int startOfSlicedSubpath = o.reachesBeginningOfPath ? 0 : o.start;
                    const int endOfSlicedSubpath = o.reachesEndOfPath ? fullPath.size() : o.end;

                    // Reduce flow for removed subpath.
                    for (int i = startOfSlicedSubpath; i < endOfSlicedSubpath; ++i) {
                        --residualFlow[fullPath[i]];
                        sumRiderTravelTime += inputGraph.travelTime(fullPath[i]);
                    }
                    sumNumRiderEdges += endOfSlicedSubpath - startOfSlicedSubpath;

                    const auto &[begId, endId] = paths.sliceOutSubpath(pathId, startOfSlicedSubpath,
                                                                       endOfSlicedSubpath);

                    // If subpath at beginning of full path remains, mark new path starts, ends
                    if (begId != INVALID_ID) {
                        pathStartEndInfo.notifyNewPathIdForBeginnings(pathId, begId);
                        pathStartEndInfo.addPathEndAtVertex(begId, 0, 0,
                                                            inputGraph.edgeHead(paths.getPathFor(begId).back()));
                    }

                    // If subpath at beginning of full path remains, mark new path starts, ends
                    if (endId != INVALID_ID) {
                        pathStartEndInfo.notifyNewPathIdForEnds(pathId, endId);
                        pathStartEndInfo.addPathBeginningAtVertex(endId, 0, 0,
                                                                  inputGraph.edgeTail(paths.getPathFor(endId).front()));
                    }
                }
                runningTimeStats.updatePathsTime = timer.elapsed<std::chrono::nanoseconds>();

                // Log line:
                logLine(line, lines.size(), initialEdge, maxFlowOnLine, chosenOverlaps.size(), sumNumRiderEdges,
                        sumRiderTravelTime, runningTimeStats);

                // Add line
                lines.emplace_back(std::move(line));
            }

            // Gather gaps left in initial paths by counting remaining subpaths for each:
            std::vector<int> initialPathsNumGaps(initialPathsNumLegs.size(), 0);
            std::vector<int> initialPathsRemainingRiderTT(initialPathsNumLegs.size(), 0);
            for (const auto &p: paths) {
                int tt = 0;
                for (const auto &e: p)
                    tt += inputGraph.travelTime(e);
                ++initialPathsNumGaps[p.getAncestorId()];
                initialPathsRemainingRiderTT[p.getAncestorId()] += tt;
            }

            for (int i = 0; i < initialPathsNumLegs.size(); ++i) {
                initialPathsCoverageLogger << i << "," << initialPathsNumLegs[i] << "," << initialPathsNumGaps[i] << ","
                                           << initialPathsRemainingRiderTT[i] << "\n";
            }

            return lines;
        }


    private:

        using Path = typename PreliminaryPathsT::Path;

        // Naive pattern matching to find start index of an overlap in the line.
        // (Overlap only contains information about indices in the path.)
        int findStartIdxOfOverlapInLine(const std::vector<int> &line, const Path &path,
                                        const OverlappingPaths::Overlap &o) {
            for (int start = 0; start <= line.size() - (o.end - o.start); ++start) {
                int i = 0;
                while (o.start + i < o.end && line[start + i] == path[o.start + i])
                    ++i;
                if (o.start + i == o.end)
                    return start;
            }
            return INVALID_INDEX;
        }

        void findMaxFlowEdge(const std::vector<int> &flows, int &initialEdge, int &flowOnInitialEdge) {
            // flowOnInitialEdge may be larger than the number of paths if one paths visits maxFlowEdge multiple times.
            flowOnInitialEdge = 0;
            initialEdge = INVALID_EDGE;
            FORALL_EDGES(inputGraph, e) {
                if (flows[e] > flowOnInitialEdge) {
                    initialEdge = e;
                    flowOnInitialEdge = flows[e];
                }
            }
        }

        // Greedily constructs new line from remaining paths. Returns line and max flow on line.
        void constructNextLine(PreliminaryPathsT &paths, FixedLine &line,
                               const int &initialEdge, const int &flowOnInitialEdge,
                               OverlappingPaths &globalOverlaps,
                               std::vector<int> &overlapsWithLine) {

            line = {initialEdge};
            const int tail = inputGraph.edgeTail(initialEdge);
            const int head = inputGraph.edgeHead(initialEdge);
            for (const auto &path: paths) {
                KASSERT(!globalOverlaps.hasKnownOverlap(path.getPathId()));
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == initialEdge) {
                        const bool possibleStartReached = pathStartEndInfo.hasPathBeginningAt(tail, path.getPathId());
                        const bool possibleEndReached = pathStartEndInfo.hasPathEndAt(head, path.getPathId());
                        globalOverlaps.initializeOverlap(path.getPathId(), i, i + 1, possibleStartReached,
                                                         possibleEndReached);
                        overlapsWithLine.push_back(path.getPathId());
                        break; // If path overlaps with edge in multiple places, only consider first overlap
                    }
                }
            }
            const auto numOverlapsOnInitialEdge = overlapsWithLine.size();

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.

            // Extend forward first. All initial overlapping paths that do not have a reachable end are active.
            std::vector<int> activeOverlaps;
            for (int i = 0; i < numOverlapsOnInitialEdge; ++i) {
                const auto &p = overlapsWithLine[i];
                if (!globalOverlaps.getOverlapFor(p).reachesEndOfPath)
                    activeOverlaps.push_back(p);
            }
            extendLineForwards(line, flowOnInitialEdge, globalOverlaps, overlapsWithLine, activeOverlaps, paths);


            // Extend backward. All initial overlapping paths that do not have a reachable start are active.
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
            std::vector<std::pair<int, int>> startingOverlaps; // Overlaps starting on extension [request ID, index of edge in path of request]
            while (!activeOverlaps.empty()) {
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

                if (flowOnExtension < inputConfig.minFlowOnLine)
                    break;

                const int extensionInForwGraph = graph.edgeId(extension);

                // Remove passengers whose paths do not overlap with extension
                int i = 0;
                while (i < activeOverlaps.size()) {
                    auto &o = activeOverlaps[i];
                    const auto &path = paths.getPathFor(o);
                    if (extendOverlapIfPossible(o, path, extensionInForwGraph, mostRecentUsefulVertex)) {
                        ++i;
                    } else {
                        // If overlapping path does not overlap with extension or a possible end has been found,
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

                // There is a new overlap for path p if p may start at the currently last vertex of the line
                // (the tail vertex of the extension edge) and p visits this vertex.
                const auto &starting = pathStartEndInfo.getPathsPossiblyBeginningAt(inputGraph.edgeTail(extension));
                const auto &edgeIndicesOfStarting = pathStartEndInfo.getEdgeIndicesOfPathBeginningsAt(
                        inputGraph.edgeTail(extension));
                KASSERT(starting.size() == edgeIndicesOfStarting.size());
                for (int j = 0; j < starting.size(); ++j) {
                    const auto &pathId = starting[j];

                    // If request has already been answered, ignore it
                    if (!paths.hasPathFor(pathId))
                        continue;

                    // If we already know an earlier overlap of the path with the line, we only keep the earlier one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (globalOverlaps.hasKnownOverlap(pathId))
                        continue;

                    // Check if extension edge lies on the path for reqId:
                    const auto &path = paths.getPathFor(pathId);
                    if (edgeIndicesOfStarting[j] >= path.size())
                        continue;
                    if (path[edgeIndicesOfStarting[j]] != extension)
                        continue;

                    startingOverlaps.push_back({pathId, edgeIndicesOfStarting[j]});
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
                KASSERT(o.end < path.size());
                if (path[o.end] != extension)
                    return false;
                ++o.end;
                if (!pathStartEndInfo.hasPathEndAt(newReachedVertex, path.getPathId())) {
                    // If path continues on extension but no end is reached yet, keep it as overlapping path
                    KASSERT(o.end < path.size());
                    return true;
                }
                o.reachesEndOfPath = true;
                mostRecentUsefulVertex = newReachedVertex;
                return false;
            };

            // Hook function for starting an overlap if possible for a chosen extension.
            // Each starting overlap for the extension found by findStartingOverlapsAtExtension is added as an
            // overlapping path. Starting overlaps of requests for which the head of the extension is a valid end
            // are instead added to fully covered paths.
            const auto commitStartingOverlaps = [this, &globalOverlaps, &overlapsWithLine](
                    std::vector<int> &activeOverlaps,
                    const std::vector<std::pair<int, int>> &startingOverlaps,
                    const int extension) {
                const int head = inputGraph.edgeHead(extension);

                for (const auto &[pathId, startEdgeIdx]: startingOverlaps) {
                    KASSERT(pathStartEndInfo.hasPathBeginningAt(inputGraph.edgeTail(extension), pathId),
                            "Starting overlap does not have a potential start at the tail of the extension.");
                    const bool reachesEndOfPath = pathStartEndInfo.hasPathEndAt(head, pathId);
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
                                                                                 const int extensionInForw,
                                                                                 int &currentOverlapLength) {
                const auto &o = globalOverlaps.getOverlapFor(pathId);
                const auto &path = paths.getPathFor(pathId);
                KASSERT(o.start > 0);
                const bool extensionContinuesOverlap = path[o.start - 1] == extensionInForw;
                if (extensionContinuesOverlap) {
                    currentOverlapLength = o.end - o.start;
                    return true;
                }
                currentOverlapLength = -1;
                return false;
            };

            // Hook function for checking if an extension starts an overlap with a path when deciding next extension.
            static auto findOverlapsStartingAtExtension = [this, &globalOverlaps, &paths](const int extensionInForw) {

                std::vector<std::pair<int, int>> startingOverlaps;

                // There is a new overlap for path p if p may end at the currently last vertex of the line
                // (the head vertex of the extension edge) and p visits this vertex.
                const auto &ending = pathStartEndInfo.getPathsPossiblyEndingAt(inputGraph.edgeHead(extensionInForw));
                const auto &edgeRevIndicesOfEnding = pathStartEndInfo.getRevEdgeIndicesOfPathEndsAt(
                        inputGraph.edgeHead(extensionInForw));
                KASSERT(ending.size() == edgeRevIndicesOfEnding.size());
                for (int j = 0; j < ending.size(); ++j) {
                    const auto &pathId = ending[j];

                    // If request has already been answered, ignore it
                    if (!paths.hasPathFor(pathId))
                        continue;

                    // If we already know a later overlap of the path with the line, we only keep the later one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (globalOverlaps.hasKnownOverlap(pathId))
                        continue;

                    // Check if extension edge lies on the path for reqId:
                    const auto &path = paths.getPathFor(pathId);
                    const int edgeIdx = path.size() - 1 - edgeRevIndicesOfEnding[j];
                    if (edgeIdx < 0)
                        continue;
                    if (path[edgeIdx] != extensionInForw)
                        continue;

                    startingOverlaps.push_back({pathId, edgeIdx});
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
                if (path[o.start - 1] != extensionInForw)
                    return false;

                --o.start;
                if (!pathStartEndInfo.hasPathBeginningAt(newReachedVertex, pathId)) {
                    // If path continues on extension but no possible start has been reached, keep it as overlapping path
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
                const int tail = inputGraph.edgeTail(extensionInForw);

                for (const auto &[pathId, lastEdgeIdx]: startingOverlaps) {
                    KASSERT(pathStartEndInfo.hasPathEndAt(inputGraph.edgeHead(extensionInForw), pathId),
                            "Starting overlap does not have a potential end at the head of the extension.");

                    const bool reachesBegOfPath = pathStartEndInfo.hasPathBeginningAt(tail, pathId);
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

        void logLine(const FixedLine &line,
                     const int lineId,
                     const int initialEdge,
                     const int maxFlow,
                     const int numPartiallyServed,
                     const int sumNumRiderEdges,
                     const int sumRiderTravelTime,
                     const RunningTimePerLineStats &runningTimeStats) {

            // Travel time for vehicle
            uint64_t vehicleTravelTime = 0;
            for (const auto &e: line)
                vehicleTravelTime += inputGraph.travelTime(e);


            lineStatsLogger << lineId << ", "
                            << initialEdge << ","
                            << maxFlow << ","
                            << line.size() << ", "
                            << vehicleTravelTime << ", "
                            << numPartiallyServed << ", "
                            << sumNumRiderEdges << ", "
                            << sumRiderTravelTime << ", "
                            << runningTimeStats.constructLineTime << ", "
                            << runningTimeStats.choosePartiallyServedRidersTime << ", "
                            << runningTimeStats.updatePathsTime << "\n";
        }


        const VehicleInputGraphT &inputGraph;
        const VehicleInputGraphT &reverseGraph;
        PathStartEndInfo &pathStartEndInfo;
        const std::vector<Request> &requests;
        const InputConfig &inputConfig;

        OverviewLoggerT &lineStatsLogger;
        OverviewLoggerT &initialPathsCoverageLogger;

    };

} // end namespace