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
            uint64_t findServedPaxTime = 0;
            uint64_t updatePathsTime = 0;
        };

    public:

        GreedyFixedLineFinder(const VehicleInputGraphT &inputGraph, const VehicleInputGraphT &reverseGraph,
                              const PickupDropoffInfo &pdInfo, const std::vector<Request> &requests)
                : inputGraph(inputGraph), reverseGraph(reverseGraph),
                  pdInfo(pdInfo), requests(requests), inputConfig(InputConfig::getInstance()),
                  residualFlow(inputGraph.numEdges(), 0),
                  firstPathStartingAtEdge(inputGraph.numEdges() + 1, 0),
                  pathsStartingAtEdge(),
                  firstPathEndingAtEdge(inputGraph.numEdges() + 1, 0),
                  pathsEndingAtEdge(),
                  lineOverviewLogger(LogManager<OverviewLoggerT>::getLogger("lines.csv",
                                                                            "line_id,"
                                                                            "initial_edge,"
                                                                            "max_flow,"
                                                                            "num_edges,"
                                                                            "total_travel_time,"
                                                                            "num_pax_served,"
                                                                            "num_pax_fully_covered,"
                                                                            "num_trip_time_violations,"
                                                                            "avg_direct_travel_time,"
                                                                            "avg_rel_pax_detour,"
                                                                            "avg_abs_pax_detour,"
                                                                            "avg_pickup_walking_time,"
                                                                            "avg_dropoff_walking_time,"
                                                                            "avg_walking_time_diff,"
                                                                            "avg_tt_weighted_avg_sharing,"
                                                                            "avg_tt_weighted_occupancy,"
                                                                            "construct_line_time,"
                                                                            "find_served_pax_time,"
                                                                            "update_paths_time\n")) {}

        // Finds fixed lines given preliminary rider paths.
        // Each path is expected to be a sequence of edges in the network.
        std::vector<std::pair<FixedLine, std::vector<ServedRequest>>> findFixedLines(PreliminaryPathsT &paths) {

            std::vector<std::pair<FixedLine, std::vector<ServedRequest>>> lines;

            initializePathsStartingOrEndingAtEdge<true>(paths, firstPathStartingAtEdge, pathsStartingAtEdge);
            initializePathsStartingOrEndingAtEdge<false>(paths, firstPathEndingAtEdge, pathsEndingAtEdge);

            BitVector isServable(requests.size(), false);
            std::vector<int> pickupableIdx(requests.size(), INVALID_INDEX);
            Subset tripTimeViolators(requests.size());
            FastResetFlagArray<> hasOverlapped(requests.size());

            FixedLine line;
            std::vector<int> fullyCoveredPaths;
            Timer timer;
            while (!paths.empty()) {
                std::cout << "\n\n";

                RunningTimePerLineStats runningTimeStats;

                // Construct line:
                timer.restart();
                int initialEdge, maxFlowOnLine;
                line.clear();
                fullyCoveredPaths.clear();
                hasOverlapped.reset();
                findInitialEdge(paths, initialEdge, maxFlowOnLine);
                std::cout << "Initial edge " << initialEdge
                          << " (tail: " << inputGraph.osmNodeId(inputGraph.edgeTail(initialEdge))
                          << ", head: " << inputGraph.osmNodeId(inputGraph.edgeHead(initialEdge))
                          << ") with flow " << maxFlowOnLine << std::endl;
                if (maxFlowOnLine < inputConfig.minMaxFlowOnLine) {
                    std::cout << "Stopping line construction due to insufficient flow on initial edge." << std::endl;
                    break;
                }
                constructNextLine(paths, line, initialEdge, maxFlowOnLine, fullyCoveredPaths, hasOverlapped);
                runningTimeStats.constructLineTime = timer.elapsed<std::chrono::nanoseconds>();

                // Find passengers served by line:
                timer.restart();
                tripTimeViolators.clear();
                std::vector<int> verticesInLine;
                verticesInLine.push_back(inputGraph.edgeTail(line.front()));
                for (const auto &e: line)
                    verticesInLine.push_back(inputGraph.edgeHead(e));

                KASSERT(verifyFullyCoveredPathsPickupsAndDropoffs(verticesInLine, fullyCoveredPaths),
                        "Line does not have possible pickup and dropoff locations for all fully covered paths.");

                const auto &pax = findPassengersServableByLine(line, verticesInLine, [&](const int reqId) {
                    return !paths.hasPathFor(reqId);
                }, fullyCoveredPaths, isServable, pickupableIdx, tripTimeViolators);
                runningTimeStats.findServedPaxTime = timer.elapsed<std::chrono::nanoseconds>();
                std::cout << "Found a line of length " << line.size() << " that can serve " << pax.size()
                          << " passengers. (Num fully covered paths = " << fullyCoveredPaths.size() << ")" << std::endl;


                if (pax.size() < inputConfig.minNumPaxPerLine)
                    break;

                // Update paths data structures:
                timer.restart();
                int numServedButNotFullyCovered = 0;
                for (const auto &served: pax) {
                    if (!contains(fullyCoveredPaths.begin(), fullyCoveredPaths.end(), served.requestId)) {
                        ++numServedButNotFullyCovered;
                    }
                    paths.removePathForRequest(served.requestId);
                    isServable[served.requestId] = false;
                }
                updatePathsStartingOrEndingAtEdge(paths, firstPathStartingAtEdge, pathsStartingAtEdge);
                verifyPathsStartingOrEndingAtEdge<true>(paths, firstPathStartingAtEdge, pathsStartingAtEdge);
                updatePathsStartingOrEndingAtEdge(paths, firstPathEndingAtEdge, pathsEndingAtEdge);
                verifyPathsStartingOrEndingAtEdge<false>(paths, firstPathEndingAtEdge, pathsEndingAtEdge);
                runningTimeStats.updatePathsTime = timer.elapsed<std::chrono::nanoseconds>();
                std::cout << "\tNumber of passengers served whose paths are not fully covered by "
                             "line (origin to destination): " << numServedButNotFullyCovered << std::endl;

                logLine(line, lines.size(), initialEdge, maxFlowOnLine, pax, fullyCoveredPaths.size(),
                        tripTimeViolators, runningTimeStats);

                // Add line
                lines.push_back(std::make_pair(std::move(line), std::move(pax)));
            }

            return lines;
        }


    private:

        using Path = typename PreliminaryPathsT::Path;

        struct OverlappingPath {

            OverlappingPath(const int requestId, const int start, const int end,
                            const bool possiblePickupReached, const bool possibleDropoffReached)
                    : requestId(requestId), start(start), end(end),
                      possiblePickupReached(possiblePickupReached), possibleDropoffReached(possibleDropoffReached) {}

            int requestId;
            int start; // smallest known index in path of passenger where path overlaps with line
            int end; // (one past) largest known index in path of passenger where path overlaps with line

            bool possiblePickupReached;
            bool possibleDropoffReached;
        };

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
                               std::vector<int> &fullyCoveredPaths,
                               FastResetFlagArray<> &hasOverlapped) {
            std::vector<OverlappingPath> overlappingPaths;
            std::vector<OverlappingPath> overlappingReachedEnd;
            line = {initialEdge};
            const auto pickupsAtTail = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(initialEdge));
            const auto dropoffsAtHead = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(initialEdge));
            for (const auto &path: paths) {
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == initialEdge) {
                        const bool possiblePickupReached = contains(pickupsAtTail.begin(), pickupsAtTail.end(),
                                                                    path.getPathId());
                        const bool possibleDropoffReached = contains(dropoffsAtHead.begin(), dropoffsAtHead.end(),
                                                                     path.getPathId());
                        hasOverlapped.set(path.getPathId());
                        if (possiblePickupReached && possibleDropoffReached) {
                            fullyCoveredPaths.push_back(path.getPathId());
                        } else if (possibleDropoffReached) {
                            overlappingReachedEnd.push_back(
                                    {path.getPathId(), i, i + 1, possiblePickupReached, possibleDropoffReached});
                        } else {
                            overlappingPaths.push_back(
                                    {path.getPathId(), i, i + 1, possiblePickupReached, possibleDropoffReached});
                        }
                        break; // If path overlaps with edge in multiple places, only consider first overlap
                    }
                }
            }

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.
            // Paths that contain the initial edge whose end is reached by the forward extension but who may still be
            // extended toward their start by the backward extension
            extendLineForwards(line, flowOnInitialEdge, overlappingPaths, overlappingReachedEnd, fullyCoveredPaths,
                               paths, hasOverlapped);
            std::vector<OverlappingPath> overlappingReachedBeginning;
            extendLineBackwards(line, flowOnInitialEdge, overlappingReachedEnd, overlappingReachedBeginning,
                                fullyCoveredPaths, paths, hasOverlapped);
        }

        void constructNextLineBackwardFirst(PreliminaryPathsT &paths, FixedLine &line,
                                            const int &initialEdge, const int &flowOnInitialEdge,
                                            std::vector<int> &fullyCoveredPaths) {
            std::vector<OverlappingPath> overlappingPaths;
            std::vector<OverlappingPath> overlappingReachedBeginning;
            line = {initialEdge};
            const auto pickupsAtTail = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(initialEdge));
            const auto dropoffsAtHead = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(initialEdge));
            for (const auto &path: paths) {
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == initialEdge) {
                        const bool possiblePickupReached = contains(pickupsAtTail.begin(), pickupsAtTail.end(),
                                                                    path.getPathId());
                        const bool possibleDropoffReached = contains(dropoffsAtHead.begin(), dropoffsAtHead.end(),
                                                                     path.getPathId());
                        if (possiblePickupReached && possibleDropoffReached) {
                            fullyCoveredPaths.push_back(path.getPathId());
                        } else if (possiblePickupReached) {
                            overlappingReachedBeginning.push_back(
                                    {path.getPathId(), i, i + 1, possiblePickupReached, possibleDropoffReached});
                        } else {
                            overlappingPaths.push_back(
                                    {path.getPathId(), i, i + 1, possiblePickupReached, possibleDropoffReached});
                        }
                        break; // If path overlaps with edge in multiple places, only consider first overlap
                    }
                }
            }

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.
            // Paths that contain the initial edge whose end is reached by the forward extension but who may still be
            // extended toward their start by the backward extension
            extendLineBackwards(line, flowOnInitialEdge, overlappingPaths, overlappingReachedBeginning,
                                fullyCoveredPaths, paths);
            std::vector<OverlappingPath> overlappingReachedEnd;
            extendLineForwards(line, flowOnInitialEdge, overlappingReachedBeginning, overlappingReachedEnd,
                               fullyCoveredPaths, paths);
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
                                          std::vector<OverlappingPath> &startingOverlaps,
                                          const VehicleInputGraphT &graph,
                                          const int v,
                                          const std::vector<OverlappingPath> &overlapping,
                                          const PreliminaryPathsT &paths,
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
                for (const auto &o: overlapping) {
                    const auto &path = paths.getPathFor(o.requestId);
                    if (doesExtensionContinueOverlap(o, path, eInForwGraph)) {
                        const uint64_t overlapLength = 1 + o.end - o.start;
                        score += computeScoreForOverlapLength(overlapLength);
                        ++flow;
                    }
                }


                // Paths that begin at e increase the score by 1.
                const auto startingOverlapsOnE = findOverlapsStartingAtExtension(paths, eInForwGraph);
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
                        std::vector<OverlappingPath> &overlapping,
                        const PreliminaryPathsT &paths,
                        const VehicleInputGraphT &graph,
                        const DoesExtensionContinueOverlapT &doesExtensionContinueOverlap,
                        const FindOverlapsStartingAtExtensionT &findOverlapsStartingAtExtension,
                        const ExtendOverlapIfPossibleT &extendOverlapIfPossible,
                        const CommitStartingOverlapsT &commitStartingOverlaps) {

            // Begin extending
            int mostRecentUsefulVertex = graph.edgeHead(line.back());
            double flowDif = 0.0;
            std::vector<OverlappingPath> startingOverlaps; // Overlaps starting on extension
            while (flowDif < inputConfig.maxFlowRatioOnLine && !overlapping.empty()) {
                const auto v = graph.edgeHead(line.back());

                int extension, flowOnExtension;
                startingOverlaps.clear();
                if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::NONE) {
                    // Allow loops on lines
                    findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, overlapping, paths,
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

                    findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, overlapping, paths,
                                          doesExtensionContinueOverlap, findOverlapsStartingAtExtension);

                } else if constexpr (AvoidLoopsStrat == AvoidLoopsStrategy::EDGE
                                     || AvoidLoopsStrat == AvoidLoopsStrategy::EDGE_ROLLBACK) {
                    // Stop extending if chosen extension edge has been visited before

                    findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, overlapping, paths,
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

                        findMaxScoreExtension(extension, flowOnExtension, startingOverlaps, graph, v, overlapping,
                                              paths,
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
                while (i < overlapping.size()) {
                    auto &o = overlapping[i];
                    const auto &path = paths.getPathFor(o.requestId);
                    if (extendOverlapIfPossible(o, path, extensionInForwGraph, mostRecentUsefulVertex)) {
                        ++i;
                    } else {
                        // If overlapping path does not overlap with extension or a dropoff has been found,
                        // remove from overlapping paths.
                        std::swap(overlapping[i], overlapping.back());
                        overlapping.pop_back();
                    }
                }

                // Add overlaps starting with extension
                commitStartingOverlaps(overlapping, startingOverlaps, extensionInForwGraph);

                // Add extension
                line.push_back(extension);
            }
        }

        void extendLineForwards(FixedLine &line,
                                const int &maxFlowOnLine,
                                std::vector<OverlappingPath> &overlapping,
                                std::vector<OverlappingPath> &overlappingReachedEnd,
                                std::vector<int> &fullyCoveredPaths,
                                const PreliminaryPathsT &paths,
                                FastResetFlagArray<> &hasOverlapped) {


            // Hook function for checking if an extension continues an overlap with a path when deciding next extension.
            static auto doesExtensionContinueOverlap = [](const OverlappingPath &o, const Path &path,
                                                          const int extension) {
                KASSERT(o.end < path.size());
                return path[o.end] == extension;
            };

            // Hook function for finding new overlaps when deciding next extension.
            auto findOverlapsStartingAtExtension = [this, &hasOverlapped](const PreliminaryPathsT &paths,
                                                                          const int extension) {

                std::vector<OverlappingPath> startingOverlaps;

                // There is a new overlap for request r if r can be picked up at the currently last vertex of the line
                // (the tail vertex of the extension edge) and the preliminary path of r visits this vertex.
                const auto &pickups = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension));
                for (const auto &reqId: pickups) {

                    // If request has already been answered, ignore it
                    if (!paths.hasPathFor(reqId))
                        continue;

                    // If we already know an earlier overlap of the path with the line, we only keep the earlier one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (hasOverlapped.isSet(reqId))
                        continue;

                    // Check if extension edge lies on the path for reqId:
                    const auto &path = paths.getPathFor(reqId);
                    int i = 0;
                    while (i < path.size() && path[i] != extension)
                        ++i;
                    if (i == path.size())
                        continue;

                    startingOverlaps.push_back({reqId, i, i + 1, true, false});
                }

                return startingOverlaps;
            };

            // Hook function for extending an overlap if possible for a chosen extension.
            // Returns true if overlap has been extended and should be kept as overlapping or false if it
            // should be removed.
            const auto extendOverlapIfPossible = [this, &fullyCoveredPaths, &overlappingReachedEnd](
                    OverlappingPath &o,
                    const Path &path,
                    const int extension,
                    int &mostRecentUsefulVertex) {
                const int newReachedVertex = inputGraph.edgeHead(extension);
                const auto &dropoffsAtNewVertex = pdInfo.getPossibleDropoffsAt(newReachedVertex);
                KASSERT(o.end < path.size());
                if (path[o.end] != extension)
                    return false;
                ++o.end;
                if (!contains(dropoffsAtNewVertex.begin(), dropoffsAtNewVertex.end(), o.requestId)) {
                    // If path continues on extension but no dropoff is reached yet, keep it as overlapping path
                    return true;
                }
                o.possibleDropoffReached = true;
                mostRecentUsefulVertex = newReachedVertex;
                if (!o.possiblePickupReached)
                    // In case a dropoff is reached but no pickup spot is covered yet, the backward
                    // extension may still fully cover it.
                    overlappingReachedEnd.push_back(o);
                else
                    // In case both a pickup and dropoff is on the line, the path is fully covered.
                    fullyCoveredPaths.push_back(o.requestId);
                return false;
            };

            // Hook function for starting an overlap if possible for a chosen extension.
            // Each starting overlap for the extension found by findStartingOverlapsAtExtension is added as an
            // overlapping path. Starting overlaps of requests for which the head of the extension is a valid dropoff
            // are instead added to fully covered paths.
            const auto commitStartingOverlaps = [this, &fullyCoveredPaths, &hasOverlapped](
                    std::vector<OverlappingPath> &overlapping,
                    const std::vector<OverlappingPath> &startingOverlaps,
                    const int extension) {
                const auto &dropoffsAtNewVertex = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extension));

                for (const auto &o: startingOverlaps) {
                    KASSERT(contains(pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension)).begin(),
                                     pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension)).end(),
                                     o.requestId),
                            "Starting overlap does not have a pickup at the tail of the extension.");
                    hasOverlapped.set(o.requestId);

                    if (contains(dropoffsAtNewVertex.begin(), dropoffsAtNewVertex.end(), o.requestId)) {
                        fullyCoveredPaths.push_back(o.requestId);
                        continue;
                    }
                    overlapping.push_back(o);
                }
            };

            // Execute line extension with the forward extension hooks
            extendLine(line, maxFlowOnLine, overlapping, paths, inputGraph, doesExtensionContinueOverlap,
                       findOverlapsStartingAtExtension, extendOverlapIfPossible, commitStartingOverlaps);
        }


        void extendLineBackwards(FixedLine &line,
                                 const int &maxFlowOnLine,
                                 std::vector<OverlappingPath> &overlapping,
                                 std::vector<OverlappingPath> &overlappingReachedBeginning,
                                 std::vector<int> &fullyCoveredPaths,
                                 const PreliminaryPathsT &paths,
                                 FastResetFlagArray<> &hasOverlapped) {

            // Hook function for checking if an extension continues an overlap with a path when deciding next extension.
            static auto doesExtensionContinueOverlap = [](const OverlappingPath &o, const Path &path,
                                                          const int extensionInForw) {
                KASSERT(o.start > 0);
                return path[o.start - 1] == extensionInForw;
            };

            // Hook function for checking if an extension starts an overlap with a path when deciding next extension.
            static auto findOverlapsStartingAtExtension = [this, &hasOverlapped](const PreliminaryPathsT &paths,
                                                                                 const int extensionInForw) {

                std::vector<OverlappingPath> startingOverlaps;

                // There is a new overlap for request r if r can be dropped off at the currently last vertex of the line
                // (the head vertex of the extension edge) and the preliminary path of r visits this vertex.
                const auto &dropoffs = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extensionInForw));
                for (const auto &reqId: dropoffs) {

                    // If request has already been answered, ignore it
                    if (!paths.hasPathFor(reqId))
                        continue;

                    // If we already know a later overlap of the path with the line, we only keep the later one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (hasOverlapped.isSet(reqId))
                        continue;

                    // Check if extension edge lies on the path for reqId:
                    const auto &path = paths.getPathFor(reqId);
                    int i = path.size() - 1;
                    while (i >= 0 && path[i] != extensionInForw)
                        --i;
                    if (i == -1)
                        continue;

                    startingOverlaps.push_back({reqId, i, i + 1, false, true});
                }

                return startingOverlaps;
            };

            // Hook function for extending an overlap if possible for a chosen extension.
            // Returns true if overlap has been extended and should be kept as overlapping or false if it
            // should be removed.
            const auto extendOverlapIfPossible = [this, &fullyCoveredPaths, &overlappingReachedBeginning](
                    OverlappingPath &o,
                    const Path &path,
                    const int extensionInForw,
                    int &mostRecentUsefulVertex) {
                KASSERT(o.start > 0);
                const auto newReachedVertex = inputGraph.edgeTail(extensionInForw);
                const auto &pickupsAtNewVertex = pdInfo.getPossiblePickupsAt(newReachedVertex);
                if (path[o.start - 1] != extensionInForw)
                    return false;

                --o.start;
                if (!contains(pickupsAtNewVertex.begin(), pickupsAtNewVertex.end(), o.requestId)) {
                    // If path continues on extension but no possible pickup has been reached, keep it as overlapping path
                    return true;
                }
                o.possiblePickupReached = true;
                mostRecentUsefulVertex = newReachedVertex;
                if (!o.possibleDropoffReached)
                    // In case a pickup is reached but no dropoff spot is covered yet, the backward
                    // extension may still fully cover it.
                    overlappingReachedBeginning.push_back(o);
                else
                    // In case both a pickup and dropoff is on the line, the path is fully covered.
                    fullyCoveredPaths.push_back(o.requestId);
                return false;
            };

            // Hook function for starting overlaps for a chosen extension.
            // If extension starts an overlap with the path, we do not know an overlap already, and the path is not
            // fully covered right away, the overlap is added to overlapping..
            const auto commitStartingOverlaps = [this, &fullyCoveredPaths, &hasOverlapped](
                    std::vector<OverlappingPath> &overlapping,
                    const std::vector<OverlappingPath> &startingOverlaps,
                    const int extensionInForw) {
                const auto &pickupsAtNewVertex = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extensionInForw));

                for (const auto &o: startingOverlaps) {
                    KASSERT(contains(pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extensionInForw)).begin(),
                                     pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extensionInForw)).end(),
                                     o.requestId),
                            "Starting overlap does not have a dropoff at the head of the extension.");
                    hasOverlapped.set(o.requestId);

                    if (contains(pickupsAtNewVertex.begin(), pickupsAtNewVertex.end(), o.requestId)) {
                        fullyCoveredPaths.push_back(o.requestId);
                        continue;
                    }
                    overlapping.push_back(o);
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
            extendLine(line, maxFlowOnLine, overlapping, paths, reverseGraph, doesExtensionContinueOverlap,
                       findOverlapsStartingAtExtension, extendOverlapIfPossible, commitStartingOverlaps);

            // Convert line back into forwards representation
            std::reverse(line.begin(), line.end());
            for (auto &e: line)
                e = reverseGraph.edgeId(e);
        }


        struct Pickupable {
            int requestId;
            int pickupVertexIdx;
            int walkingTime;
        };

        template<typename IsPassengerAlreadyServedT>
        std::vector<ServedRequest>
        findPassengersServableByLine(const FixedLine &line,
                                     const std::vector<int> &verticesInLine,
                                     const IsPassengerAlreadyServedT &isPaxAlreadyServedByOtherLine,
                                     const std::vector<int> &,
                                     BitVector &servedByThisLine,
                                     std::vector<int> &pickupableIdx,
                                     Subset &tripTimeViolators) const {
            KASSERT(servedByThisLine.cardinality() == 0);
            KASSERT(std::all_of(pickupableIdx.begin(), pickupableIdx.end(),
                                [](const int idx) { return idx == INVALID_INDEX; }));

            // We iterate over the vertices of the line from front to back, keeping track of a subset of requests P that
            // may be picked up by the vehicle before the current vertex.
            // When reaching vertex i, we perform the following:
            //  1. Remove a request r from P if the detour of r has become too large with vertex i.
            //  2. For every r in P, check whether r can be dropped off at vertex i. If so, r can be served by the line. Remove r from P.
            //  3. Add all unserved requests that can be picked up at vertex i to P.
            //
            // There may be multiple possible pickup vertices on the same line for a request r. We always use the one with
            // the smallest travel time to the current vertex i, i.e. walking distance + current in-vehicle distance.

            // Build prefix sum of travel time along line for fast retrieval of travel time between any two vertices:
            std::vector<int> ttPrefixSum(verticesInLine.size(), 0);
            for (int i = 0; i < line.size(); ++i)
                ttPrefixSum[i + 1] = ttPrefixSum[i] + inputGraph.travelTime(line[i]);

            std::vector<ServedRequest> servableRequests;
            std::vector<Pickupable> pickupables;
            for (int i = 0; i < verticesInLine.size(); ++i) {
                const auto &v = verticesInLine[i];

                // 1. Remove pickup-able requests whose detour would grow too large with edge i:
//                removePickupablesWithExceededDetour(pickupables, pickupableIdx, ttPrefixSum, i, fullyCoveredPaths);

                // 2. Check which pickup-able requests can be dropped off at edge i.
                findPickupablesThatCanBeDroppedOff(pickupables, pickupableIdx, servableRequests, servedByThisLine,
                                                   tripTimeViolators, ttPrefixSum, i, v);

                // 3. Add unserved passengers that may be picked up here:
                addNewPickupables(pickupables, pickupableIdx, servedByThisLine, isPaxAlreadyServedByOtherLine,
                                  ttPrefixSum, i, v);
            }

//            // If the line comprises a single loop, we try to find dropoffs for the remaining pickup-able passengers
//            // along the loop:
//            if (verticesInLine.front() == verticesInLine.back()) {
//                // The prefix sum no longer works when crossing the stitch of the loop. Thus, we explicitly advance
//                // from the stitch forward for every remaining pickup-able:
//                for (const auto &p: pickupables) {
//                    int tt = p.walkingTime + (ttPrefixSum[verticesInLine.size() - 1] - ttPrefixSum[p.pickupVertexIdx]);
//                    bool dropoffFound = false;
//                    for (int i = 1; !dropoffFound && i < p.pickupVertexIdx; ++i) {
//                        const int v = verticesInLine[i];
//                        tt += inputGraph.travelTime(line[i - 1]);
//                        if (tt > getMaxTravelTime(requests[p.requestId], inputConfig))
//                            break;
//
//                        const auto &dropoffsAtV = pdInfo.getPossibleDropoffsAt(v);
//                        const auto &dropoffWalkingTimes = pdInfo.getDropoffWalkingDistsAt(v);
//                        for (int j = 0; j < dropoffsAtV.size(); ++j) {
//                            const auto reqId = dropoffsAtV[j];
//                            if (reqId != p.requestId)
//                                continue;
//
//                            const auto totalTT = tt + dropoffWalkingTimes[j];
//                            if (totalTT <= getMaxTravelTime(requests[p.requestId], inputConfig)) {
//                                // Request can be served by line
//                                servedByThisLine[reqId] = true;
//                                servableRequests.push_back(
//                                        {reqId, p.pickupVertexIdx, p.walkingTime, i, dropoffWalkingTimes[j],
//                                         totalTT - p.walkingTime - dropoffWalkingTimes[j], 0.0});
//                                dropoffFound = true;
//                                break;
//                            }
//                        }
//                    }
//                }
//            }

            for (const auto &p: pickupables) {
                pickupableIdx[p.requestId] = INVALID_INDEX;
            }



            // Find travel time weighted average degree of sharing of each served request
            std::vector<int> numOccupants(line.size(), 0);
            for (const auto &served: servableRequests) {
                for (int i = served.pickupVertexIdx; i < served.dropoffVertexIdx; ++i) {
                    ++numOccupants[i];
                }
            }

            for (auto &served: servableRequests) {
                int64_t weightedNumOtherRiders = 0;
                if (served.pickupVertexIdx < served.dropoffVertexIdx) {
                    for (int i = served.pickupVertexIdx; i < served.dropoffVertexIdx; ++i) {
                        weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
                    }
                } else {
                    // For passengers crossing the stitch of a loop line:
                    for (int i = served.pickupVertexIdx; i < verticesInLine.size(); ++i) {
                        weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
                    }
                    for (int i = 0; i < served.dropoffVertexIdx; ++i) {
                        weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
                    }
                }
//                int64_t totalTT = served.pickupWalkingTime + served.inVehicleTime + served.dropoffWalkingTime;
                int64_t totalTT = served.inVehicleTime;
                served.ttWeightedAvgSharing =
                        static_cast<double>(weightedNumOtherRiders) / static_cast<double>(totalTT);
            }

            return servableRequests;
        }

        void removePickupablesWithExceededDetour(std::vector<Pickupable> &pickupables,
                                                 std::vector<int> &pickupableIdx,
                                                 const std::vector<int> &ttPrefixSum,
                                                 const int vertexInLineIdx,
                                                 const std::vector<int> &) const {
            int k = 0;
            while (k < pickupables.size()) {
                auto &p = pickupables[k];
                const auto tt = p.walkingTime + (ttPrefixSum[vertexInLineIdx] - ttPrefixSum[p.pickupVertexIdx]);
                if (tt > getMaxTravelTime(requests[p.requestId], inputConfig)) {
                    const int removedReqId = p.requestId;
                    KASSERT(pickupableIdx[removedReqId] == k);
                    p = pickupables.back();
                    pickupableIdx[p.requestId] = k;
                    pickupables.pop_back();
                    pickupableIdx[removedReqId] = INVALID_INDEX;
                    continue;
                }
                ++k;
            }
        }

        void findPickupablesThatCanBeDroppedOff(std::vector<Pickupable> &pickupables,
                                                std::vector<int> &pickupableIdx,
                                                std::vector<ServedRequest> &servableRequests,
                                                BitVector &servedByThisLine,
                                                Subset &tripTimeViolators,
                                                const std::vector<int> &ttPrefixSum,
                                                const int vertexInLineIdx, const int v) const {
            const auto &dropoffsAtV = pdInfo.getPossibleDropoffsAt(v);
            const auto &dropoffWalkingTimes = pdInfo.getDropoffWalkingDistsAt(v);
            for (int j = 0; j < dropoffsAtV.size(); ++j) {
                const auto reqId = dropoffsAtV[j];
                if (pickupableIdx[reqId] == INVALID_INDEX)
                    continue;
                auto &p = pickupables[pickupableIdx[reqId]];
                KASSERT(p.requestId == reqId);
                const auto inVehTime = ttPrefixSum[vertexInLineIdx] - ttPrefixSum[p.pickupVertexIdx];
                const auto totalTT = p.walkingTime + inVehTime + dropoffWalkingTimes[j];
                if (totalTT <= getMaxTravelTime(requests[p.requestId], inputConfig)) {
                    // Request can be served by line
                    servedByThisLine[reqId] = true;
                    servableRequests.push_back(
                            {reqId, p.pickupVertexIdx, p.walkingTime, vertexInLineIdx, dropoffWalkingTimes[j],
                             inVehTime, 0.0});
                    tripTimeViolators.remove(reqId);
                    p = pickupables.back();
                    pickupableIdx[p.requestId] = pickupableIdx[reqId];
                    pickupables.pop_back();
                    pickupableIdx[reqId] = INVALID_INDEX;
                } else {
                    tripTimeViolators.insert(reqId);
                }
            }
        }

        template<typename IsPassengerAlreadyServedT>
        void addNewPickupables(std::vector<Pickupable> &pickupables,
                               std::vector<int> &pickupableIdx,
                               const BitVector &servedByThisLine,
                               const IsPassengerAlreadyServedT &isPaxAlreadyServedByOtherLine,
                               const std::vector<int> &ttPrefixSum,
                               const int vertexInLineIdx, const int v) const {
            const auto &pickupsAtV = pdInfo.getPossiblePickupsAt(v);
            const auto &pickupWalkingTimes = pdInfo.getPickupWalkingDistsAt(v);
            for (int j = 0; j < pickupsAtV.size(); ++j) {
                const auto &reqId = pickupsAtV[j];
                if (isPaxAlreadyServedByOtherLine(reqId) || servedByThisLine[reqId])
                    continue;
                const auto &walkingTime = pickupWalkingTimes[j];

                // If request is not pickupable already, mark it as such
                if (pickupableIdx[reqId] == INVALID_INDEX) {
                    pickupableIdx[reqId] = pickupables.size();
                    pickupables.push_back({reqId, vertexInLineIdx, walkingTime});
                    continue;
                }

                // If request can already be picked up at earlier edge on line, only store vertex that provides
                // better travel time (ignoring waiting times).
                auto &p = pickupables[pickupableIdx[reqId]];
                const auto ttExisting = p.walkingTime + ttPrefixSum[vertexInLineIdx] - ttPrefixSum[p.pickupVertexIdx];
                const auto ttNew = walkingTime;
                if (ttNew < ttExisting) {
                    p = {reqId, vertexInLineIdx, walkingTime};
                }
            }
        }

        template<bool Starting>
        void initializePathsStartingOrEndingAtEdge(const PreliminaryPathsT &paths,
                                                   std::vector<int> &firstPathAtEdge,
                                                   std::vector<int> &pathsAtEdge) {
            KASSERT(firstPathAtEdge.size() == inputGraph.numEdges() + 1);
            KASSERT(std::all_of(firstPathAtEdge.begin(), firstPathAtEdge.end(),
                                [](const int i) { return i == 0; }));
            // Count number of paths starting at each edge:
            for (const auto &path: paths) {
                KASSERT(path.size() > 0);
                if constexpr (Starting) {
                    ++firstPathAtEdge[path.front()];
                } else {
                    ++firstPathAtEdge[path.back()];
                }
            }

            // Compute prefix sum:
            int sum = 0;
            for (int i = 0; i < firstPathAtEdge.size(); ++i) {
                const int tmp = firstPathAtEdge[i];
                firstPathAtEdge[i] = sum;
                sum += tmp;
            }
            KASSERT(firstPathAtEdge.back() == paths.numPaths());

            // Write request IDs to right spots, using firstPathStartingAtEdge[e] as counter for edge e.
            pathsAtEdge.resize(paths.numPaths());
            for (const auto &path: paths) {
                const auto e = Starting ? path.front() : path.back();
                pathsAtEdge[firstPathAtEdge[e]] = path.getPathId();
                ++firstPathAtEdge[e];
            }

            // Restore indices in firstPathStartingAtEdge:
            for (int i = firstPathAtEdge.size() - 1; i > 0; --i) {
                firstPathAtEdge[i] = firstPathAtEdge[i - 1];
            }
            firstPathAtEdge[0] = 0;

            verifyPathsStartingOrEndingAtEdge<Starting>(paths, firstPathAtEdge, pathsAtEdge);
        }

        void updatePathsStartingOrEndingAtEdge(const PreliminaryPathsT &paths,
                                               std::vector<int> &firstPathAtEdge,
                                               std::vector<int> &pathsAtEdge) {
            // Iterate through edges and remove all requests who no longer have a path in paths:
            int numRemoved = 0;
            int curEdge = 0;
            for (int readIdx = 0; readIdx < pathsAtEdge.size(); ++readIdx) {
                // If we reach end of range of current edge, update end of range and move on to next edge:
                while (firstPathAtEdge[curEdge + 1] <= readIdx) {
                    firstPathAtEdge[curEdge + 1] -= numRemoved;
                    ++curEdge;
                }
                const int writeIdx = readIdx - numRemoved;
                KASSERT(firstPathAtEdge[curEdge] <= writeIdx);

                // Check if path is still in paths and possibly remove it:
                const auto reqId = pathsAtEdge[readIdx];
                if (paths.hasPathFor(reqId)) {
                    pathsAtEdge[writeIdx] = reqId;
                } else {
                    ++numRemoved;
                }
            }
            KASSERT(pathsAtEdge.size() - numRemoved == paths.numPaths());
            while (curEdge < inputGraph.numEdges()) {
                firstPathAtEdge[curEdge + 1] -= numRemoved;
                ++curEdge;
            }
            pathsAtEdge.erase(pathsAtEdge.end() - numRemoved, pathsAtEdge.end());
            KASSERT(firstPathAtEdge.back() == paths.numPaths());
        }

        template<bool Starting>
        void verifyPathsStartingOrEndingAtEdge(const PreliminaryPathsT &paths,
                                               const std::vector<int> &firstPathAtEdge,
                                               const std::vector<int> &pathsAtEdge) const {
            // Verify:
            for (int i = 0; i < firstPathAtEdge.size() - 1; ++i) {
                for (int j = firstPathAtEdge[i]; j < firstPathAtEdge[i + 1]; ++j) {
                    if constexpr (Starting)
                        KASSERT(paths.getPathFor(pathsAtEdge[j]).front() == i,
                                "Path does not start at edge it is supposed to start at.");
                    else
                        KASSERT(paths.getPathFor(pathsAtEdge[j]).back() == i,
                                "Path does not end at edge it is supposed to end at.");
                }
            }
        }

        // Verify that every fully covered path has a possible pickup and dropoff location on the line
        bool verifyFullyCoveredPathsPickupsAndDropoffs(const std::vector<int> &verticesInLine,
                                                       const std::vector<int> &fullyCoveredPaths) {
            int numFoundPickup = 0;
            BitVector foundPickup(fullyCoveredPaths.size());
            int numFoundDropoff = 0;
            BitVector foundDropoff(fullyCoveredPaths.size());
            for (const auto &v: verticesInLine) {
                const auto &possiblePickups = pdInfo.getPossiblePickupsAt(v);
                const auto &possibleDropoffs = pdInfo.getPossibleDropoffsAt(v);
                for (int i = 0; i < fullyCoveredPaths.size(); ++i) {
                    const auto &reqId = fullyCoveredPaths[i];
                    if (!foundPickup[i] && contains(possiblePickups.begin(), possiblePickups.end(), reqId)) {
                        foundPickup[i] = true;
                        ++numFoundPickup;
                    }
                    if (!foundDropoff[i] && contains(possibleDropoffs.begin(), possibleDropoffs.end(), reqId)) {
                        foundDropoff[i] = true;
                        ++numFoundDropoff;
                    }
                }

                if (numFoundPickup == fullyCoveredPaths.size() && numFoundDropoff == fullyCoveredPaths.size())
                    break;
            }
            return numFoundPickup == fullyCoveredPaths.size() && numFoundDropoff == fullyCoveredPaths.size();
        }

        void
        logLine(const FixedLine &line, const int lineId, const int initialEdge, const int maxFlow,
                const std::vector<ServedRequest> &pax, const int numFullyCovered, const Subset &tripTimeViolators,
                const RunningTimePerLineStats &runningTimeStats) {

            uint64_t totalTravelTime = 0;
            for (const auto &e: line)
                totalTravelTime += inputGraph.travelTime(e);

            int64_t sumActualTT = 0;
            int64_t sumDirectTT = 0;
            int64_t sumPickupWalkingTime = 0;
            int64_t sumDropoffWalkingTime = 0;
            int64_t sumWalkingTimeDiff = 0;
            double sumTTWeightedAvgSharing = 0.0;
            for (const auto &p: pax) {
                sumActualTT += p.pickupWalkingTime + p.inVehicleTime + p.dropoffWalkingTime;
                sumDirectTT += requests[p.requestId].directDist;
                sumTTWeightedAvgSharing += p.ttWeightedAvgSharing;
                sumPickupWalkingTime += p.pickupWalkingTime;
                sumDropoffWalkingTime += p.dropoffWalkingTime;
                sumWalkingTimeDiff += std::abs(p.pickupWalkingTime - p.dropoffWalkingTime);
            }
            const double avgDirectTT = pax.empty() ? 0 : static_cast<double>(sumDirectTT) /
                                                         static_cast<double>(pax.size());
            const double avgRelDetour = static_cast<double>(sumActualTT) / static_cast<double>(sumDirectTT);
            const double avgAbsDetour = pax.empty() ? 0 :
                                        static_cast<double>(sumActualTT - sumDirectTT) /
                                        static_cast<double>(pax.size());
            const double avgTTWeightedAvgSharing = pax.empty() ? 0 : sumTTWeightedAvgSharing /
                                                                     static_cast<double>(pax.size());
            const double avgPickupWalkingTime = pax.empty() ? 0 : static_cast<double>(sumPickupWalkingTime) /
                                                                  static_cast<double>(pax.size());
            const double avgDropoffWalkingTime = pax.empty() ? 0 : static_cast<double>(sumDropoffWalkingTime) /
                                                                   static_cast<double>(pax.size());
            const double avgWalkingTimeDiff = pax.empty() ? 0 : static_cast<double>(sumWalkingTimeDiff) /
                                                                static_cast<double>(pax.size());

            // Compute average occupancy weighted with travel time
            uint64_t ttWeightedOcc = 0;
            for (const auto &p: pax)
                for (int i = p.pickupVertexIdx; i < p.dropoffVertexIdx; ++i)
                    ttWeightedOcc += inputGraph.travelTime(line[i]);

            const double avgTTWeightedOcc = static_cast<double>(ttWeightedOcc) / static_cast<double>(totalTravelTime);


            lineOverviewLogger << lineId << ", "
                               << initialEdge << ","
                               << maxFlow << ","
                               << line.size() << ", "
                               << totalTravelTime << ", "
                               << pax.size() << ", "
                               << numFullyCovered << ", "
                               << tripTimeViolators.size() << ", "
                               << avgDirectTT << ", "
                               << avgRelDetour << ", "
                               << avgAbsDetour << ", "
                               << avgPickupWalkingTime << ", "
                               << avgDropoffWalkingTime << ", "
                               << avgWalkingTimeDiff << ", "
                               << avgTTWeightedAvgSharing << ", "
                               << avgTTWeightedOcc << ", "
                               << runningTimeStats.constructLineTime << ", "
                               << runningTimeStats.findServedPaxTime << ", "
                               << runningTimeStats.updatePathsTime << "\n";
        }


        const VehicleInputGraphT &inputGraph;
        const VehicleInputGraphT &reverseGraph;
        const PickupDropoffInfo &pdInfo;
        const std::vector<Request> &requests;
        const InputConfig &inputConfig;

        TimestampedVector<int> residualFlow;

        // For every edge e, the IDs of not-yet-served requests whose paths start at e are stored in
        // pathsStartingAtEdge[firstPathStartingAtEdge[e]..firstPathStartingAtEdge[e + 1]].
        std::vector<int> firstPathStartingAtEdge;
        std::vector<int> pathsStartingAtEdge;

        // For every edge e, the IDs of not-yet-served requests whose paths end at e are stored in
        // pathsEndingAtEdge[firstPathEndingAtEdge[e]..firstPathEndingAtEdge[e + 1]].
        std::vector<int> firstPathEndingAtEdge;
        std::vector<int> pathsEndingAtEdge;

        OverviewLoggerT &lineOverviewLogger;

    };

} // end namespace