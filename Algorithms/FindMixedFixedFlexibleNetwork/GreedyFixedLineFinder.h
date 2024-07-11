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
            typename OverviewLoggerT = NullLogger,
            typename FullPathLoggerT = NullLogger>
    class GreedyFixedLineFinder {

        struct ServedRequest {
            int requestId;
            int pickupVertexIdx;
            int pickupWalkingTime;
            int dropoffVertexIdx;
            int dropoffWalkingTime;
            int inVehicleTime;
            double ttWeightedAvgSharing;
        };

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
                                                                            "avg_rel_pax_detour,"
                                                                            "avg_abs_pax_detour,"
                                                                            "avg_tt_weighted_avg_sharing,"
                                                                            "construct_line_time,"
                                                                            "find_served_pax_time,"
                                                                            "update_paths_time\n")),
                  linePathLogger(LogManager<FullPathLoggerT>::getLogger("line_paths.csv",
                                                                        "line_id,"
                                                                        "path_as_edge_ids,"
                                                                        "path_as_lat_lng,"
                                                                        "served\n")) {}

        // Finds fixed lines given preliminary rider paths.
        // Each path is expected to be a sequence of edges in the network.
        void findFixedLines(PreliminaryPathsT &paths, const bool buildGeoJson = false) {

            lines.clear();
            if (buildGeoJson) {
                linesGeoJson.clear();
                linesGeoJson["type"] = "FeatureCollection";
            }

            initializePathsStartingOrEndingAtEdge<true>(paths, firstPathStartingAtEdge, pathsStartingAtEdge);
            initializePathsStartingOrEndingAtEdge<false>(paths, firstPathEndingAtEdge, pathsEndingAtEdge);

            BitVector isServable(requests.size(), false);
            std::vector<int> pickupableIdx(requests.size(), INVALID_INDEX);

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
                findInitialEdge(paths, initialEdge, maxFlowOnLine);
                std::cout << "Initial edge " << initialEdge
                          << " (tail: " << inputGraph.osmNodeId(inputGraph.edgeTail(initialEdge))
                          << ", head: " << inputGraph.osmNodeId(inputGraph.edgeHead(initialEdge))
                          << ") with flow " << maxFlowOnLine << std::endl;
                if (maxFlowOnLine < inputConfig.minMaxFlowOnLine) {
                    std::cout << "Stopping line construction due to insufficient flow on initial edge." << std::endl;
                    break;
                }
                constructNextLine(paths, line, initialEdge, maxFlowOnLine, fullyCoveredPaths);
                runningTimeStats.constructLineTime = timer.elapsed<std::chrono::nanoseconds>();

                // Find passengers served by line:
                timer.restart();
                std::vector<int> verticesInLine;
                verticesInLine.push_back(inputGraph.edgeTail(line.front()));
                for (const auto &e: line)
                    verticesInLine.push_back(inputGraph.edgeHead(e));

                KASSERT(verifyFullyCoveredPathsPickupsAndDropoffs(verticesInLine, fullyCoveredPaths),
                        "Line does not have possible pickup and dropoff locations for all fully covered paths.");

                const auto &pax = findPassengersServableByLine(line, verticesInLine, [&](const int reqId) {
                    return !paths.hasPathFor(reqId);
                }, fullyCoveredPaths, isServable, pickupableIdx);
                runningTimeStats.findServedPaxTime = timer.elapsed<std::chrono::nanoseconds>();
                std::cout << "Found a line of length " << line.size() << " that can serve " << pax.size()
                          << " passengers. (Num fully covered paths = " << fullyCoveredPaths.size() << ")" << std::endl;

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

                if (pax.size() < inputConfig.minNumPaxPerLine)
                    break;

                logLine(line, lines.size(), initialEdge, maxFlowOnLine, pax, fullyCoveredPaths.size(), runningTimeStats,
                        buildGeoJson);

                // Add line
                lines.push_back(std::make_pair(std::move(line), std::move(pax)));
            }
        }

        const std::vector<std::pair<FixedLine, std::vector<ServedRequest>>> &getLines() const {
            return lines;
        }

        const nlohmann::json &getLinesGeoJson() const {
            return linesGeoJson;
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
                               std::vector<int> &fullyCoveredPaths) {
            std::vector<OverlappingPath> overlappingPaths;
            std::vector<OverlappingPath> overlappingReachedEnd;
            line = {initialEdge};
            const auto pickupsAtTail = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(initialEdge));
            const auto dropoffsAtHead = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(initialEdge));
            for (const auto &path: paths) {
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == initialEdge) {
                        const bool possiblePickupReached = contains(pickupsAtTail.begin(), pickupsAtTail.end(),
                                                                    path.getRequestId());
                        const bool possibleDropoffReached = contains(dropoffsAtHead.begin(), dropoffsAtHead.end(),
                                                                     path.getRequestId());
                        if (possiblePickupReached && possibleDropoffReached) {
                            fullyCoveredPaths.push_back(path.getRequestId());
                        } else if (possibleDropoffReached) {
                            overlappingReachedEnd.push_back(
                                    {path.getRequestId(), i, i + 1, possiblePickupReached, possibleDropoffReached});
                        } else {
                            overlappingPaths.push_back(
                                    {path.getRequestId(), i, i + 1, possiblePickupReached, possibleDropoffReached});
                        }
                        break; // If path overlaps with edge in multiple places, only consider first overlap
                    }
                }
            }

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.
            // Paths that contain the initial edge whose end is reached by the forward extension but who may still be
            // extended toward their start by the backward extension
            extendLineForwards(line, flowOnInitialEdge, overlappingPaths, overlappingReachedEnd,
                               fullyCoveredPaths, paths);
            extendLineBackwards(line, flowOnInitialEdge, overlappingReachedEnd, fullyCoveredPaths, paths);
        }

        static uint64_t computeScoreForOverlapLength(const uint64_t overlapLength) {
            const auto res = std::pow(overlapLength, InputConfig::getInstance().overlapScoreExponent);
            return static_cast<uint64_t>(res);
        }

        template<
                typename DoesExtensionContinueOverlapT,
                typename CountOverlapsStartingAtExtensionT,
                typename ExtendOverlapIfPossibleT,
                typename StartNewOverlapsAtExtensionT
        >
        void extendLine(FixedLine &line,
                        const int &maxFlowOnLine,
                        std::vector<OverlappingPath> &overlapping,
                        const PreliminaryPathsT &paths,
                        const VehicleInputGraphT &graph,
                        const DoesExtensionContinueOverlapT &doesExtensionContinueOverlap,
                        const CountOverlapsStartingAtExtensionT &countOverlapsStartingAtExtension,
                        const ExtendOverlapIfPossibleT &extendOverlapIfPossible,
                        const StartNewOverlapsAtExtensionT &startNewOverlapsAtExtension) {

            // Begin extending
            double flowDif = 0.0;
            while (flowDif < inputConfig.maxFlowRatioOnLine) {
                const auto v = graph.edgeHead(line.back());

                // Search for edge on which to extend line. Edge is chosen using a scoring system.
                uint64_t maxScore = 0;
                int extension = INVALID_EDGE;
                int flowOnExtension = 0;
                FORALL_INCIDENT_EDGES(graph, v, e) {
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
                    const int numNewOverlaps = countOverlapsStartingAtExtension(paths, eInForwGraph);
                    score += numNewOverlaps;
                    flow += numNewOverlaps;

                    if (score > maxScore) {
                        maxScore = score;
                        extension = e;
                        flowOnExtension = flow;
                    }
                }

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


                // If extension edge is already part of line, stop extending
                bool extensionClosesLoop = false;

                // Check if edge (directional) is already on line
                for (const auto &e: line) {
                    if (e == extension) {
                        extensionClosesLoop = true;
                        break;
                    }
                }
                if (extensionClosesLoop)
                    break; // Stop extending line

                const int extensionInForwGraph = graph.edgeId(extension);
                const int newReachedVertex = graph.edgeHead(extension);

                // Remove passengers whose paths do not overlap with extension
                int i = 0;
                while (i < overlapping.size()) {
                    auto &o = overlapping[i];
                    const auto &path = paths.getPathFor(o.requestId);
                    if (extendOverlapIfPossible(o, path, extensionInForwGraph, newReachedVertex)) {
                        ++i;
                    } else {
                        // If overlapping path does not overlap with extension or a dropoff has been found,
                        // remove from overlapping paths.
                        std::swap(overlapping[i], overlapping.back());
                        overlapping.pop_back();
                    }
                }

                // Add passengers whose paths start at extension
                startNewOverlapsAtExtension(overlapping, paths, extensionInForwGraph, newReachedVertex);

                // Add extension
                line.push_back(extension);
            }
        }

        void extendLineForwards(FixedLine &line,
                                const int &maxFlowOnLine,
                                std::vector<OverlappingPath> &overlapping,
                                std::vector<OverlappingPath> &overlappingReachedEnd,
                                std::vector<int> &fullyCoveredPaths,
                                const PreliminaryPathsT &paths) {

            static auto earlierOverlapKnown = [](const int requestId, const std::vector<OverlappingPath> &overlapping) {
                return std::find_if(overlapping.begin(), overlapping.end(), [&](const OverlappingPath &o) {
                    return o.requestId == requestId;
                }) != overlapping.end();
            };

            // Hook function for checking if an extension continues an overlap with a path when deciding next extension.
            static auto doesExtensionContinueOverlap = [](const OverlappingPath &o, const Path &path,
                                                          const int extension) {
                KASSERT(o.end < path.size());
                return path[o.end] == extension;
            };

            // Hook function for counting the number of new overlaps when deciding next extension.
            auto countOverlapsStartingAtExtension = [this](const PreliminaryPathsT &, const int extension) {
                return firstPathStartingAtEdge[extension + 1] - firstPathStartingAtEdge[extension];
            };

            // Hook function for extending an overlap if possible for a chosen extension.
            // Returns true if overlap has been extended and should be kept as overlapping or false if it
            // should be removed.
            const auto extendOverlapIfPossible = [this, &fullyCoveredPaths, &overlappingReachedEnd](
                    OverlappingPath &o,
                    const Path &path,
                    const int extension,
                    const int newReachedVertex) {
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
            // If extension starts an overlap with the path, we do not know an overlap already, and the path is not
            // fully covered right away, the overlap is added to overlapping.
            const auto startNewOverlapsAtExtension = [this, &fullyCoveredPaths](
                    std::vector<OverlappingPath> &overlapping,
                    const PreliminaryPathsT &paths,
                    const int extension,
                    const int newReachedVertex) {
                const auto &dropoffsAtNewVertex = pdInfo.getPossibleDropoffsAt(newReachedVertex);
                for (int idx = firstPathStartingAtEdge[extension];
                     idx < firstPathStartingAtEdge[extension + 1]; ++idx) {
                    const auto &path = paths.getPathFor(pathsStartingAtEdge[idx]);
                    KASSERT(path.front() == extension);
                    KASSERT(contains(pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension)).begin(),
                                     pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(extension)).end(),
                                     path.getRequestId()),
                            "Path that begins at extension does not have a pickup at the tail of the extension.");

                    // If we already know an earlier overlap of the path with the line, we only keep the earlier one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (earlierOverlapKnown(path.getRequestId(), overlapping))
                        continue;

                    if (contains(dropoffsAtNewVertex.begin(), dropoffsAtNewVertex.end(), path.getRequestId())) {
                        fullyCoveredPaths.push_back(path.getRequestId());
                        continue;
                    }

                    overlapping.push_back({path.getRequestId(), 0, 1, true, false});
                }
            };

            // Execute line extension with the forward extension hooks
            extendLine(line, maxFlowOnLine, overlapping, paths, inputGraph, doesExtensionContinueOverlap,
                       countOverlapsStartingAtExtension, extendOverlapIfPossible, startNewOverlapsAtExtension);
        }


        void extendLineBackwards(FixedLine &line,
                                 const int &maxFlowOnLine,
                                 std::vector<OverlappingPath> &overlapping,
                                 std::vector<int> &fullyCoveredPaths,
                                 const PreliminaryPathsT &paths) {

            static auto laterOverlapKnown = [](const int requestId, const std::vector<OverlappingPath> &overlapping) {
                return std::find_if(overlapping.begin(), overlapping.end(), [&](const OverlappingPath &o) {
                    return o.requestId == requestId;
                }) != overlapping.end();
            };

            // Hook function for checking if an extension continues an overlap with a path when deciding next extension.
            static auto doesExtensionContinueOverlap = [](const OverlappingPath &o, const Path &path,
                                                          const int extension) {
                KASSERT(o.start > 0);
                return path[o.start - 1] == extension;
            };

            // Hook function for checking if an extension starts an overlap with a path when deciding next extension.
            static auto countOverlapsStartingAtExtension = [this](const PreliminaryPathsT &, const int extension) {
                return firstPathEndingAtEdge[extension + 1] - firstPathEndingAtEdge[extension];
            };

            // Hook function for extending an overlap if possible for a chosen extension.
            // Returns true if overlap has been extended and should be kept as overlapping or false if it
            // should be removed.
            const auto extendOverlapIfPossible = [this, &fullyCoveredPaths](
                    OverlappingPath &o,
                    const Path &path,
                    const int extension,
                    const int newReachedVertex) {
                KASSERT(o.start > 0);
                KASSERT(o.possibleDropoffReached);
                const auto &pickupsAtNewVertex = pdInfo.getPossiblePickupsAt(newReachedVertex);
                if (path[o.start - 1] != extension)
                    return false;

                --o.start;
                if (!contains(pickupsAtNewVertex.begin(), pickupsAtNewVertex.end(), o.requestId)) {
                    // If path continues on extension but no possible pickup has been reached, keep it as overlapping path
                    return true;
                }
                // If a possible pickup has been reached, the path is fully covered.
                fullyCoveredPaths.push_back(o.requestId);
                return false;
            };

            // Hook function for starting overlaps for a chosen extension.
            // If extension starts an overlap with the path, we do not know an overlap already, and the path is not
            // fully covered right away, the overlap is added to overlapping..
            const auto startNewOverlapsAtExtension = [this, &fullyCoveredPaths](
                    std::vector<OverlappingPath> &overlapping,
                    const PreliminaryPathsT &paths,
                    const int extension,
                    const int newReachedVertex) {
                const auto &pickupsAtNewVertex = pdInfo.getPossiblePickupsAt(newReachedVertex);
                for (int idx = firstPathEndingAtEdge[extension]; idx < firstPathEndingAtEdge[extension + 1]; ++idx) {
                    const auto &path = paths.getPathFor(pathsEndingAtEdge[idx]);
                    KASSERT(path.back() == extension);
                    KASSERT(contains(pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extension)).begin(),
                                     pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(extension)).end(),
                                     path.getRequestId()),
                            "Path that ends at extension does not have a dropoff at the head of the extension.");

                    // If we already know a later overlap of the path with the line, we only keep the later one
                    // and do not add a second overlap (happens if a path visits the same edge multiple times).
                    if (laterOverlapKnown(path.getRequestId(), overlapping))
                        continue;

                    if (contains(pickupsAtNewVertex.begin(), pickupsAtNewVertex.end(), path.getRequestId())) {
                        fullyCoveredPaths.push_back(path.getRequestId());
                        continue;
                    }

                    overlapping.push_back({path.getRequestId(), path.size() - 1, path.size(), false, true});
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
                       countOverlapsStartingAtExtension, extendOverlapIfPossible, startNewOverlapsAtExtension);

            // Convert line back into forwards representation
            std::reverse(line.begin(), line.end());
            for (auto &e: line)
                e = reverseGraph.edgeId(e);
        }


        template<typename IsPassengerAlreadyServedT>
        std::vector<ServedRequest>
        findPassengersServableByLine(const FixedLine &line,
                                     const std::vector<int> &verticesInLine,
                                     const IsPassengerAlreadyServedT &isPaxAlreadyServedByOtherLine,
                                     const std::vector<int> &,
                                     BitVector &servedByThisLine,
                                     std::vector<int> &pickupableIdx) const {
            KASSERT(servedByThisLine.cardinality() == 0);
            KASSERT(std::all_of(pickupableIdx.begin(), pickupableIdx.end(),
                                [](const int idx) { return idx == INVALID_INDEX; }));

            // We iterate over the vertices of the line from front to back, keeping track of a subset of requests P that
            // may be picked up by the vehicle before the current vertex.
            // When reaching vertex i, we perform the following:
            //  1. Remove a request r from P if the detour of r has become too large with vertex i.
            //  2. Check whether r can be dropped off at vertex i. If so, r can be served by the line. Remove r from P.
            //  3. Add all unserved requests that can be picked up at vertex i to P.
            //
            // There may be multiple possible pickup vertices on the same line for a request r. We always use the one with
            // the smallest travel time to the current vertex i, i.e. walking distance + current in-vehicle distance.

            // Build prefix sum of travel time along line for fast retrieval of travel time between any two vertices:
            std::vector<int> ttPrefixSum(verticesInLine.size(), 0);
            for (int i = 0; i < line.size(); ++i)
                ttPrefixSum[i + 1] = ttPrefixSum[i] + inputGraph.travelTime(line[i]);

            std::vector<ServedRequest> servableRequests;

            struct Pickupable {
                int requestId;
                int pickupVertexIdx;
                int walkingTime;
            };

            std::vector<Pickupable> pickupables;
            for (int i = 0; i < verticesInLine.size(); ++i) {
                const auto &v = verticesInLine[i];

                // 1. Remove pickup-able requests whose detour would grow too large with edge i:
                int k = 0;
                while (k < pickupables.size()) {
                    auto &p = pickupables[k];
                    const auto tt = p.walkingTime + (ttPrefixSum[i] - ttPrefixSum[p.pickupVertexIdx]);
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

                // 2. Check which pickup-able requests can be dropped off at edge i.
                const auto &dropoffsAtV = pdInfo.getPossibleDropoffsAt(v);
                const auto &dropoffWalkingTimes = pdInfo.getDropoffWalkingDistsAt(v);
                for (int j = 0; j < dropoffsAtV.size(); ++j) {
                    const auto reqId = dropoffsAtV[j];
                    if (pickupableIdx[reqId] == INVALID_INDEX)
                        continue;
                    auto &p = pickupables[pickupableIdx[reqId]];
                    KASSERT(p.requestId == reqId);
                    const auto inVehTime = ttPrefixSum[i] - ttPrefixSum[p.pickupVertexIdx];
                    const auto totalTT = p.walkingTime + inVehTime + dropoffWalkingTimes[j];
                    if (totalTT <= getMaxTravelTime(requests[p.requestId], inputConfig)) {
                        // Request can be served by line
                        servedByThisLine[reqId] = true;
                        servableRequests.push_back(
                                {reqId, p.pickupVertexIdx, p.walkingTime, i, dropoffWalkingTimes[j], inVehTime, 0.0});
                        p = pickupables.back();
                        pickupableIdx[p.requestId] = pickupableIdx[reqId];
                        pickupables.pop_back();
                        pickupableIdx[reqId] = INVALID_INDEX;
                    }
                }


                // 3. Add unserved passengers that may be picked up here:
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
                        pickupables.push_back({reqId, i, walkingTime});
                        continue;
                    }

                    // If request can already be picked up at earlier edge on line, only store vertex that provides
                    // better travel time (ignoring waiting times).
                    auto &p = pickupables[pickupableIdx[reqId]];
                    const auto ttExisting = p.walkingTime + ttPrefixSum[i] - ttPrefixSum[p.pickupVertexIdx];
                    const auto ttNew = walkingTime;
                    if (ttNew < ttExisting) {
                        p = {reqId, i, walkingTime};
                    }
                }
            }

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
                for (int i = served.pickupVertexIdx; i < served.dropoffVertexIdx; ++i) {
                    weightedNumOtherRiders += (numOccupants[i] - 1) * inputGraph.travelTime(line[i]);
                }
//                int64_t totalTT = served.pickupWalkingTime + served.inVehicleTime + served.dropoffWalkingTime;
                int64_t totalTT = served.inVehicleTime;
                served.ttWeightedAvgSharing =
                        static_cast<double>(weightedNumOtherRiders) / static_cast<double>(totalTT);
            }

            return servableRequests;
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
                pathsAtEdge[firstPathAtEdge[e]] = path.getRequestId();
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
                const std::vector<ServedRequest> &pax, const int numFullyCovered,
                const RunningTimePerLineStats &runningTimeStats,
                const bool buildGeoJson) {

            int totalTravelTime = 0;
            for (const auto &e: line)
                totalTravelTime += inputGraph.travelTime(e);

            int64_t sumActualTT = 0;
            int64_t sumDirectTT = 0;
            double sumTTWeightedAvgSharing = 0.0;
            for (const auto &p: pax) {
                sumActualTT += p.pickupWalkingTime + p.inVehicleTime + p.dropoffWalkingTime;
                sumDirectTT += requests[p.requestId].directDist;
                sumTTWeightedAvgSharing += p.ttWeightedAvgSharing;
            }
            const double avgRelDetour = static_cast<double>(sumActualTT) / static_cast<double>(sumDirectTT);
            KASSERT(pax.size() > 0);
            const double avgAbsDetour =
                    static_cast<double>(sumActualTT - sumDirectTT) / static_cast<double>(pax.size());
            const double avgTTWeightedAvgSharing = sumTTWeightedAvgSharing / static_cast<double>(pax.size());


            lineOverviewLogger << lineId << ", "
                               << initialEdge << ","
                               << maxFlow << ","
                               << line.size() << ", "
                               << totalTravelTime << ", "
                               << pax.size() << ", "
                               << numFullyCovered << ", "
                               << avgRelDetour << ", "
                               << avgAbsDetour << ", "
                               << avgTTWeightedAvgSharing << ", "
                               << runningTimeStats.constructLineTime << ", "
                               << runningTimeStats.findServedPaxTime << ", "
                               << runningTimeStats.updatePathsTime << "\n";


            linePathLogger << lineId << ", ";
            // Log path as edge Ids:
            for (int i = 0; i < line.size(); ++i)
                linePathLogger << line[i] << (i < line.size() - 1 ? " : " : ", ");

            // Log path as latlngs
            std::vector<LatLng> latLngPath;
            for (int i = 0; i < line.size(); ++i) {
                const auto e = line[i];
                const auto tail = inputGraph.edgeTail(e);
                const auto latLng = inputGraph.latLng(tail);
                linePathLogger << latLngForCsv(latLng) << " : ";
                latLngPath.push_back(latLng);
            }
            if (!line.empty()) {
                const auto latLng = inputGraph.latLng(inputGraph.edgeHead(line.back()));
                linePathLogger << latLngForCsv(latLng);
                latLngPath.push_back(latLng);
            }
            linePathLogger << ", ";
            // Log served passengers
            for (int i = 0; i < pax.size(); ++i)
                linePathLogger << pax[i].requestId << (i < pax.size() - 1 ? " : " : "\n");


            if (buildGeoJson) {
                // Also write GeoJson for line:
                addGeoJsonFeaturesForLine(latLngPath, lineId, pax, linesGeoJson);
            }
        }

        void addGeoJsonFeaturesForLine(const std::vector<LatLng> &latLngPath, const int lineId,
                                       const std::vector<ServedRequest> &pax,
                                       nlohmann::json &featureCol) {
            static char lastEdgeColor[] = "red";


            // LineString feature for the line's edges
            nlohmann::json lineFeature;
            lineFeature["type"] = "Feature";
            lineFeature["geometry"] = {{"type",        "LineString"},
                                       {"coordinates", nlohmann::json::array()}};

            for (const auto &latLng: latLngPath) {
                const auto coord = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
                lineFeature["geometry"]["coordinates"].push_back(coord);
            }
            lineFeature["properties"] = {{"line_id", lineId},
                                         {"num_pax", pax.size()}};
            featureCol["features"].push_back(lineFeature);

            // MultiPoint feature for start and end of line
            nlohmann::json startFeature;
            startFeature["type"] = "Feature";
            startFeature["geometry"] = {{"type",        "MultiPoint"},
                                        {"coordinates", nlohmann::json::array()}};
            const auto firstEdgeTail = latLngPath.front();
            const auto lastEdgeHead = latLngPath.back();
            startFeature["geometry"]["coordinates"].push_back({firstEdgeTail.lngInDeg(), firstEdgeTail.latInDeg()});
            startFeature["geometry"]["coordinates"].push_back({lastEdgeHead.lngInDeg(), lastEdgeHead.latInDeg()});
            startFeature["properties"] = {{"stroke",  lastEdgeColor},
                                          {"line_id", lineId}};
            featureCol["features"].push_back(startFeature);
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

        std::vector<std::pair<FixedLine, std::vector<ServedRequest>>> lines;
        nlohmann::json linesGeoJson;

        OverviewLoggerT &lineOverviewLogger;
        FullPathLoggerT &linePathLogger;

    };

} // end namespace