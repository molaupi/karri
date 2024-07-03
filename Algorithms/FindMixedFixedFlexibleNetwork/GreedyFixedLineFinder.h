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
            typename LoggerT = NullLogger>
    class GreedyFixedLineFinder {

        struct ServedRequest {
            int requestId;
            int pickupVertexIdx;
            int pickupWalkingTime;
            int dropoffVertexIdx;
            int dropoffWalkingTime;
        };

    public:

        GreedyFixedLineFinder(const VehicleInputGraphT &inputGraph, const VehicleInputGraphT &reverseGraph,
                              const PickupDropoffInfo &pdInfo, const std::vector<Request> &requests)
                : inputGraph(inputGraph), reverseGraph(reverseGraph),
                  pdInfo(pdInfo), requests(requests), inputConfig(InputConfig::getInstance()),
                  residualFlow(inputGraph.numEdges(), 0),
                  linesLogger(LogManager<LoggerT>::getLogger("lines.csv",
                                                             "line_id, "
                                                             "num_edges, "
                                                             "path_as_edge_ids, "
                                                             "path_as_lat_lng, "
                                                             "num_pax_served, "
                                                             "served\n")) {}

        // Finds fixed lines given preliminary rider paths.
        // Each path is expected to be a sequence of edges in the network.
        void findFixedLines(PreliminaryPathsT &paths) {

            lines.clear();
            linesGeoJson.clear();
            linesGeoJson["type"] = "FeatureCollection";

            while (!paths.empty()) {
                std::cout << "\n\n";
                FixedLine line;
                int maxFlowOnLine;
                std::vector<int> fullyCoveredPaths;
                constructNextLine(paths, line, maxFlowOnLine, fullyCoveredPaths);

                if (maxFlowOnLine < inputConfig.minMaxFlowOnLine)
                    break;

                const auto &pax = findPassengersServableByLine(line, [&](const int reqId) {
                    return !paths.hasPathFor(reqId);
                });
                std::cout << "Found a line of length " << line.size() << " that can serve " << pax.size()
                          << " passengers." << std::endl;
                if (pax.size() < inputConfig.minNumPaxPerLine)
                    break;


                int numServedButNotFullyCovered = 0;
                for (const auto &served: pax) {
                    if (!contains(fullyCoveredPaths.begin(), fullyCoveredPaths.end(), served.requestId)) {
//                        std::cout << "Passenger " << served.requestId << " is served by line but their path does not overlap." << std::endl;
                        ++numServedButNotFullyCovered;
                    }
                    paths.removePathForRequest(served.requestId);
                }
                std::cout << "Number of passengers served whose paths are not fully covered by line (origin to destination): "
                          << numServedButNotFullyCovered << std::endl;

                logLine(line, lines.size(), pax);

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

            OverlappingPath(const int requestId, const int start, const int end) : requestId(requestId), start(start),
                                                                                   end(end) {}

            int requestId;
            int start; // smallest known index in path of passenger where path overlaps with line
            int end; // (one past) largest known index in path of passenger where path overlaps with line
        };

        // Greedily constructs new line from remaining paths. Returns line and max flow on line.
        void constructNextLine(PreliminaryPathsT &paths, FixedLine &line, int &maxFlowOnLine,
                               std::vector<int> &fullyCoveredPaths) {


            residualFlow.clear();
            int maxFlow = 0;
            int maxFlowEdge = INVALID_EDGE;
            for (const auto &path: paths) {
                for (const auto &e: path) {
                    ++residualFlow[e];
                    if (residualFlow[e] > maxFlow) {
                        maxFlowEdge = e;
                        maxFlow = residualFlow[e];
                    }
                }
            }

            std::vector<OverlappingPath> overlappingPaths;
            line = {maxFlowEdge};
            for (const auto &path: paths) {
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == maxFlowEdge) {
                        if (path.size() == 1) {
                            fullyCoveredPaths.push_back(path.getRequestId());
                        } else {
                            overlappingPaths.push_back({path.getRequestId(), i, i + 1});
                        }
                    }
                }
                if (overlappingPaths.size() == maxFlow)
                    break;
            }

            std::cout << "Line construction started at edge " << maxFlowEdge
                      << " (tail: " << inputGraph.osmNodeId(inputGraph.edgeTail(maxFlowEdge))
                      << ", head: " << inputGraph.osmNodeId(inputGraph.edgeHead(maxFlowEdge))
                      << ") with flow " << maxFlow << std::endl;

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.
            maxFlowOnLine = maxFlow;
            int minFlowOnLine = maxFlow;
            // Paths that contain the initial edge whose end is reached by the forward extension but who may still be
            // extended toward their start by the backward extension
            std::vector<OverlappingPath> overlappingReachedEnd;
            extendLineForwards(line, maxFlowOnLine, minFlowOnLine, overlappingPaths, overlappingReachedEnd,
                               fullyCoveredPaths, paths);
            extendLineBackwards(line, maxFlowOnLine, minFlowOnLine, overlappingReachedEnd, fullyCoveredPaths, paths);
        }

        static inline int square(const int n) { return n * n; }

        void extendLineForwards(FixedLine &line,
                                const int &maxFlowOnLine,
                                int &minFlowOnLine,
                                std::vector<OverlappingPath> &overlapping,
                                std::vector<OverlappingPath> &overlappingReachedEnd,
                                std::vector<int> &fullyCoveredPaths,
                                const PreliminaryPathsT &paths) {

            // Begin extending
//            double flowDif = static_cast<double>(maxFlowOnLine) / static_cast<double>(minFlowOnLine);
            double flowDif = 0.0;
            while (flowDif < inputConfig.maxFlowRatioOnLine) {
                const auto v = inputGraph.edgeHead(line.back());

                // Search for edge on which to extend line. Edge is chosen using a scoring system.
                int maxScore = 0;
                int extension = INVALID_EDGE;
                int flowOnExtension = 0;
                FORALL_INCIDENT_EDGES(inputGraph, v, e) {
                    int score = 0; // Each path that overlaps with the edge contributes to the score as described below
                    int flow = 0; // Each path that overlaps with the edge contributes to the load with a value of one

                    // Paths already on line whose next edge is e increase the score by 1 + the square of the length of
                    // the current overlap between the line and the path.
                    for (const auto &o: overlapping) {
                        const auto &path = paths.getPathFor(o.requestId);
                        if (o.end < path.size() && path[o.end] == e) {
                            const int overlapLength = 1 + o.end - o.start;
                            score += square(overlapLength);
                            ++flow;
                        }
                    }

                    // TODO: Sort paths by first edge to find these in O(log |paths|)
                    // Paths that begin at e increase the score by 1.
                    for (const auto &path: paths) {
                        score += path.front() == e;
                        flow += path.front() == e;
                    }

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
//
//                if (inputGraph.osmNodeId(inputGraph.edgeHead(extension)) == 266781215) {
//                    std::cout << "Forward: Found extension edge " << extension << " (tail: "
//                              << inputGraph.osmNodeId(inputGraph.edgeTail(extension)) << ", head: "
//                              << inputGraph.osmNodeId(inputGraph.edgeHead(extension)) << ") with flow "
//                              << flowOnExtension << std::endl;
//                }

                // If flow difference (i.e. difference between least and most flow) has become too large, stop extending
//                flowDif = static_cast<double>(std::max(maxFlowOnLine, flowOnExtension)) /
//                          static_cast<double>(std::min(minFlowOnLine, flowOnExtension));
                flowDif = static_cast<double>(maxFlowOnLine) /
                          static_cast<double>(flowOnExtension);

                // In the paper, they give this formula for the flow difference. The given intention is to stop
                // extending once the difference in flow becomes too large since this would imply bad utilization of
                // vehicle capacity in parts of the line. However, a line is started on the edge with most flow so
                // flowOnExtension will become smaller over time which means flowDif also becomes smaller over time.
//                flowDif = static_cast<double>(flowOnExtension) /
//                          static_cast<double>(std::max(maxFlowOnLine, flowOnExtension));
                if (flowDif >= inputConfig.maxFlowRatioOnLine)
                    break; // Stop extending line

                // If extension edge is already part of line, stop extending
                bool extensionClosesLoop = false;
//                const int newReachedVertex = inputGraph.edgeHead(extension);

//                // Check if edge (in either direction) is already on line
//                for (const auto &e: line) {
//                    if ((inputGraph.edgeHead(e) == inputGraph.edgeHead(extension) &&
//                         inputGraph.edgeTail(e) == inputGraph.edgeTail(extension))
//                        || (inputGraph.edgeHead(e) == inputGraph.edgeTail(extension) &&
//                            inputGraph.edgeTail(e) == inputGraph.edgeHead(extension))) {
//                        extensionClosesLoop = true;
//                        break;
//                    }
//                }
                // Check if edge (directional) is already on line
                for (const auto &e: line) {
                    if (e == extension) {
                        extensionClosesLoop = true;
                        break;
                    }
                }
                if (extensionClosesLoop)
                    break; // Stop extending line


                // Remove overlapping paths that do not overlap with extension
                int i = 0;
                while (i < overlapping.size()) {
                    auto &o = overlapping[i];
                    const auto &path = paths.getPathFor(o.requestId);
                    if (o.end < path.size() && path[o.end] == extension) {
                        ++o.end;
                        if (o.end < path.size()) {
                            ++i; // If path continues on extension but is not fully covered yet, keep it as overlapping path
                            continue;
                        }
                        if (o.start > 0)
                            // In case end of path is reached but the overlap started at the initial edge, the backward
                            // extension may still fully cover it.
                            overlappingReachedEnd.push_back(o);
                        else
                            // In case both the path is contained in the line from start to end, it is fully covered.
                            fullyCoveredPaths.push_back(o.requestId);
                    }
                    // If overlapping path does not overlap with extension or path has been fully covered by extension,
                    // remove from overlapping paths.
                    std::swap(overlapping[i], overlapping.back());
                    overlapping.pop_back();
                }

                // Add overlapping paths that start at extension
                int numOnLine = overlapping.size();
                // TODO: Sort paths by first edge to find these in O(log |paths|)
                //  (or store them while counting flow in FORALL_INCIDENT_EDGES loop)
                for (const auto &path: paths) {
                    if (path.front() == extension) {
                        if (path.size() == 1) {
                            fullyCoveredPaths.push_back(path.getRequestId());
                        } else {
                            overlapping.push_back({path.getRequestId(), 0, 1});
                        }
                        if (++numOnLine == flowOnExtension)
                            break;
                    }
                }

                // Add extension
                line.push_back(extension);
                minFlowOnLine = std::min(minFlowOnLine, flowOnExtension);
            }
        }

        void extendLineBackwards(FixedLine &line,
                                 const int &maxFlowOnLine,
                                 int &minFlowOnLine,
                                 std::vector<OverlappingPath> &overlapping,
                                 std::vector<int> &fullyCoveredPaths,
                                 const PreliminaryPathsT &paths) {

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

            // Begin extending
//            double flowDif = static_cast<double>(maxFlowOnLine) / static_cast<double>(minFlowOnLine);
            double flowDif = 0.0;
            while (flowDif < inputConfig.maxFlowRatioOnLine) {
                const auto v = reverseGraph.edgeHead(line.back());

                // Search for edge on which to extend line. Edge is chosen using a scoring system.
                int maxScore = 0;
                int extension = INVALID_EDGE;
                int flowOnExtension = 0;
                FORALL_INCIDENT_EDGES(reverseGraph, v, e) {
                    int score = 0; // Each path that overlaps with the edge contributes to the score as described below
                    int flow = 0; // Each path that overlaps with the edge contributes to the load with a value of one

                    int eInForwGraph = reverseGraph.edgeId(e);

                    // Paths already on line whose previous edge is e increase the score by 1 + the square of the length of
                    // the current overlap between the line and the path.
                    for (const auto &o: overlapping) {
                        const auto &path = paths.getPathFor(o.requestId);
                        LIGHT_KASSERT(o.start > 0);
                        if (path[o.start - 1] == eInForwGraph) {
                            const int overlapLength = 1 + o.end - o.start;
                            score += square(overlapLength);
                            ++flow;
                        }
                    }

                    // TODO: Sort paths by last edge to find these in O(log |paths|)
                    // Paths that end at e increase the score by 1.
                    for (const auto &path: paths) {
                        score += path.back() == eInForwGraph;
                        flow += path.back() == eInForwGraph;
                    }

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

//                if (reverseGraph.osmNodeId(reverseGraph.edgeHead(extension)) == 266781215) {
//                    std::cout << "Reverse: Found extension edge " << extension << " (tail: "
//                              << reverseGraph.osmNodeId(reverseGraph.edgeTail(extension)) << ", head: "
//                              << reverseGraph.osmNodeId(reverseGraph.edgeHead(extension)) << ") with flow "
//                              << flowOnExtension << std::endl;
//                }

                // If flow difference (i.e. difference between least and most flow) has become too large, stop extending
//                flowDif = static_cast<double>(std::max(maxFlowOnLine, flowOnExtension)) /
//                          static_cast<double>(std::min(minFlowOnLine, flowOnExtension));
                flowDif = static_cast<double>(maxFlowOnLine) /
                          static_cast<double>(flowOnExtension);

                // In the paper, they give this formula for the flow difference. The given intention is to stop
                // extending once the difference in flow becomes too large since this would imply bad utilization of
                // vehicle capacity in parts of the line. However, a line is started on the edge with most flow so
                // flowOnExtension will become smaller over time which means flowDif also becomes smaller over time.
                // Thus, this stopping criterion is never met.
//                flowDif = static_cast<double>(flowOnExtension) /
//                          static_cast<double>(std::max(maxFlowOnLine, flowOnExtension));
                if (flowDif >= inputConfig.maxFlowRatioOnLine)
                    break; // Stop extending line


                // If extension edge is already part of line, stop extending
                bool extensionClosesLoop = false;

                // Check if edge (in either direction) is already on line
//                const int newReachedVertex = reverseGraph.edgeHead(extension);
//                for (const auto &e: line) {
//                    if ((reverseGraph.edgeHead(e) == reverseGraph.edgeHead(extension) &&
//                         reverseGraph.edgeTail(e) == reverseGraph.edgeTail(extension))
//                        || (reverseGraph.edgeHead(e) == reverseGraph.edgeTail(extension) &&
//                            reverseGraph.edgeTail(e) == reverseGraph.edgeHead(extension))) {
//                        extensionClosesLoop = true;
//                        break;
//                    }
//                }
                // Check if edge (directional) is already on line
                for (const auto &e: line) {
                    if (e == extension) {
                        extensionClosesLoop = true;
                        break;
                    }
                }
                if (extensionClosesLoop)
                    break; // Stop extending line


                const int extensionInForwGraph = reverseGraph.edgeId(extension);

                // Remove passengers on line whose paths do not overlap with extension
                int i = 0;
                while (i < overlapping.size()) {
                    auto &o = overlapping[i];
                    const auto &path = paths.getPathFor(o.requestId);
                    if (o.start > 0 && path[o.start - 1] == extensionInForwGraph) {
                        --o.start;
                        if (o.start > 0 || o.end < path.size()) {
                            ++i; // If path continues on extension but is not fully covered yet, keep it as overlapping path
                            continue;
                        }
                        fullyCoveredPaths.push_back(o.requestId);
                    }
                    // If overlapping path does not overlap with extension or path has been fully covered by extension,
                    // remove from overlapping paths.
                    std::swap(overlapping[i], overlapping.back());
                    overlapping.pop_back();
                }


                // Add passengers whose paths start at extension
                int numOnLine = overlapping.size();
                // TODO: Sort paths by last edge to find these in O(log |paths|)
                //  (or store them while counting flow in FORALL_INCIDENT_EDGES loop)
                for (const auto &path: paths) {
                    if (path.back() == extensionInForwGraph) {
                        if (path.size() == 1) {
                            fullyCoveredPaths.push_back(path.getRequestId());
                        } else {
                            overlapping.push_back({path.getRequestId(), path.size() - 1, path.size()});
                        }
                        if (++numOnLine == flowOnExtension)
                            break;
                    }
                }

                // Add extension
                line.push_back(extension);
                minFlowOnLine = std::min(minFlowOnLine, flowOnExtension);
            }

            // Convert line back into forwards representation
            std::reverse(line.begin(), line.end());
            for (auto &e: line)
                e = reverseGraph.edgeId(e);
        }

        template<typename IsPassengerAlreadyServedT>
        std::vector<ServedRequest>
        findPassengersServableByLine(const FixedLine &line,
                                     const IsPassengerAlreadyServedT &isPaxAlreadyServedByOtherLine) {

            std::vector<int> verticesInLine;
            verticesInLine.push_back(inputGraph.edgeTail(line.front()));
            for (const auto &e: line)
                verticesInLine.push_back(inputGraph.edgeHead(e));

            // We iterate over the vertices of the line from front to back, keeping track of a subset of requests P that
            // may be picked up by the vehicle before the current vertex.
            // When reaching vertex i, we perform the following:
            //  1. Remove a request r from P if the detour of r has become too large with vertex i.
            //  2. Check whether r can be dropped off at vertex i. If so, r can be served by the line. Remove r from P.
            //  3. Add all unserved requests that can be picked up at vertex i to P.
            //
            // There may be multiple possible pickup vertices on the same line for a request r. We always use the one with
            // the smallest travel time to the current vertex i, i.e. walking distance + current in-vehicle distance.

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
                    const auto tt = pickupables[k].walkingTime +
                            getVehicleTravelTimeInEdgeInterval(line, pickupables[k].pickupVertexIdx, i);
                    if (tt > getMaxTravelTime(requests[pickupables[k].requestId], inputConfig)) {
                        std::swap(pickupables[k], pickupables.back());
                        pickupables.pop_back();
                        continue;
                    }
                    ++k;
                }

                // 2. Check which pickup-able requests can be dropped off at edge i.
                const auto &dropoffsAtV = pdInfo.getPossibleDropoffsAt(v);
                const auto &dropoffWalkingTimes = pdInfo.getDropoffWalkingDistsAt(v);
                for (int j = 0; j < dropoffsAtV.size(); ++j) {
                    const auto reqId = dropoffsAtV[j];
                    const auto pickupIt = std::find_if(pickupables.begin(), pickupables.end(),
                                                       [&](const auto &p) { return p.requestId == reqId; });
                    if (pickupIt == pickupables.end())
                        continue;
                    auto &p = *pickupIt;
                    const auto totalTT = p.walkingTime + getVehicleTravelTimeInEdgeInterval(line, p.pickupVertexIdx, i) +
                                         dropoffWalkingTimes[j];
                    if (totalTT <= getMaxTravelTime(requests[p.requestId], inputConfig)) {
                        // Request can be served by line
                        servableRequests.push_back({reqId, p.pickupVertexIdx, p.walkingTime, i, dropoffWalkingTimes[j]});
                        std::swap(p, pickupables.back());
                        pickupables.pop_back();
                    }
                }


                // 3. Add unserved passengers that may be picked up here:
                const auto &pickupsAtV = pdInfo.getPossiblePickupsAt(v);
                const auto &pickupWalkingTimes = pdInfo.getPickupWalkingDistsAt(v);
                for (int j = 0; j < pickupsAtV.size(); ++j) {
                    const auto &reqId = pickupsAtV[j];
                    if (isPaxAlreadyServedByOtherLine(reqId))
                        continue;
                    bool servedByThisLine = std::find_if(servableRequests.begin(), servableRequests.end(),
                                                         [&](const auto &p) { return p.requestId == reqId; }) !=
                                            servableRequests.end();
                    if (servedByThisLine)
                        continue;
                    const auto &walkingTime = pickupWalkingTimes[j];

                    const auto dupIt = std::find_if(pickupables.begin(), pickupables.end(),
                                                    [&](const auto &p) { return p.requestId == reqId; });
                    if (dupIt == pickupables.end()) {
                        pickupables.push_back({reqId, i, walkingTime});
                        continue;
                    }

                    // If request can already be picked up at earlier edge on line, only store vertex that provides
                    // better travel time (ignoring waiting times).
                    auto &p = *dupIt;
                    const auto ttExisting =
                            p.walkingTime + getVehicleTravelTimeInEdgeInterval(line, p.pickupVertexIdx, i);
                    const auto ttNew = walkingTime;
                    if (ttNew < ttExisting) {
                        p = {reqId, i, walkingTime};
                    }
                }
            }

            return servableRequests;
        }

        int getVehicleTravelTimeInEdgeInterval(const std::vector<int> &line, const int start, const int end) const {
            if (start >= end)
                return 0;
            return std::accumulate(line.begin() + start, line.begin() + end, 0, [&](const int a, const int b) {
                return a + inputGraph.travelTime(b);
            });
        }

        void logLine(const FixedLine &line, const int lineId, const std::vector<ServedRequest> &pax) {

            linesLogger << lineId << ", " << line.size() << ", ";

            // Log path as edge Ids:
            for (int i = 0; i < line.size(); ++i)
                linesLogger << line[i] << (i < line.size() - 1 ? " : " : ", ");

            // Log path as edge Ids:
            std::vector<LatLng> latLngPath;
            for (int i = 0; i < line.size(); ++i) {
                const auto e = line[i];
                const auto tail = inputGraph.edgeTail(e);
                const auto latLng = inputGraph.latLng(tail);
                linesLogger << latLngForCsv(latLng) << " : ";
                latLngPath.push_back(latLng);
            }
            if (!line.empty()) {
                const auto latLng = inputGraph.latLng(inputGraph.edgeHead(line.back()));
                linesLogger << latLngForCsv(latLng);
                latLngPath.push_back(latLng);
            }
            linesLogger << ", ";

            linesLogger << pax.size() << ", ";
            for (int i = 0; i < pax.size(); ++i)
                linesLogger << pax[i].requestId << (i < pax.size() - 1 ? " : " : "\n");

            // Also write GeoJson for line:
            addGeoJsonFeaturesForLine(latLngPath, lineId, pax, linesGeoJson);
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
            startFeature["properties"] = {{"stroke", lastEdgeColor},
                                              {"line_id", lineId}};
            featureCol["features"].push_back(startFeature);
        }


        const VehicleInputGraphT &inputGraph;
        const VehicleInputGraphT &reverseGraph;
        const PickupDropoffInfo &pdInfo;
        const std::vector<Request> &requests;
        const InputConfig &inputConfig;

        TimestampedVector<int> residualFlow;

        std::vector<std::pair<FixedLine, std::vector<ServedRequest>>> lines;
        nlohmann::json linesGeoJson;

        LoggerT &linesLogger;

    };

} // end namespace