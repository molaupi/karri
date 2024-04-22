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
#include "ServabilityChecker.h"

namespace mixfix {

    // Given a set of OD-pairs and paths between each origin and destination in a road network, this facility
    // constructs a set of fixed lines with the goal of serving as many passengers as possible, i.e. s.t. the
    // number of OD-pairs (o,d) for which both o and d lie in the vicinity of at least one single line is maximized.
    //
    // This greedy algorithm is based on Andres Fielbaum, Javier Alonso-Mora, "Design of mixed fixed-flexible bus
    // public transport networks by tracking the paths of on-demand vehicles", Transportation Research Part C: Emerging
    // Technologies, 2024, https://doi.org/10.1016/j.trc.2024.104580.
    template<typename VehicleInputGraphT,
            typename PreliminaryPathsT>
    class GreedyFixedLineFinder {

    public:

        // Finds fixed lines given preliminary rider paths.
        // Each path is expected to be a sequence of edges in the network.
        std::vector<FixedLine> findFixedLines(PreliminaryPathsT &paths) {

            while (true) {

                const auto &[line, flow] = constructNextLine(paths);

                if (flow < inputConfig.minMaxFlowOnLine)
                    break;

                const auto &pax = findPassengersServedByLine(line);

            }

        }


    private:

        using Path = typename PreliminaryPathsT::Path;

        struct OverlappingPath {

            OverlappingPath(const Path &path, const int start, const int end) : path(path), start(start), end(end) {}

            const Path &path;
            int start; // smallest known index in path of passenger where path overlaps with line
            int end; // (one past) largest known index in path of passenger where path overlaps with line
        };

        // Greedily constructs new line from remaining paths. Returns line and max flow on line.
        std::pair<FixedLine, int> constructNextLine(PreliminaryPathsT &paths) {
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

            FixedLine line = {maxFlowEdge};
            std::vector<OverlappingPath> overlapping;
            for (const auto &path: paths) {
                for (int i = 0; i < path.size(); ++i) {
                    if (path[i] == maxFlowEdge) {
                        overlapping.push_back({path.getRequestId(), i, i + 1});
                        break;
                    }
                }
                if (overlapping.size() == maxFlow)
                    break;
            }

            // Construct line by greedily extending in both directions.
            // TODO: Try extending by one edge forward and backward in alternating fashion.
            extendLineBackwards(line, maxFlow, overlapping, paths);
            extendLineForwards(line, maxFlow, overlapping, paths);

            return {line, maxFlow};
        }

        static inline int square(const int n) { return n * n; }

        void extendLineForwards(FixedLine &line,
                                int &maxFlowOnLine,
                                std::vector<OverlappingPath> &overlapping,
                                const PreliminaryPathsT &paths) {

            // Begin extending
            double flowDif = 0.0;
            while (flowDif < inputConfig.maxFlowDif) {
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
                        const auto &path = o.path;
                        if (o.end < path.size() && path[o.end] == e) {
                            score += 1 + square(o.end - o.start);
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

                // If flow difference (i.e. difference between least and most flow) has become too large, stop extending
                flowDif = static_cast<double>(flowOnExtension) /
                          static_cast<double>(std::max(maxFlowOnLine, flowOnExtension));
                if (flowDif >= inputConfig.maxFlowDif)
                    break;

                // If extension edge is already part of line, stop extending
                for (const auto &e: line)
                    if ((inputGraph.edgeHead(e) == inputGraph.edgeHead(extension) &&
                         inputGraph.edgeTail(e) == inputGraph.edgeTail(extension)) ||
                        (inputGraph.edgeHead(e) == inputGraph.edgeTail(extension) &&
                         inputGraph.edgeTail(e) == inputGraph.edgeHead(extension)))
                        break;

                // Remove overlapping paths that do not overlap with extension
                int i = 0;
                while (i < overlapping.size()) {
                    const auto &o = overlapping[i];
                    const auto &path = o.path;
                    if (o.end < path.size() && path[o.end] == extension) {
                        ++o.end;
                        ++i;
                    } else {
                        std::swap(overlapping[i], overlapping.back());
                        overlapping.pop_back();
                    }
                }

                // Add overlapping paths that start at extension
                int numOnLine = overlapping.size();
                // TODO: Sort paths by first edge to find these in O(log |paths|)
                //  (or store them while counting flow in FORALL_INCIDENT_EDGES loop)
                for (const auto &path: paths) {
                    if (path.front() == extension) {
                        overlapping.push_back({path, 0, 1});
                        if (++numOnLine == flowOnExtension)
                            break;
                    }
                }

                // Add extension
                line.push_back(extension);
                maxFlowOnLine = std::max(maxFlowOnLine, flowOnExtension);
            }
        }

        void extendLineBackwards(FixedLine &line,
                                 int &maxFlowOnLine,
                                 std::vector<OverlappingPath> &overlapping,
                                 const PreliminaryPathsT &paths) {

            // Get reverse representation of line: Order of edges is reversed and edges are transformed to edges in
            // reverseGraph.
            std::reverse(line.begin(), line.end());
            for (auto &e: line) {
                FORALL_INCIDENT_EDGES(reverseGraph, inputGraph.edgeHead(e), eInRev) {
                    if (reverseGraph.edgeId(eInRev) == e) {
                        e = eInRev;
                        break;
                    }
                }
                KASSERT(false, "No reverse edge found for " << e << "!", kassert::assert::light);
            }

            // Begin extending
            double flowDif = 0.0;
            while (flowDif < inputConfig.maxFlowDif) {
                const auto v = reverseGraph.edgeHead(line.back());

                // Search for edge on which to extend line. Edge is chosen using a scoring system.
                int maxScore = 0;
                int extension = INVALID_EDGE;
                int flowOnExtension = 0;
                FORALL_INCIDENT_EDGES(reverseGraph, v, e) {
                    int score = 0; // Each path that overlaps with the edge contributes to the score as described below
                    int flow = 0; // Each path that overlaps with the edge contributes to the load with a value of one

                    // Paths already on line whose previous edge is e increase the score by 1 + the square of the length of
                    // the current overlap between the line and the path.
                    for (const auto &o: overlapping) {
                        const auto &path = o.path;
                        if (o.start > 0 && path[o.start - 1] == e) {
                            score += 1 + square(o.end - o.start);
                            ++flow;
                        }
                    }

                    // TODO: Sort paths by last edge to find these in O(log |paths|)
                    // Paths that end at e increase the score by 1.
                    for (const auto &path: paths) {
                        score += path.back() == e;
                        flow += path.back() == e;
                    }

                    if (score > maxScore) {
                        maxScore = score;
                        extension = e;
                        flowOnExtension = flow;
                    }
                }

                // If flow difference (i.e. difference between least and most flow) has become too large, stop extending
                flowDif = static_cast<double>(flowOnExtension) /
                          static_cast<double>(std::max(maxFlowOnLine, flowOnExtension));
                if (flowDif >= inputConfig.maxFlowDif)
                    break;

                // If extension edge is already part of line, stop extending
                for (const auto &e: line)
                    if ((reverseGraph.edgeHead(e) == reverseGraph.edgeHead(extension) &&
                         reverseGraph.edgeTail(e) == reverseGraph.edgeTail(extension)) ||
                        (reverseGraph.edgeHead(e) == reverseGraph.edgeTail(extension) &&
                         reverseGraph.edgeTail(e) == reverseGraph.edgeHead(extension)))
                        break;

                // Remove passengers on line whose paths do not overlap with extension
                int i = 0;
                while (i < overlapping.size()) {
                    const auto &o = overlapping[i];
                    const auto &path = o.path;
                    if (o.start > 0 && path[o.start - 1] == extension) {
                        --o.start;
                        ++i;
                    } else {
                        std::swap(overlapping[i], overlapping.back());
                        overlapping.pop_back();
                    }
                }

                // Add passengers whose paths start at extension
                int numOnLine = overlapping.size();
                // TODO: Sort paths by last edge to find these in O(log |paths|)
                //  (or store them while counting flow in FORALL_INCIDENT_EDGES loop)
                for (const auto &path: paths) {
                    if (path.back() == extension) {
                        overlapping.push_back({path.getRequestId(), path.size() - 1, path.size()});
                        if (++numOnLine == flowOnExtension)
                            break;
                    }
                }

                // Add extension
                line.push_back(extension);
                maxFlowOnLine = std::max(maxFlowOnLine, flowOnExtension);
            }

            // Convert line back into forwards representation
            std::reverse(line.begin(), line.end());
            for (auto &e: line)
                e = reverseGraph.edgeId(e);
        }

        std::vector<int> findPassengersServedByLine(const FixedLine &line, const Subset& paxToServe) {



        }


        const VehicleInputGraphT &inputGraph;
        const VehicleInputGraphT &reverseGraph;
        const std::vector<Request> requests;
        TimestampedVector<int> residualFlow;

    };

} // end namespace