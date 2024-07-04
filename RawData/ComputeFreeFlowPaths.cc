/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler
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


#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <csv.h>

#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include "Algorithms/CH/CHQuery.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/TraversalCostAttribute.h"
#include "DataStructures/Graph/Attributes/FloatingPointTraversalCostAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/ParentInfo.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "DataStructures/Utilities/OriginDestination.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "RawData/floating_point_cch/cch.h"

inline void printUsage() {
    std::cout <<
              "Usage: ComputeFreeFlowPaths -g <file> -d <file> -o <file>\n\n"

              "Given a set of OD-pairs and a road network, computes the free-flow shortest paths for each pair and\n"
              "outputs them.\n\n"
              "  -g <file>                      input graph in binary format\n"
              "  -d <file>                      CSV file that contains OD pairs (columns 'origin' and 'destination')\n"
              "  -edges                         If set, origins and destinations are assumed to be edge IDs instead of vertex IDs.\n"
              "  -traversal-costs {int, float}  If set, computes shortest paths based on TraversalCostAttribute (int) or FloatingPointTraversalCostAttribute (float) rather than TravelTimeAttribute.\n"
              "  -avoid-cost-zero               If set, adds 1 to all edge costs if any are zero in input. Only if -traversal-costs=int.\n"
              "  -node-order <file>             Path to node order for the CCH. Only if -traversal-costs=float.\n"
              "  -o <file>                      place output in <file>\n"
              "  -help                          display this help and exit\n";
}

// Some helper aliases.
using VertexAttributes = VertexAttrs<LatLngAttribute>;
using EdgeAttributes = EdgeAttrs<LengthAttribute, TravelTimeAttribute, TraversalCostAttribute, FloatingPointTraversalCostAttribute, EdgeTailAttribute>;
using InputGraph = StaticGraph<VertexAttributes, EdgeAttributes>;
using LabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;

template<typename WeightT>
void computePathsUsingCH(const std::vector<OriginDestination>& odPairs, const InputGraph& inputGraph, const bool edges, const std::string& outputFileName) {
    std::cout << "Building CH... " << std::flush;
    CH ch;
    ch.preprocess<WeightT>(inputGraph);
    std::cout << "done.\n";

    // Run queries
    std::cout << "Running queries... " << std::flush;
    std::ofstream outputFile(outputFileName);
    if (!outputFile.good())
        throw std::invalid_argument("could not open file -- '" + outputFileName + "'");
    outputFile << "request_id, path_as_graph_edge_ids\n";
    ProgressBar progressBar(odPairs.size(), true);
    CHQuery<LabelSet> chQuery(ch);
    CHPathUnpacker pathUnpacker(ch);
    std::vector<int> path;
    for (int i = 0; i < odPairs.size(); ++i) {
        const auto &od = odPairs[i];
        int src = INVALID_VERTEX, dst = INVALID_VERTEX;
        if (!edges) {
            src = ch.rank(od.origin);
            dst = ch.rank(od.destination);
        } else {
            src = ch.rank(inputGraph.edgeHead(od.origin));
            dst = ch.rank(inputGraph.edgeHead(od.destination));
        }
        chQuery.run(src, dst);
        path.clear();
        pathUnpacker.unpackUpDownPath(chQuery.getUpEdgePath(), chQuery.getDownEdgePath(), path);
        outputFile << i << ", ";
//            if (clp.isSet("edges"))
//                outputFile << od.origin << " : ";
        for (int j = 0; j < path.size(); ++j) {
            if (j > 0) {
                outputFile << " : ";
            }
            outputFile << path[j];
        }
//            if (clp.isSet("edges"))
//                outputFile << (path.empty()? "" : " : ") << od.destination;
        outputFile << '\n';
        ++progressBar;
    }
    outputFile.close();
    std::cout << " done.\n";
}

void computePathsUsingFloatingPointCCH(const std::vector<OriginDestination>& odPairs, const InputGraph& inputGraph,
                                       const bool edges,
                                       const std::string& outputFileName,
                                       const std::string& nodeOrderFilePath) {
    std::cout << "Building and customizing CCH... " << std::flush;
    std::vector<floating_point_cch::node_id> heads(inputGraph.numEdges());
    std::vector<floating_point_cch::node_id> tails(inputGraph.numEdges());
    std::vector<floating_point_cch::weight_t> weights(inputGraph.numEdges());
    FORALL_VALID_EDGES(inputGraph, v, e) {
        heads[e] = inputGraph.edgeHead(e);
        tails[e] = v;
        weights[e] = inputGraph.template get<FloatingPointTraversalCostAttribute>(e);
    }

    std::vector<floating_point_cch::node_id> node_order = floating_point_cch::loadVector<floating_point_cch::node_id>(nodeOrderFilePath);
    floating_point_cch::CCH cch(node_order, tails, heads);
    floating_point_cch::CCH_metric metric(cch);
    metric.customize(weights, true);
    floating_point_cch::CCH_query query(metric);
    std::cout << "done.\n";

    // Run queries
    std::cout << "Running queries... " << std::flush;
    std::ofstream outputFile(outputFileName);
    if (!outputFile.good())
        throw std::invalid_argument("could not open file -- '" + outputFileName + "'");
    outputFile << "request_id, path_as_graph_edge_ids\n";
    ProgressBar progressBar(odPairs.size(), true);
    for (int i = 0; i < odPairs.size(); ++i) {
        const auto &od = odPairs[i];
        int src = INVALID_VERTEX, dst = INVALID_VERTEX;
        if (!edges) {
            src = od.origin;
            dst = od.destination;
        } else {
            src = inputGraph.edgeHead(od.origin);
            dst = inputGraph.edgeHead(od.destination);
        }
        query.query(src, dst);
        auto vertexPath = query.path();

        // Reconstruct edge path from vertex path and output
        outputFile << i << ", ";
        for (int j = 0; j + 1 < vertexPath.size(); ++j) {
            const auto tail = vertexPath[j];
            const auto head = vertexPath[j + 1];
            if (j > 0) {
                outputFile << " : ";
            }

            // Find edge from tail to head:
            int edge = INVALID_EDGE;
            FORALL_INCIDENT_EDGES(inputGraph, tail, e) {
                if (inputGraph.edgeHead(e) == head) {
                    edge = e;
                    break;
                }
            }
            if (edge == INVALID_EDGE) {
                throw std::runtime_error("edge from " + std::to_string(tail) + " to " + std::to_string(head) + " not found");
            }
            outputFile << edge;
        }
        outputFile << '\n';
        ++progressBar;
    }
    outputFile.close();
    std::cout << " done.\n";
}


int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        const auto graphFileName = clp.getValue<std::string>("g");
        const auto demandFileName = clp.getValue<std::string>("d");
        auto outputFileName = clp.getValue<std::string>("o");
        const auto useCosts = clp.getValue<std::string>("traversal-costs");
        const auto avoidCostZero = clp.isSet("avoid-cost-zero") && useCosts == "int";
        auto nodeOrderFileName = clp.getValue<std::string>("node-order");
        if (!endsWith(outputFileName, ".csv")) {
            outputFileName += ".csv";
        }

        // Read the graph from file.
        std::cout << "Reading full vehicle network from file... " << std::flush;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();

        // Set edge tails and check for zero cost edges (if integer costs):
        bool hasZeroCostEdge = false;
        FORALL_VALID_EDGES(inputGraph, v, e) {
                inputGraph.edgeTail(e) = v;
                if (avoidCostZero && inputGraph.traversalCost(e) == 0) {
                    hasZeroCostEdge = true;
                    break;
                }
            }

        if (hasZeroCostEdge) {
            std::cout << "Warning: Graph contains edges with zero cost. Adding 1 to all edge costs.\n"
                      << "This may affect the results (we technically need floating point costs).\n";
            FORALL_VALID_EDGES(inputGraph, v, e) {
                    inputGraph.traversalCost(e) += 1;
                }
        }
        std::cout << "done." << std::endl;


        // Read the OD pairs from file.
        std::cout << "Reading OD pairs from file... " << std::flush;
        auto odPairs = importODPairsFrom(demandFileName);
        std::cout << "done.\n";

        if (useCosts.empty()) {
            computePathsUsingCH<TravelTimeAttribute>(odPairs, inputGraph, clp.isSet("edges"), outputFileName);
        } else if (useCosts == "int") {
            computePathsUsingCH<TraversalCostAttribute>(odPairs, inputGraph, clp.isSet("edges"), outputFileName);
        } else if (useCosts == "float") {
            computePathsUsingFloatingPointCCH(odPairs, inputGraph, clp.isSet("edges"), outputFileName,
                                              nodeOrderFileName);
        } else {
            throw std::invalid_argument("invalid traversal cost type -- '" + useCosts + "'");
        }

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
