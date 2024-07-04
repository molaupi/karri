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
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/ParentInfo.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "DataStructures/Utilities/OriginDestination.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"

inline void printUsage() {
    std::cout <<
              "Usage: ComputeFreeFlowPaths -g <file> -d <file> -o <file>\n\n"

              "Given a set of OD-pairs and a road network, computes the free-flow shortest paths for each pair and\n"
              "outputs them.\n\n"
              "  -g <file>         input graph in binary format\n"
              "  -d <file>         CSV file that contains OD pairs (columns 'origin' and 'destination')\n"
              "  -edges            If set, origins and destinations are assumed to be edge IDs instead of vertex IDs.\n"
              "  -traversal-costs  If set, computes shortest paths based on TraversalCostAttribute rather than TravelTimeAttribute.\n"
              "  -avoid-cost-zero  If set, adds 1 to all edge costs if any are zero in input.\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

// Some helper aliases.
using VertexAttributes = VertexAttrs<LatLngAttribute>;
using EdgeAttributes = EdgeAttrs<LengthAttribute, TravelTimeAttribute, TraversalCostAttribute, EdgeTailAttribute>;
using InputGraph = StaticGraph<VertexAttributes, EdgeAttributes>;
using LabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;


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
        const bool useCosts = clp.isSet("traversal-costs");
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
        bool hasZeroCostEdge = false;
        FORALL_VALID_EDGES(inputGraph, v, e) {
                inputGraph.edgeTail(e) = v;
                if ((useCosts && inputGraph.traversalCost(e) == 0) || (!useCosts && inputGraph.travelTime(e) == 0)) {
                    hasZeroCostEdge = true;
                    break;
                }
            }

            if (hasZeroCostEdge) {
                std::cout << "Warning: Graph contains edges with zero cost. Adding 1 to all edge costs.\nThis may affect the results (we technically need floating point costs).\n";
                FORALL_VALID_EDGES(inputGraph, v, e) {
                    if (useCosts) {
                        inputGraph.traversalCost(e) += 1;
                    } else {
                        inputGraph.travelTime(e) += 1;
                    }
                }
            }
        std::cout << "done." << std::endl;

        std::unique_ptr<CH> chPtr;
        std::cout << "Building CH... " << std::flush;
        chPtr = std::make_unique<CH>();
        if (useCosts) {
            chPtr->preprocess<TraversalCostAttribute>(inputGraph);
        } else {
            chPtr->preprocess<TravelTimeAttribute>(inputGraph);
        }
        std::cout << "done.\n";

        auto& ch = *chPtr;

        // Read the OD pairs from file.
        std::cout << "Reading OD pairs from file... " << std::flush;
        auto odPairs = importODPairsFrom(demandFileName);
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
            const auto& od = odPairs[i];
            int src = INVALID_VERTEX, dst = INVALID_VERTEX;
            if (!clp.isSet("edges")) {
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
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
