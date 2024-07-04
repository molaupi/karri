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


#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <csv.h>

#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInReducedVehAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/TraversalCostAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Attributes/FloatingPointTraversalCostAttribute.h"

inline void printUsage() {
    std::cout <<
              "Usage: AddStaTraversalCost -g <file> -f <file> -x <factor> -o <file>\n\n"

              "Given a road network and a flow f(e) on each edge, adds (or changes) the TraversalCostAttribute of the\n"
              "graph to reflect a synergistic traffic assignment cost function of the form \n"
              "c(e) = x * tt(e) + (1-x) * tt(e) / f(e) where tt(e) is the travel time on an edge\n"
              "and x is a configuration parameter. Outputs all attributes necessary for KaRRi plus TraversalCost.\n\n"

              "  -g <file>         input graph in binary format\n"
              "  -f <file>         CSV file that contains flows (column 'flow')\n"
              "  -x <factor>       factor determining lower bound for cost relative to travel time.\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

// Some helper aliases.
using VertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
using EdgeAttributes = EdgeAttrs<
        FreeFlowSpeedAttribute,
        MapToEdgeInPsgAttribute,
        MapToEdgeInReducedVehAttribute,
        OsmRoadCategoryAttribute,
        TravelTimeAttribute,
        TraversalCostAttribute,
        FloatingPointTraversalCostAttribute>;
using InputGraph = StaticGraph<VertexAttributes, EdgeAttributes>;


int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        const auto graphFileName = clp.getValue<std::string>("g");
        const auto flowFileName = clp.getValue<std::string>("f");
        const auto factor = clp.getValue<double>("x");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".gr.bin")) {
            outputFileName += ".gr.bin";
        }

        // Read the graph from file.
        std::cout << "Reading full vehicle network from file... " << std::flush;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        std::cout << "done." << std::endl;

        // Read the flow data from file.
        std::cout << "Reading flow data from file... " << std::flush;
        std::vector<int> flows;
        int flowInFile;
        io::CSVReader<1, io::trim_chars<' '>> flowsFileReader(flowFileName);
        flowsFileReader.read_header(io::ignore_no_column, "flow");
        while (flowsFileReader.read_row(flowInFile)) {
            if (flowInFile < 0)
                throw std::invalid_argument("invalid flow -- '" + std::to_string(flowInFile) + "'");
            flows.push_back(flowInFile);
        }
        if (flows.size() != inputGraph.numEdges())
            throw std::invalid_argument("Number of given traffic flows (" + std::to_string(flows.size())
                                        + ") is not equal to number of edges in vehicle input graph ("
                                        + std::to_string(inputGraph.numEdges()) + ")!");
        std::cout << "done." << std::endl;

        std::cout << "Compute new traversal costs..." << std::flush;
        FORALL_VALID_EDGES(inputGraph, u, e) {
                const int travelTime = inputGraph.get<TravelTimeAttribute>(e);
                const int flow = flows[e];
                const double floatTraversalCost =
                        flow == 0 ? static_cast<float>(travelTime) : factor * static_cast<float>(travelTime) +
                                                                      (1.0 - factor) * static_cast<float>(travelTime) /
                                                                      static_cast<float>(flow);
                const int traversalCost = flow == 0 ? travelTime : static_cast<int>(floatTraversalCost);
                LIGHT_KASSERT(factor != 1 || traversalCost == travelTime,
                              "factor is 1, but traversal cost is not equal to travel time");
                inputGraph.get<TraversalCostAttribute>(e) = traversalCost;
            }
        std::cout << "done." << std::endl;

        std::cout << "Write the network with new traversal costs to output file..." << std::flush;
        std::ofstream fullVehOut(outputFileName, std::ios::binary);
        if (!fullVehOut.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".gr.bin'");
        inputGraph.writeTo(fullVehOut);
        std::cout << " done." << std::endl;


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
