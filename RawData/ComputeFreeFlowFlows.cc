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
#include <routingkit/nested_dissection.h>

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

inline void printUsage() {
    std::cout <<
              "Usage: ComputeFreeFlowFlows -g <file> -d <file> -o <file>\n\n"

              "Given a set of OD-pairs and a road network, computes the free-flow shortest paths for each pair and outputs the\n"
              "traffic flow for every edge e in the road network, i.e. the number of shortest paths using e.\n\n"
              "  -g <file>         input graph in binary format\n"
              "  -d <file>         CSV file that contains OD pairs (columns 'origin' and 'destination' containing vertex-ids)\n"
              "  -h <file>         weighted contraction hierarchy (optional)\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

// Some helper aliases.
using VertexAttributes = VertexAttrs<LatLngAttribute>;
using EdgeAttributes = EdgeAttrs<LengthAttribute, TravelTimeAttribute>;
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
        const auto chFileName = clp.getValue<std::string>("h");
        auto outputFileName = clp.getValue<std::string>("o");
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
        std::cout << "done." << std::endl;

        std::unique_ptr<CH> chPtr;
        if (chFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            chPtr = std::make_unique<CH>();
            chPtr->preprocess<TravelTimeAttribute>(inputGraph);
            std::cout << "done.\n";
        } else {
            // Read the CH from file.
            std::cout << "Reading CH from file... " << std::flush;
            std::ifstream chFile(chFileName, std::ios::binary);
            if (!chFile.good())
                throw std::invalid_argument("file not found -- '" + chFileName + "'");
            chPtr = std::make_unique<CH>(chFile);
            chFile.close();
            std::cout << "done.\n";
        }
        auto& ch = *chPtr;

        // Read the OD pairs from file.
        std::cout << "Reading OD pairs from file... " << std::flush;
        const auto odPairs = importODPairsFrom(demandFileName);
        std::cout << "done.\n";

        // Run queries
        std::cout << "Running queries... " << std::flush;
        ProgressBar progressBar(odPairs.size(), true);
        std::vector<int> flows(inputGraph.numEdges(), 0);
        CHQuery<LabelSet> chQuery(ch);
        CHPathUnpacker pathUnpacker(ch);
        std::vector<int> path;
        for (const auto& od : odPairs) {
            int src = ch.rank(od.origin);
            int dst = ch.rank(od.destination);
            chQuery.run(src, dst);
            path.clear();
            pathUnpacker.unpackUpDownPath(chQuery.getUpEdgePath(), chQuery.getDownEdgePath(), path);
            for (const auto edge : path) {
                flows[edge]++;
            }
            ++progressBar;
        }
        std::cout << " done.\n";

        // Write the results to file.
        std::cout << "Writing results to file... " << std::flush;
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("could not open file -- '" + outputFileName + "'");
        outputFile << "flow\n";
        for (const auto flow : flows) {
            outputFile << flow << '\n';
        }
        outputFile.close();
        std::cout << "done.\n";
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
