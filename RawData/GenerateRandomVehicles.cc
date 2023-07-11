/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <random>
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Geometry/Area.h"

inline void printUsage() {
    std::cout <<
              "Usage: GenerateRandomVehicles -n <num> -g <file> [-a <file>] -o <file>\n"
              "Generates vehicles with an initial location chosen uniformly at random. Optionally restricts initial\n"
              "locations to given polygonal area.\n"
              "  -n <num>          number of vehicles.\n"
              "  -c <num>          seating capacity of each vehicle (dflt: 4)"
              "  -start <time>     start of the vehicles' service time (dflt: 0)"
              "  -end <time>       end of the vehicles' service time (dflt: 108000)"
              "  -s <seed>         start random number generator with <seed> (dflt: 0)\n"
              "  -g <file>         input graph in binary format\n"
              "  -a <file>         restrict initial locations to polygonal study area\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto numVehicles = clp.getValue<int>("n");
        const auto capacity = clp.getValue<int>("c", 4);
        const auto startOfServiceTime = clp.getValue<int>("start", 0);
        const auto endOfServiceTime = clp.getValue<int>("end", 108000);
        const auto seed = clp.getValue<int>("s", 0);
        const auto graphFileName = clp.getValue<std::string>("g");
        const auto areaFileName = clp.getValue<std::string>("a");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<>>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        std::cout << "done.\n";

        // Read the study area from OSM POLY file.
        std::vector<int> eligibleEdges;
        eligibleEdges.reserve(inputGraph.numEdges());
        if (!areaFileName.empty()) {
            std::cout << "Reading study area from OSM POLY file..." << std::flush;
            Area studyArea;
            studyArea.importFromOsmPolyFile(areaFileName);
            const auto box = studyArea.boundingBox();
            FORALL_EDGES(inputGraph, e) {
                const auto &headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
                const Point headP(headLatLng.longitude(), headLatLng.latitude());
                if (box.contains(headP) && studyArea.contains(headP))
                    eligibleEdges.push_back(e);
            }
            std::cout << " done.\n";
        }
        eligibleEdges.shrink_to_fit();

        // Initialize output
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        out << "initial_location,start_of_service_time,end_of_service_time,capacity\n";

        std::cout << "Generating vehicles at random locations...";
        std::minstd_rand rand(seed + 42);
        std::uniform_int_distribution<> dist(0, eligibleEdges.size() - 1);
        for (int i = 0; i < numVehicles; ++i) {
            const auto locIdx = dist(rand);
            out << eligibleEdges[locIdx] << "," << startOfServiceTime << "," << endOfServiceTime << "," << capacity << "\n";
        }
        std::cout << "done.\n";

        out.close();


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] <<" -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}