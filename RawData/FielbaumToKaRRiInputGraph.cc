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


#include <iostream>
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Geometry/LatLng.h"
#include "DataStructures/Containers/BitVector.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: FielbaumToKaRRiInputGraph -i <path> -o <file>\n"
              "Convert Fielbaum input data into KaRRi vehicle and passenger networks in binary format.\n"
              "  -i <path>          path to csv files returned from FielbaumMatToCsv.py (expects <path>.nodes.csv and <path>.edges.csv).\n"
              "  -o <path>          place graph binaries in <path>_veh.gr.bin and <path>_psg.gr.bin\n"
              "  -help              display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        auto inPath = clp.getValue<std::string>("i");
        auto nodesFileName = inPath + ".nodes.csv";
        auto edgesFileName = inPath + ".edges.csv";

        auto outputFilePath = clp.getValue<std::string>("o");

        using VehVertexAttr = VertexAttrs<LatLngAttribute>;
        using VehEdgeAttr = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, TravelTimeAttribute, CarEdgeToPsgEdgeAttribute>;
        using VehGraph = StaticGraph<VehVertexAttr, VehEdgeAttr>;
        using PsgVertexAttr = VertexAttrs<LatLngAttribute>;
        using PsgEdgeAttr = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, TravelTimeAttribute, PsgEdgeToCarEdgeAttribute>;
        using PsgGraph = StaticGraph<PsgVertexAttr, PsgEdgeAttr>;

        std::cout << "Reading latitude/longitude of nodes from file... " << std::flush;
        double lat, lng;
        io::CSVReader<2, io::trim_chars<' '>> nodesLatLngReader(nodesFileName);
        nodesLatLngReader.read_header(io::ignore_no_column, "latitude", "longitude");

        AlignedVector<LatLng> nodeLatLngs;
        while (nodesLatLngReader.read_row(lat, lng)) {
            nodeLatLngs.emplace_back(lat, lng);
        }
        std::cout << "done.\n";
        const int numVertices = nodeLatLngs.size();

        std::cout << "Reading edges from file... " << std::flush;
        int tail, head;
        double vehTtInMin;
        io::CSVReader<3, io::trim_chars<' '>> edgesReader(edgesFileName);
        edgesReader.read_header(io::ignore_no_column, "tail", "head", "travel_time");

        AlignedVector<VehGraph::OutEdgeRange> outEdgeRanges(numVertices + 1);
        AlignedVector<int32_t> edgeHeads;
        AlignedVector<int> edgeIds;
        AlignedVector<int> edgeTails;
        AlignedVector<int> vehTravelTimes;
        AlignedVector<int> pedTravelTimes;
        int prevVertex = 0;
        outEdgeRanges[0] = {0};
        static constexpr double WALK_SPEED_KPH = 4.5;
        while (edgesReader.read_row(tail, head, vehTtInMin)) {
            for (int i = prevVertex + 1; i <= tail; ++i)
                outEdgeRanges[i] = {static_cast<int>(edgeHeads.size())};
            prevVertex = tail;
            const int edgeId = edgeHeads.size();
            const int vehTt = static_cast<int>(std::round(
                    vehTtInMin * 60 * 10)); // KaRRi expects travel times in tenth of seconds
            const int pedTt = static_cast<int>(std::round(
                    36.0 * nodeLatLngs[tail].getGreatCircleDistanceTo(nodeLatLngs[head]) / WALK_SPEED_KPH));
            edgeHeads.push_back(head);
            edgeTails.push_back(tail);
            vehTravelTimes.push_back(vehTt);
            pedTravelTimes.push_back(pedTt);
            edgeIds.push_back(edgeId);
        }
        for (int i = prevVertex + 1; i <= numVertices; ++i) {
            outEdgeRanges[i] = {static_cast<int>(edgeHeads.size())};
        }
        std::cout << "done.\n";

        const int numEdges = edgeHeads.size();

        std::cout << "Graph has " << numVertices << " vertices and " << numEdges << " edges." << std::endl;

        std::cout << "Constructing graphs... " << std::flush;
        AlignedVector<int> carEdgeToPedEdge(numEdges);
        std::iota(carEdgeToPedEdge.begin(), carEdgeToPedEdge.end(), 0);
        AlignedVector<int> pedEdgeToCarEdge(numEdges);
        std::iota(pedEdgeToCarEdge.begin(), pedEdgeToCarEdge.end(), 0);

        AlignedVector<PsgGraph::OutEdgeRange> pedOutEdgeRanges(numVertices + 1);
        for (int i = 0; i <= numVertices; ++i)
            pedOutEdgeRanges[i] = {outEdgeRanges[i].first()};
        auto pedEdgeHeads = edgeHeads;
        auto pedNodeLatLngs = nodeLatLngs;
        auto pedEdgeIds = edgeIds;
        auto pedEdgeTails = edgeTails;


        VehGraph vehGraph(std::move(outEdgeRanges), std::move(edgeHeads), numEdges,
                          std::move(nodeLatLngs),
                          std::move(edgeIds), std::move(edgeTails), std::move(vehTravelTimes),
                          std::move(carEdgeToPedEdge));
        PsgGraph psgGraph(std::move(pedOutEdgeRanges), std::move(pedEdgeHeads), numEdges,
                          std::move(pedNodeLatLngs),
                          std::move(pedEdgeIds), std::move(pedEdgeTails), std::move(pedTravelTimes),
                          std::move(pedEdgeToCarEdge));
        std::cout << "done.\n";

        std::cout << "Writing graphs to output files... " << std::flush;

        auto vehOutputFileName = outputFilePath + "_veh.gr.bin";
        auto psgOutputFileName = outputFilePath + "_psg.gr.bin";

        std::ofstream vehOutputFile(vehOutputFileName);
        if (!vehOutputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + vehOutputFileName + "'");
        vehGraph.writeTo(vehOutputFile);

        std::ofstream psgOutputFile(psgOutputFileName);
        if (!psgOutputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + psgOutputFileName + "'");
        psgGraph.writeTo(psgOutputFile);

        std::cout << " done." << std::endl;

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}