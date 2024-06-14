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
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateGeoJsonForFlows -g <file> -f <file> -o <file>\n"
              "Outputs all vertices (or edges) in a given road network as GeoJson.\n"
              "  -g <file>         input road network in binary format.\n"
              "  -f <file>         input flow data in CSV format.\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}



template<typename InputGraphT>
nlohmann::json generateGeoJsonObjectForEdge(const InputGraphT &inputGraph, const int tail, const int e) {
    static char color[] = "blue";
    // Construct edge feature
    nlohmann::json edgeFeature;
    edgeFeature["type"] = "LineString";

    const auto tailLatLng = inputGraph.latLng(tail);
    const auto tailCoordinate = nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()});
    edgeFeature["coordinates"].push_back(tailCoordinate);

    const auto headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
    const auto headCoordinate = nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()});
    edgeFeature["coordinates"].push_back(headCoordinate);

    edgeFeature["properties"] = {{"edge_id",      e},
                                 {"stroke",       color},
                                 {"stroke-width", 3}};
    return edgeFeature;
}


template<typename InputGraphT>
nlohmann::json generateGeoJsonObjectForNonZeroFlowEdges(const InputGraphT &inputGraph, const std::vector<int> &flows) {
    // Construct the needed path GeoJSON objects
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "GeometryCollection";

    FORALL_VALID_EDGES(inputGraph, v, e) {
            if (flows[e] == 0)
                continue;
            topGeoJson["geometries"].push_back(generateGeoJsonObjectForEdge(inputGraph, v, e));
        }

    return topGeoJson;
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        auto inputGraphFileName = clp.getValue<std::string>("g");
        if (!endsWith(inputGraphFileName, ".gr.bin"))
            inputGraphFileName += ".gr.bin";
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".geojson"))
            outputFileName += ".geojson";

        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute>>;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        InputGraph inputGraph(inputGraphFile);
        inputGraphFile.close();
        std::cout << "done.\n";

        // Read the flow data from file.
        std::cout << "Reading flow data from file... " << std::flush;
        auto flowDataFileName = clp.getValue<std::string>("f");
        std::vector<int> trafficFlows;
        int flow;
        io::CSVReader<1, io::trim_chars<' '>> flowsFileReader(flowDataFileName);
        flowsFileReader.read_header(io::ignore_no_column, "flow");
        while (flowsFileReader.read_row(flow)) {
            if (flow < 0)
                throw std::invalid_argument("invalid flow -- '" + std::to_string(flow) + "'");
            trafficFlows.push_back(flow);
        }
        if (trafficFlows.size() != inputGraph.numEdges())
            throw std::invalid_argument("Number of given traffic flows (" + std::to_string(trafficFlows.size())
                                        + ") is not equal to number of edges in vehicle input graph ("
                                        + std::to_string(inputGraph.numEdges()) + ")!");
        std::cout << "done.\n";


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson = generateGeoJsonObjectForNonZeroFlowEdges(inputGraph, trafficFlows);
        std::cout << " done." << std::endl;

        std::cout << "Writing GeoJSON to output file... " << std::flush;
        // Open the output file and write the GeoJson.
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        outputFile << std::setw(2) << geoJson << std::endl;

        std::cout << " done." << std::endl;
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}