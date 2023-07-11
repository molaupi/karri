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
              "Usage: GenerateGeoJsonForEdges -g <file> -o <file>\n"
              "Outputs all vertices (or edges) in a given road network as GeoJson.\n"
              "  -g <file>         input road network in binary format.\n"
              "  -edges            If set, outputs edges instead of vertices.\n"
              "  -p <[0,1]>        only outputs <[0,1]> * 100 % of vertices or edges (dflt: 1)\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}

template<typename InputGraphT>
nlohmann::json generateGeoJsonObjectForEdges(const InputGraphT &inputGraph, const double ratio) {
    // Construct the needed path GeoJSON objects
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "GeometryCollection";
    static char color[] = "blue";

    int numEdgesOutput = 0;
    int numEdgesToOutput = inputGraph.numEdges() * ratio;
    FORALL_VALID_EDGES(inputGraph, v, e) {

            // Construct edge feature
            nlohmann::json edgeFeature;
            edgeFeature["type"] = "LineString";

            const auto tailLatLng = inputGraph.latLng(v);
            const auto tailCoordinate = nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()});
            edgeFeature["coordinates"].push_back(tailCoordinate);

            const auto headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
            const auto headCoordinate = nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()});
            edgeFeature["coordinates"].push_back(headCoordinate);

            edgeFeature["properties"] = {{"edge_id",      e},
                                         {"stroke",       color},
                                         {"stroke-width", 3}};

            topGeoJson["geometries"].push_back(edgeFeature);

            if (++numEdgesOutput >= numEdgesToOutput)
                return topGeoJson;
        }

    return topGeoJson;
}

template<typename InputGraphT>
nlohmann::json generateGeoJsonObjectForVertices(const InputGraphT &inputGraph, const double ratio) {
    // Construct the needed path GeoJSON objects
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "MultiPoint";

    int numVerticesOutput = 0;
    int numVerticesToOutput = inputGraph.numVertices() * ratio;
    FORALL_VERTICES(inputGraph, v) {
        const auto latLng = inputGraph.latLng(v);
        const auto coordinate = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
        topGeoJson["coordinates"].push_back(coordinate);

        if (++numVerticesOutput >= numVerticesToOutput)
            return topGeoJson;
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

        const bool outputEdges = clp.isSet("edges");
        const double ratio = clp.getValue<double>("p", 1.0);

        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute>>;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        InputGraph inputGraph(inputGraphFile);
        inputGraphFile.close();
        std::cout << "done.\n";


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson;
        if (outputEdges) {
            geoJson = generateGeoJsonObjectForEdges(inputGraph, ratio);
        } else {
            geoJson = generateGeoJsonObjectForVertices(inputGraph, ratio);
        }
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