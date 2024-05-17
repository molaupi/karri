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
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateGeoJsonForRequests -g <file> -r <file> -o <file>\n"
              "Outputs all request origin and destination edges in a given road network as GeoJson.\n"
              "  -g <file>         input road network in binary format.\n"
              "  -p <file>         input pairs file in CSV format.\n"
              "  -o-col-name <str> name of origin column in pairs file (default: 'origin').\n"
              "  -d-col-name <str> name of destination column in pairs file (default: 'destination').\n"
              "  -edges            if set, origin and destination values are assumed to be edge IDs instead of vertex IDs.\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}


template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForEdge(const InputGraphT &inputGraph, const int e) {

    static char color[] = "blue";
    nlohmann::json feature;
    feature["type"] = "LineString";

    const auto tailLatLng = inputGraph.latLng(inputGraph.edgeTail(e));
    const auto tailCoordinate = nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()});
    feature["coordinates"].push_back(tailCoordinate);

    const auto headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
    const auto headCoordinate = nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()});
    feature["coordinates"].push_back(headCoordinate);

    feature["properties"] = {{"stroke",       color},
                             {"stroke-width", 3},
                             {"edge_id",      e}};

    return feature;
}

template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForVertex(const InputGraphT &inputGraph, const int v) {
    static char color[] = "red";
    nlohmann::json feature;
    feature["type"] = "Point";

    const auto latLng = inputGraph.latLng(v);
    const auto coord = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
    feature["coordinates"].push_back(coord);

    feature["properties"] = {{"stroke",       color},
                             {"stroke-width", 3},
                             {"vertex_id",    v}};

    return feature;
}

template<typename InputGraphT, bool idsAreEdges>
nlohmann::json
generateGeoJsonObjectForPairs(const InputGraphT &inputGraph, const std::vector<std::pair<int, int>> &pairs) {
    // Construct the needed GeoJSON object
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "GeometryCollection";

    for (const auto &[o, d]: pairs) {
        if (idsAreEdges) {
            topGeoJson["geometries"].push_back(generateGeoJsonFeatureForEdge(inputGraph, o));
            topGeoJson["geometries"].push_back(generateGeoJsonFeatureForEdge(inputGraph, d));
        } else {
            topGeoJson["geometries"].push_back(generateGeoJsonFeatureForVertex(inputGraph, o));
            topGeoJson["geometries"].push_back(generateGeoJsonFeatureForVertex(inputGraph, d));
        }
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
        auto pairsFileName = clp.getValue<std::string>("p");
        if (!endsWith(pairsFileName, ".csv"))
            pairsFileName += ".csv";
        const std::string oColName = clp.getValue<std::string>("o-col-name", "origin");
        const std::string dColName = clp.getValue<std::string>("d-col-name", "destination");
        const bool idsAreEdges = clp.isSet("edges");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".geojson"))
            outputFileName += ".geojson";


        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, OsmRoadCategoryAttribute>>;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        InputGraph inputGraph(inputGraphFile);
        inputGraphFile.close();
        std::vector<int32_t> origIdToSeqId;
        if (inputGraph.numEdges() > 0 && inputGraph.edgeId(0) == INVALID_ID) {
            origIdToSeqId.assign(inputGraph.numEdges(), INVALID_ID);
            std::iota(origIdToSeqId.begin(), origIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(inputGraph, v, e) {
                    assert(inputGraph.edgeId(e) == INVALID_ID);
                    inputGraph.edgeId(e) = e;
                    inputGraph.edgeTail(e) = v;
                }
        } else {
            FORALL_VALID_EDGES(inputGraph, v, e) {
                    assert(inputGraph.edgeId(e) != INVALID_ID);
                    if (inputGraph.edgeId(e) >= origIdToSeqId.size()) {
                        const auto numElementsToBeInserted = inputGraph.edgeId(e) + 1 - origIdToSeqId.size();
                        origIdToSeqId.insert(origIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                    }
                    assert(origIdToSeqId[inputGraph.edgeId(e)] == INVALID_ID);
                    origIdToSeqId[inputGraph.edgeId(e)] = e;
                    inputGraph.edgeId(e) = e;
                    inputGraph.edgeTail(e) = v;
                }
        }
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<std::pair<int, int>> odPairs;
        int origin, destination;
        io::CSVReader<2, io::trim_chars<' '>> pairsFileReader(pairsFileName);
        pairsFileReader.read_header(io::ignore_extra_column, oColName, dColName);
        while (pairsFileReader.read_row(origin, destination)) {
            if (idsAreEdges) {
                if (origin < 0 || origin >= origIdToSeqId.size() || origIdToSeqId[origin] == INVALID_ID)
                    throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
                if (destination < 0 || destination >= origIdToSeqId.size() ||
                    origIdToSeqId[destination] == INVALID_ID)
                    throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
                origin = origIdToSeqId[origin];
                destination = origIdToSeqId[destination];
            } else {
                if (origin < 0 || origin >= inputGraph.numVertices())
                    throw std::invalid_argument("invalid vertex -- '" + std::to_string(origin) + "'");
                if (destination < 0 || destination >= inputGraph.numVertices())
                    throw std::invalid_argument("invalid vertex -- '" + std::to_string(destination) + "'");
            }
            odPairs.emplace_back(origin, destination);
        }
        std::cout << "done.\n";


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson;
        if (idsAreEdges) {
            geoJson = generateGeoJsonObjectForPairs<InputGraph, true>(inputGraph, odPairs);
        } else {
            geoJson = generateGeoJsonObjectForPairs<InputGraph, false>(inputGraph, odPairs);
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