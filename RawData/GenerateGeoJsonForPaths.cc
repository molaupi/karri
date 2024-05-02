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
#include "Tools/custom_assertion_levels.h"
#include "kassert/kassert.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateGeoJsonForPaths -g <file> -r <file> -o <file>\n"
              "Outputs all paths in a given file as GeoJson given the underlying network containing geographic coordinates.\n"
              "  -g <file>              input road network in binary format.\n"
              "  -p <file>              input paths in CSV format.\n"
              "  -path-col-name <name>  name of column in paths CSV that contains paths. Paths are expected to be in format 'e_1 : e_2 : ... : e_n'.\n"
              "  -o <file>              place GeoJSON in <file>\n"
              "  -help                  display this help and exit\n";
}


template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForPath(const InputGraphT &inputGraph, const std::vector<int>& edges) {

    static char color[] = "blue";
    nlohmann::json feature;
    feature["type"] = "LineString";

    for (int i = 0; i < edges.size(); ++i) {
        const auto e = edges[i];
        const auto tail = inputGraph.edgeTail(e);
        const auto latLng = inputGraph.latLng(tail);
        const auto coord = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
        feature["coordinates"].push_back(coord);
    }
    if (!edges.empty()) {
        const auto latLng = inputGraph.latLng(inputGraph.edgeHead(edges.back()));
        const auto coord = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
        feature["coordinates"].push_back(coord);
    }

    feature["properties"] = {{"stroke",       color},
                             {"stroke-width", 3}};

    return feature;
}

template<typename InputGraphT>
nlohmann::json
generateGeoJsonObjectForPaths(const InputGraphT &inputGraph, const std::vector<std::vector<int>> &paths) {
    // Construct the needed GeoJSON object
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "GeometryCollection";

    for (const auto &path: paths) {
        topGeoJson["geometries"].push_back(generateGeoJsonFeatureForPath(inputGraph, path));
    }

    return topGeoJson;
}

std::vector<int> parseEdgePathString(std::string s) {
    // Paths are expected to be sequence of edge ids separated by " : "
    s.erase(remove(s.begin(), s.end(), ' '), s.end());
    std::vector<int> result;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, ':')) {
        result.push_back(std::stoi(item));
    }

    return result;
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
        auto pathFileName = clp.getValue<std::string>("p");
        if (!endsWith(pathFileName, ".csv"))
            pathFileName += ".csv";
        const std::string pathColName = clp.getValue<std::string>("path-col-name");
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
        std::cout << "Reading path data from file... " << std::flush;
        std::vector<std::vector<int>> paths;
        std::string pathStr;
        io::CSVReader<1, io::trim_chars<' '>> pathFileReader(pathFileName);
        pathFileReader.read_header(io::ignore_extra_column, pathColName);


        while (pathFileReader.read_row(pathStr)) {
            if (pathStr.empty())
                continue;
            std::vector<int> edges = parseEdgePathString(pathStr);
            LIGHT_KASSERT(std::all_of(edges.begin(), edges.end(),
                                      [&](const int e) { return e < inputGraph.numEdges(); }));
            if (edges.empty())
                continue;
            paths.push_back(std::move(edges));
        }
        std::cout << "done.\n";


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson = generateGeoJsonObjectForPaths(inputGraph, paths);
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