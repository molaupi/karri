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
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/BiDijkstra.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHQuery.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateGeoJsonForSearchSpace -a <algo> -s <vertex> -t <vertex> -g <file> [-h <file>] -o <file>\n"
              "Runs a point-to-point shortest-path query and outputs all vertices or edges that were relaxed as GeoJson.\n"
              "  -a <algo>         algorithm to be used: Dij, Bi-Dij, CH\n"
              "  -s <vertex>       source vertex ID\n"
              "  -t <vertex>       target vertex ID\n"
              "  -g <file>         input road network in binary format.\n"
              "  -h <file>         (optional) CH hierarchy file in binary format (only useful if -a CH is used)\n"
              "  -edges            If set, outputs edges instead of vertices.\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}

template<typename InputGraphT>
nlohmann::json generateMarkerFeatureForVertex(const InputGraphT &inputGraph, const int v,
                                              const std::string &color, const std::string &name) {

    nlohmann::json feature;
    feature["type"] = "Feature";

    feature["properties"] = {
            {"label-text", name},
            {"fill", color}
    };

    nlohmann::json geometry;
    geometry["type"] = "Point";
    const auto latLng = inputGraph.latLng(v);
    const auto coordinate = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
    geometry["coordinates"] = coordinate;

    feature["geometry"] = geometry;

    return feature;
}

template<typename InputGraphT, typename HeadTailPairsT>
void
generateGeoJsonObjectForEdges(nlohmann::json &topGeoJson, InputGraphT &inputGraph, const HeadTailPairsT &headTailPairs,
                              const std::string &color) {
    // Construct the needed path GeoJSON objects
    for (const auto &[head, tail]: headTailPairs) {

        // Construct edge feature
        nlohmann::json feature;
        feature["type"] = "Feature";
        feature["properties"] = {
                //                                         {"stroke-width", 3},
                {"stroke", color}
        };

        nlohmann::json edgeGeometry;
        edgeGeometry["type"] = "LineString";

        const auto tailLatLng = inputGraph.latLng(tail);
        const auto tailCoordinate = nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()});
        edgeGeometry["coordinates"].push_back(tailCoordinate);

        const auto headLatLng = inputGraph.latLng(head);
        const auto headCoordinate = nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()});
        edgeGeometry["coordinates"].push_back(headCoordinate);

        feature["geometry"] = edgeGeometry;
        topGeoJson["features"].push_back(feature);
    }
}

template<typename InputGraphT, typename VerticesT>
void generateGeoJsonObjectForVertices(nlohmann::json &topGeoJson, InputGraphT &inputGraph, const VerticesT &vertices,
                                      const std::string &color) {

    // Construct the needed path GeoJSON objects
    for (const auto &v: vertices) {

        nlohmann::json feature;
        feature["type"] = "Feature";
        feature["properties"] = {{"vertex_id", v},
                                 {"stroke",    color}};

        nlohmann::json vertexGeometry;
        vertexGeometry["type"] = "Point";

        const auto latLng = inputGraph.latLng(v);
        const auto coordinate = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
        vertexGeometry["coordinates"] = coordinate;

        feature["geometry"] = vertexGeometry;
        topGeoJson["features"].push_back(feature);
    }
}

template<typename InputGraphT, typename DijSearchT>
void reconstructReachedVertices(const InputGraphT &inputGraph, DijSearchT &search, std::vector<int> &verticesReached) {
    FORALL_VERTICES(inputGraph, v) {
        if (search.getDistance(v) < INFTY) {
            verticesReached.push_back(v);
        }
    }
}

template<typename InputGraphT, typename DijSearchT>
void reconstructRelaxedEdges(const InputGraphT &inputGraph, DijSearchT &search,
                             std::vector<std::pair<int, int>> &edgesRelaxed) {
    FORALL_VERTICES(inputGraph, v) {
        if (search.getDistance(v) >= INFTY)
            continue;
        const int parent = search.getParentVertex(v);
        KASSERT(parent != INVALID_ID);
        if (v == parent)
            continue;
        edgesRelaxed.emplace_back(parent, v);
    }
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
        const auto sourceVertex = clp.getValue<int>("s");
        const auto targetVertex = clp.getValue<int>("t");
        const auto algorithm = clp.getValue<std::string>("a");

        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<TravelTimeAttribute, EdgeIdAttribute, EdgeTailAttribute>>;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        InputGraph inputGraph(inputGraphFile);
        inputGraphFile.close();
        std::cout << "done.\n";

        std::vector<int> forwVerticesReached;
        std::vector<std::pair<int, int>> forwEdgesRelaxed;
        std::vector<int> revVerticesReached;
        std::vector<std::pair<int, int>> revEdgesRelaxed;

        // Run the specified shortest-path algorithm and collect the search spaces.
        if (algorithm == "Dij") {
            Dijkstra<InputGraph, TravelTimeAttribute, BasicLabelSet<0, ParentInfo::PARENT_VERTICES_ONLY>> search(
                    inputGraph);
            search.run(sourceVertex, targetVertex);
            if (outputEdges) {
                reconstructRelaxedEdges(inputGraph, search, forwEdgesRelaxed);
            } else {
                reconstructReachedVertices(inputGraph, search, forwVerticesReached);
            }
        } else if (algorithm == "Bi-Dij") {
            const auto &revGraph = inputGraph.getReverseGraph();
            BiDijkstra<Dijkstra<InputGraph, TravelTimeAttribute, BasicLabelSet<0, ParentInfo::PARENT_VERTICES_ONLY>>> search(
                    inputGraph, revGraph);
            search.run(sourceVertex, targetVertex);
            auto &forwSearch = search.getForwardSearch();
            auto &revSearch = search.getReverseSearch();
            if (outputEdges) {
                reconstructRelaxedEdges(inputGraph, forwSearch, forwEdgesRelaxed);
                reconstructRelaxedEdges(inputGraph, revSearch, revEdgesRelaxed);
            } else {
                reconstructReachedVertices(inputGraph, forwSearch, forwVerticesReached);
                reconstructReachedVertices(inputGraph, revSearch, revVerticesReached);
            }
        } else if (algorithm == "CH") {
            auto hierarchyFileName = clp.getValue<std::string>("h");
            if (!endsWith(hierarchyFileName, ".ch.bin"))
                hierarchyFileName += ".ch.bin";

            CH ch;
            if (hierarchyFileName.empty()) {
                std::cout << "Building CH... " << std::flush;
                ch.preprocess<TravelTimeAttribute>(inputGraph);
                std::cout << "done.\n";
            } else {
                // Read the CH from file.
                std::cout << "Reading CH from file... " << std::flush;
                std::ifstream hierarchyFile(hierarchyFileName, std::ios::binary);
                if (!hierarchyFile.good())
                    throw std::invalid_argument("file not found -- '" + hierarchyFileName + "'");
                ch = CH(hierarchyFile);
                hierarchyFile.close();
                std::cout << "done.\n";
            }

            CHQuery<BasicLabelSet<0, ParentInfo::PARENT_VERTICES_ONLY>> chQuery(ch);
            chQuery.run(ch.rank(sourceVertex), ch.rank(targetVertex));
            auto &forwSearch = chQuery.getInternalSearch().getForwardSearch();
            auto &revSearch = chQuery.getInternalSearch().getReverseSearch();
            if (outputEdges) {
                reconstructRelaxedEdges(inputGraph, forwSearch, forwEdgesRelaxed);
                reconstructRelaxedEdges(inputGraph, revSearch, revEdgesRelaxed);
            } else {
                reconstructReachedVertices(inputGraph, forwSearch, forwVerticesReached);
                reconstructReachedVertices(inputGraph, revSearch, revVerticesReached);
            }
            for (auto &v: forwVerticesReached) {
                v = ch.contractionOrder(v);
            }
            for (auto &v: revVerticesReached) {
                v = ch.contractionOrder(v);
            }
            for (auto &[head, tail]: forwEdgesRelaxed) {
                head = ch.contractionOrder(head);
                tail = ch.contractionOrder(tail);
            }
            for (auto &[head, tail]: revEdgesRelaxed) {
                head = ch.contractionOrder(head);
                tail = ch.contractionOrder(tail);
            }
        } else {
            throw std::invalid_argument("unrecognized algorithm -- '" + algorithm + "'");
        }


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson;
        geoJson["type"] = "FeatureCollection";
        if (outputEdges) {
            generateGeoJsonObjectForEdges(geoJson, inputGraph, forwEdgesRelaxed, "blue");
            generateGeoJsonObjectForEdges(geoJson, inputGraph, revEdgesRelaxed, "red");
        } else {
            generateGeoJsonObjectForVertices(geoJson, inputGraph, forwVerticesReached, "blue");
            generateGeoJsonObjectForVertices(geoJson, inputGraph, revVerticesReached, "red");
        }

        geoJson["features"].push_back(generateMarkerFeatureForVertex(inputGraph, sourceVertex, "blue", "s"));
        geoJson["features"].push_back(generateMarkerFeatureForVertex(inputGraph, targetVertex, "red", "t"));

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