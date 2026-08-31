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
#include "DataStructures/Utilities/OriginDestination.h"
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
              "  -r <file>         input requests in CSV format.\n"
              "  -input-vertices   if set, treats IDs in input as vertex IDs instead of edge IDs.\n"
              "  -output-vertices  if set, outputs GeoJson for small squares around head vertices of origin/destination\n"
              "  -vertex-width <length>     side-length of square around vertex in meters, default: 50m\n"
              "  -vertex-opacity <value>     opacity of vertex squares, default: 0.1\n"
              "  -csv-in-LOUD-format    if set, assumes that input files are in the format used by LOUD.\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}


double computeLatitudeOffset(const int sideLengthInM) {
    KASSERT(sideLengthInM >= 0);
    const double offsetInM = static_cast<double>(sideLengthInM) / 2;
    return toDegrees(offsetInM / EARTH_RADIUS);
}

double computeLongitudeOffset(const LatLng &latLng, const int sideLengthInM) {
    KASSERT(sideLengthInM >= 0);
    const double offsetInM = static_cast<double>(sideLengthInM) / 2;
    return toDegrees(offsetInM / (EARTH_RADIUS * cos(toRadians(latLng.latInDeg()))));
}

std::array<LatLng, 4> computeSquareAroundLatLng(const LatLng &latLng, const int sideLengthInM) {
    const auto latOffset = computeLatitudeOffset(sideLengthInM);
    const auto lngOffset = computeLongitudeOffset(latLng, sideLengthInM);

    // 4 coordinates of square in counter-clockwise order, starting at northwest corner
    std::array<LatLng, 4> result;
    result[0] = LatLng(latLng.latInDeg() + latOffset, latLng.lngInDeg() - lngOffset);
    result[1] = LatLng(latLng.latInDeg() - latOffset, latLng.lngInDeg() - lngOffset);
    result[2] = LatLng(latLng.latInDeg() - latOffset, latLng.lngInDeg() + lngOffset);
    result[3] = LatLng(latLng.latInDeg() + latOffset, latLng.lngInDeg() + lngOffset);
    return result;
}


template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForVertex(const InputGraphT &inputGraph, const int v, const int reqId,
                                               const std::string &type, const int sideLengthInM,
                                               const double opacity) {

//    static char color[] = "blue";

    assert(type == "origin" || type == "destination");
    const std::string color = type == "origin" ? "blue" : "red";

    nlohmann::json feature;
    feature["type"] = "Feature";

    // Add properties
    feature["properties"] = {{"fill",       color},
                             {"stroke-width", 0},
                             {"opacity",    opacity},
                             {"vertex_id",  v},
                             {"request_id", reqId},
                             {"type",       type}};

    // Make polygon geometry member for vertex
    nlohmann::json geometry;
    geometry["type"] = "Polygon";

    const auto latLng = inputGraph.latLng(v);
    const auto squareLatLngs = computeSquareAroundLatLng(latLng, sideLengthInM);
    nlohmann::json polygonCoordinates;
    for (const auto &squareLatLng: squareLatLngs) {
        const auto coordinate = nlohmann::json::array({squareLatLng.lngInDeg(), squareLatLng.latInDeg()});
        polygonCoordinates.push_back(coordinate);
    }
    // Close the polygon by adding the first coordinate again
    polygonCoordinates.push_back(polygonCoordinates[0]);
    geometry["coordinates"].push_back(polygonCoordinates);

    feature["geometry"] = geometry;

    return feature;
}


template<typename InputGraphT>
nlohmann::json
generateGeoJsonObjectForVertices(const InputGraphT &inputGraph, const std::vector<OriginDestination> &odPairs,
                                        const int sideLengthInM, const double opacity) {
    // Construct the needed GeoJSON object
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "FeatureCollection";

    for (int i = 0; i < odPairs.size(); ++i) {
        const auto &od = odPairs[i];
        topGeoJson["features"].push_back(
                generateGeoJsonFeatureForVertex(inputGraph, od.origin, i, "origin", sideLengthInM, opacity));
        topGeoJson["features"].push_back(
                generateGeoJsonFeatureForVertex(inputGraph, od.destination, i, "destination", sideLengthInM, opacity));

    }

    return topGeoJson;
}

template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForEdge(const InputGraphT &inputGraph, const int e, const int reqId,
                                             const std::string &type) {

//    static char color[] = "blue";

    assert(type == "origin" || type == "destination");
    const std::string color = type == "origin" ? "blue" : "red";

    nlohmann::json feature;
    feature["type"] = "Feature";

    // Add properties
    feature["properties"] = {{"stroke",     color},
//                             {"stroke-width", 1},
                             {"edge_id",    e},
                             {"request_id", reqId},
                             {"type",       type}};

    // Make LineString geometry member for edge
    nlohmann::json geometry;
    geometry["type"] = "LineString";
    const auto tailLatLng = inputGraph.latLng(inputGraph.edgeTail(e));
    const auto tailCoordinate = nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()});
    geometry["coordinates"].push_back(tailCoordinate);

    const auto headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
    const auto headCoordinate = nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()});
    geometry["coordinates"].push_back(headCoordinate);
    feature["geometry"] = geometry;

    return feature;
}

template<typename InputGraphT>
nlohmann::json
generateGeoJsonObjectForEdges(const InputGraphT &inputGraph, const std::vector<OriginDestination> &odPairs) {
    // Construct the needed GeoJSON object
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "FeatureCollection";

    for (int i = 0; i < odPairs.size(); ++i) {
        const auto &od = odPairs[i];
        topGeoJson["features"].push_back(
                generateGeoJsonFeatureForEdge(inputGraph, od.origin, i, "origin"));
        topGeoJson["features"].push_back(
                generateGeoJsonFeatureForEdge(inputGraph, od.destination, i, "destination"));

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
        auto requestFileName = clp.getValue<std::string>("r");
        if (!endsWith(requestFileName, ".csv"))
            requestFileName += ".csv";
        const bool csvFilesInLoudFormat = clp.isSet("csv-in-LOUD-format");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".geojson"))
            outputFileName += ".geojson";

        const bool inputVertices = clp.isSet("input-vertices");
        const bool outputVertices = clp.isSet("output-vertices") || inputVertices;


        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeTailAttribute, OsmRoadCategoryAttribute>>;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        InputGraph inputGraph(inputGraphFile);
        inputGraphFile.close();
        FORALL_VALID_EDGES(inputGraph, v, e) {
            inputGraph.edgeTail(e) = v;
        }
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<OriginDestination> odPairs;
        int origin, destination, requestTime;
        io::CSVReader<3, io::trim_chars<' '>> reqFileReader(requestFileName);

        if (csvFilesInLoudFormat) {
            reqFileReader.read_header(io::ignore_missing_column | io::ignore_extra_column, "pickup_spot", "dropoff_spot", "min_dep_time");
        } else {
            reqFileReader.read_header(io::ignore_missing_column | io::ignore_extra_column, "origin", "destination", "req_time");
        }

        while (reqFileReader.read_row(origin, destination, requestTime)) {
            if (!inputVertices && outputVertices) {
                origin = inputGraph.edgeHead(origin);
                destination = inputGraph.edgeHead(destination);
            }
            odPairs.emplace_back(origin, destination);
        }
        std::cout << "done.\n";


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson;
        if (!outputVertices) {
            geoJson = generateGeoJsonObjectForEdges(inputGraph, odPairs);
        } else {
            const int sideLengthInM = clp.getValue<int>("vertex-width", 50);
            const double opacity = clp.getValue<double>("vertex-opacity", 0.1);
            geoJson = generateGeoJsonObjectForVertices(inputGraph, odPairs, sideLengthInM, opacity);
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