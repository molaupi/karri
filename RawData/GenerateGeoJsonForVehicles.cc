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
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateGeoJsonForVehicles -g <file> -v <file> -o <file>\n"
              "Outputs all request origin and destination edges in a given road network as GeoJson.\n"
              "  -g <file>              input road network in binary format.\n"
              "  -v <file>              vehicles in CSV format.\n"
              "  -csv-in-LOUD-format    if set, assumes that input files are in the format used by LOUD.\n"
              "  -o <file>              place GeoJSON in <file>\n"
              "  -help                  display this help and exit\n";
}


template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForEdge(const InputGraphT &inputGraph, const int e, const int vehId) {

    static char color[] = "blue";
    nlohmann::json feature;
    feature["type"] = "Feature";
    feature["properties"] = {{"stroke",       color},
//                             {"stroke-width", 3},
                             {"edge_id",      e},
                             {"vehicle_id",   vehId}};

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
generateGeoJsonObjectForRequestEdges(const InputGraphT &inputGraph, const std::vector<karri::Vehicle> &vehicles) {
    // Construct the needed GeoJSON object
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "FeatureCollection";

    for (const auto &veh: vehicles) {
        topGeoJson["features"].push_back(generateGeoJsonFeatureForEdge(inputGraph, veh.initialLocation, veh.vehicleId));
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
        auto vehicleFileName = clp.getValue<std::string>("v");
        if (!endsWith(vehicleFileName, ".csv"))
            vehicleFileName += ".csv";
        const bool csvFilesInLoudFormat = clp.isSet("csv-in-LOUD-format");
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

        // Read the vehicle data from file.
        std::cout << "Reading vehicle data from file... " << std::flush;
        std::vector<karri::Vehicle> vehicles;
        int location, capacity, startOfServiceTime, endOfServiceTime;
        io::CSVReader<4, io::trim_chars<' '>> vehiclesFileReader(vehicleFileName);

        if (csvFilesInLoudFormat) {
            vehiclesFileReader.read_header(io::ignore_no_column, "initial_location", "seating_capacity",
                                           "start_service_time", "end_service_time");
        } else {
            vehiclesFileReader.read_header(io::ignore_no_column,
                                           "initial_location", "start_of_service_time",
                                           "end_of_service_time", "capacity");
        }

        while ((csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, capacity, startOfServiceTime, endOfServiceTime)) ||
               (!csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, startOfServiceTime, endOfServiceTime, capacity))) {
            if (location < 0 || location >= origIdToSeqId.size() ||
                    origIdToSeqId[location] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(location) + "'");
            if (endOfServiceTime <= startOfServiceTime)
                throw std::invalid_argument("start of service time needs to be before end of service time");
            const int vehicleId = static_cast<int>(vehicles.size());
            vehicles.push_back({vehicleId, origIdToSeqId[location], startOfServiceTime * 10,
                             endOfServiceTime * 10, capacity});
        }
        std::cout << "done.\n";


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson = generateGeoJsonObjectForRequestEdges(inputGraph, vehicles);
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