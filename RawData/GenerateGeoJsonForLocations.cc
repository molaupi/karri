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
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include <nlohmann/json.hpp>
#include <fstream>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateGeoJsonForLocations -g <file> -l <file> -o <file>\n"
              "Outputs a GeoJson depicting a set of locations in a road network. Each location is\n"
              "given as an edge ID in a single column of a CSV file.\n"
              "  -g <file>         input road network in binary format.\n"
              "  -l <file>         input locations (edge IDs) in CSV format.\n"
              "  -l-col-name <name>   name of the location column in the CSV file, default: 'location'.\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}

// Dark-ish orange used as stroke color for the location edges.
static constexpr const char *LOCATION_STROKE_COLOR = "#CC6600";

template<typename InputGraphT>
nlohmann::json generateGeoJsonFeatureForEdge(const InputGraphT &inputGraph, const int e, const int locId) {
    nlohmann::json feature;
    feature["type"] = "Feature";

    feature["properties"] = {{"stroke",      LOCATION_STROKE_COLOR},
                             {"edge_id",     e},
                             {"location_id", locId}};

    nlohmann::json geometry;
    geometry["type"] = "LineString";

    const auto tailLatLng = inputGraph.latLng(inputGraph.edgeTail(e));
    geometry["coordinates"].push_back(nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()}));

    const auto headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
    geometry["coordinates"].push_back(nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()}));

    feature["geometry"] = geometry;

    return feature;
}

template<typename InputGraphT>
nlohmann::json generateGeoJsonObjectForLocations(const InputGraphT &inputGraph, const std::vector<int> &locations) {
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "FeatureCollection";

    for (int i = 0; i < static_cast<int>(locations.size()); ++i) {
        topGeoJson["features"].push_back(generateGeoJsonFeatureForEdge(inputGraph, locations[i], i));
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
        auto locationsFileName = clp.getValue<std::string>("l");
        if (!endsWith(locationsFileName, ".csv"))
            locationsFileName += ".csv";
        const auto locationColName = clp.getValue<std::string>("l-col-name", "location");
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
        FORALL_VALID_EDGES(inputGraph, v, e) {
            inputGraph.edgeTail(e) = v;
        }
        std::cout << "done.\n";

        // Read the locations from file.
        std::cout << "Reading locations from file... " << std::flush;
        std::vector<int> locations;
        int location;
        io::CSVReader<1, io::trim_chars<' '>> locFileReader(locationsFileName);
        locFileReader.read_header(io::ignore_extra_column, locationColName);
        while (locFileReader.read_row(location)) {
            if (location < 0 || location >= inputGraph.numEdges())
                throw std::invalid_argument("location is not a valid edge ID -- '" + std::to_string(location) + "'");
            locations.push_back(location);
        }
        std::cout << "done.\n";

        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson = generateGeoJsonObjectForLocations(inputGraph, locations);
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
