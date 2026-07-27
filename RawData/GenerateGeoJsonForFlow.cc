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
              "Usage: GenerateGeoJsonForFlow -g <file> -o <file>\n"
              "Outputs GeoJson depicting a flow in a road network.\n"
              "  -g <file>         input road network in binary format.\n"
              "  -f <file>         flow per edge in CSV file.\n"
              "  -o <file>         place GeoJSON in <file>\n"
              "  -help             display this help and exit\n";
}

/**
 * Maps a value between 0 and 1 to a hex color code in a yellow-to-purple gradient.
 * @param v Value between 0 and 1.
 * @return Hex color code (e.g., "#FFFF00").
 */
std::string valueToHexColor(double v) {
    if (v < 0.0 || v > 1.0) {
        throw std::invalid_argument("Value must be between 0 and 1.");
    }

    // Start (yellow) and end (purple) RGB values
    const int start_r = 255, start_g = 255, start_b = 0;
    const int end_r = 128, end_g = 0, end_b = 128;

    // Interpolate each component
    int r = std::round(start_r + (end_r - start_r) * v);
    int g = std::round(start_g + (end_g - start_g) * v);
    int b = std::round(start_b + (end_b - start_b) * v);

    // Clamp values to [0, 255]
    r = std::max(0, std::min(255, r));
    g = std::max(0, std::min(255, g));
    b = std::max(0, std::min(255, b));

    // Convert to hex string
    std::stringstream ss;
    ss << "#"
       << std::hex << std::setw(2) << std::setfill('0') << r
       << std::hex << std::setw(2) << std::setfill('0') << g
       << std::hex << std::setw(2) << std::setfill('0') << b;

    return ss.str();
}

template<typename InputGraphT>
nlohmann::json generateGeoJsonObjectForEdges(const InputGraphT &inputGraph, const std::vector<int> &flows) {
    // Construct the needed path GeoJSON objects
    nlohmann::json topGeoJson;
    topGeoJson["type"] = "FeatureCollection";

    std::vector<double> normalizedLogFlows(flows.size());
    const int maxFlow = *std::ranges::max_element(flows);
    const double maxLogFlow = std::log2(maxFlow + 1);
    for (size_t i = 0; i < flows.size(); ++i) {
        normalizedLogFlows[i] = std::log2(static_cast<double>(flows[i] + 1)) / maxLogFlow;
    }

    static constexpr double minStrokeWidth = 0.5;
    static constexpr double maxStrokeWidth = 5.0;

    FORALL_VALID_EDGES(inputGraph, v, e) {
    const double strokeWidth = normalizedLogFlows[e] * (maxStrokeWidth - minStrokeWidth) + minStrokeWidth;

            // Construct edge feature
            nlohmann::json feature;
            feature["type"] = "Feature";
            feature["properties"] = {{"edge_id",      e},
                                     {"stroke",       valueToHexColor(normalizedLogFlows[e])},
                                     {"stroke-width", strokeWidth}
            };

            nlohmann::json edgeGeometry;
            edgeGeometry["type"] = "LineString";

            const auto tailLatLng = inputGraph.latLng(v);
            const auto tailCoordinate = nlohmann::json::array({tailLatLng.lngInDeg(), tailLatLng.latInDeg()});
            edgeGeometry["coordinates"].push_back(tailCoordinate);

            const auto headLatLng = inputGraph.latLng(inputGraph.edgeHead(e));
            const auto headCoordinate = nlohmann::json::array({headLatLng.lngInDeg(), headLatLng.latInDeg()});
            edgeGeometry["coordinates"].push_back(headCoordinate);

            feature["geometry"] = edgeGeometry;
            topGeoJson["features"].push_back(feature);
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
        auto flowsFileName = clp.getValue<std::string>("f");
        if (!endsWith(flowsFileName, ".csv"))
            flowsFileName += ".csv";
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

        // Read edge flows from file
        std::vector<int> flows;
        int flow;
        io::CSVReader<1, io::trim_chars<' '>> flowsFileReader(flowsFileName);
        flowsFileReader.read_header(io::ignore_no_column, "flow");
        while (flowsFileReader.read_row(flow)) {
            KASSERT(flow >= 0, "Flow value must be non-negative.");
            flows.push_back(flow);
        }
        KASSERT(flows.size() == inputGraph.numEdges(), "Number of flow values must match number of edges in the graph.");


        std::cout << "Generating GeoJson object ..." << std::flush;
        nlohmann::json geoJson = generateGeoJsonObjectForEdges(inputGraph, flows);
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