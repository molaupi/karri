/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <fstream>
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Utilities/OriginDestination.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include "DataStructures/Geometry/Area.h"
#include "DataStructures/Geometry/CoordinateTransformation.h"
#include "Tools/CommandLine/ProgressBar.h"

#include <nlohmann/json.hpp>

inline void printUsage() {
    std::cout <<
              "Usage: AddRegioStaRClass -g <file> -r <file> -c <file> -o <file>\n"
              "Takes a set of requests and according graph as well as a RegioStaR area classification in GeoJson format\n"
              "and outputs the RegioStaR-2 classification of each request.\n"
              "  -g <file>              graph file in binary format\n"
              "  -r <file>              requests file in CSV format\n"
              "  -c <file>              RegioStaR classification in GeoJson format\n"
              "  -reverse-orientation   if set, reverse the orientation of polygons in the RegioStaR classification (default: false)\n"
              "  -cs <code>             EPSG code of coordinates in RegioStaR classification (dflt: 25832).\n"
              "  -o <file>              place output in <file>\n"
              "  -help                  display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto graphFileName = clp.getValue<std::string>("g");
        const auto requestsFileName = clp.getValue<std::string>("r");
        const auto regiostarFileName = clp.getValue<std::string>("c");
        const auto coordinateSystem = clp.getValue<int>("cs", 25832);
        unused(coordinateSystem);
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";

        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<CarEdgeToPsgEdgeAttribute>>;
        std::ifstream inputGraphFile(graphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph graph(inputGraphFile);
        inputGraphFile.close();
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<OriginDestination> pairs;
        int origin, destination;
        io::CSVReader<2, io::trim_chars<' '>> reqFileReader(requestsFileName);
        reqFileReader.read_header(io::ignore_extra_column, "origin", "destination");

        // Read origin and destination edges
        while (reqFileReader.read_row(origin, destination)) {
            assert(graph.toPsgEdge(origin) != CarEdgeToPsgEdgeAttribute::defaultValue());
            assert(graph.toPsgEdge(destination) != CarEdgeToPsgEdgeAttribute::defaultValue());
            pairs.emplace_back(origin, destination);
        }
        std::cout << "done.\n";

        std::cout << "Read RegioStaR classification..." << std::flush;
        std::ifstream ifs(regiostarFileName);
        nlohmann::json rs = nlohmann::json::parse(ifs);
        std::cout << "done.\n";

        std::cout << rs["type"] << " with " << rs["features"].size() << " features.\n";

        std::cout << "Compute city and country areas..." << std::flush;
        Area city;
        Area country;

        CoordinateTransformation trans(coordinateSystem, CoordinateTransformation::WGS_84);

        bool reverseOrientation = clp.isSet("reverse-orientation");
        for (const auto &feature: rs["features"]) {

            const int regiostar2Class = feature["properties"]["RegioStaR2"];

            for (const auto &partialArea: feature["geometry"]["coordinates"]) {
                for (const auto &polygonArray : partialArea) {
                    Polygon p;
                    for (const auto &point: polygonArray) {
                        const double xIn = point[0];
                        const double yIn = point[1];
                        double xOut, yOut;
                        trans.forward(xIn, yIn, xOut, yOut);
                        LatLng ll(yOut, xOut);
                        p.add(Point(ll.longitude(), ll.latitude()));
                    }

                    // Internal polygon representation expects outer polygons in counter-clockwise orientation while
                    // holes should be in clockwise orientation. The GeoJson standard demands the same but if the
                    // input file does it the other way around, we can reverse the orientation.
                    if (reverseOrientation)
                        p.reverseOrientation();

                    // GeoJson demands first and last point to be identical while internal polygon implementation
                    // implicitly closes polygon with edge between last and first point.
                    if (p[0] == p.back())
                        p.removeBack();

                    if (!p.simple())
                        throw std::invalid_argument("polygon is not simple");

                    if (regiostar2Class == 1)
                        city.combine(p);
                    else if (regiostar2Class == 2)
                        country.combine(p);
                    else
                        throw std::invalid_argument("unknown RegioStaR2 class '" + std::to_string(regiostar2Class) + "'");
                }
            }
        }
        std::cout << "done.\n";

        std::cout << "Classifying requests... " << std::flush;
        std::vector<std::pair<int, int>> rs2classification; // RegioStaR-2 classification for origin and destination
        rs2classification.reserve(pairs.size());
        ProgressBar progressBar(pairs.size(), true);
        for (const auto &pair: pairs) {
            const auto &originLatLng = graph.latLng(graph.edgeHead(pair.origin));
            const Point originP(originLatLng.longitude(), originLatLng.latitude());
            const auto &destinationLatLng = graph.latLng(graph.edgeHead(pair.destination));
            const Point destinationP(destinationLatLng.longitude(), destinationLatLng.latitude());

            int originClass = 0, destinationClass = 0;
            if (city.contains(originP))
                originClass = 1;
            else if (country.contains(originP))
                originClass = 2;
            else
                throw std::invalid_argument("Origin point " + latLngForCsv(originLatLng) + " not contained in any RegioStaR-2 area");

            if (city.contains(destinationP))
                destinationClass = 1;
            else if (country.contains(destinationP))
                destinationClass = 2;
            else
                throw std::invalid_argument("Destination point " + latLngForCsv(destinationLatLng) + " not contained in any RegioStaR-2 area");

            rs2classification.emplace_back(originClass, destinationClass);
            ++progressBar;
        }
        progressBar.finish();
        std::cout << "done.\n";

        // Write classification to output
        std::cout << "Writing classification to output...";
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        out << "rs2_origin,rs2_destination\n";
        for (const auto &[rs2o, rs2d]: rs2classification) {
            out << rs2o << ", " << rs2d << "\n";
        }
        std::cout << "done.\n";


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}