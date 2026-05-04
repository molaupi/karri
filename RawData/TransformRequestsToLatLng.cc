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

#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include "DataStructures/Utilities/OriginDestination.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"

inline void printUsage() {
    std::cout <<
              "Usage: TransformRequestsToLatLng -veh-g <file> -psg-g <file> -r <file> -o <file>\n"
              "Given a request file, the vehicle graph, and the passenger graph, transforms the origin and destination "
              "locations (given as edge IDs in the vehicle graph) to corresponding edges in the passenger graph, "
              "and then to latitude/longitude of edge heads and writes these coordinates to file.\n"
              "pairs of edges to the output file.\n\n"
              "  -veh-g <file>     vehicle graph in binary format\n"
              "  -psg-g <file>     passenger graph in binary format\n"
              "  -r <file>         requests file in CSV format\n"
              "  -add-time-offset <int>  time offset to add to all request times (in seconds).\n"
              "  -sub-time-offset <int>  time offset to subtract from all request times (in seconds).\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto vehicleGraphFileName = clp.getValue<std::string>("veh-g");
        const auto passengerGraphFileName = clp.getValue<std::string>("psg-g");
        const auto requestsFileName = clp.getValue<std::string>("r");
        const auto addTimeOffset = clp.getValue<int>("add-time-offset", 0);
        const auto subTimeOffset = clp.getValue<int>("sub-time-offset", 0);
        const int timeOffset = addTimeOffset - subTimeOffset;
        std::cout << "Using total time offset of " << timeOffset << " seconds.\n";
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";


        // Read the source network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleGraph = StaticGraph<VertexAttrs<>, EdgeAttrs<CarEdgeToPsgEdgeAttribute>>;
        std::ifstream vehicleGraphFile(vehicleGraphFileName, std::ios::binary);
        if (!vehicleGraphFile.good())
            throw std::invalid_argument("file not found -- '" + vehicleGraphFileName + "'");
        VehicleGraph vehicleInputGraph(vehicleGraphFile);
        vehicleGraphFile.close();
        std::cout << "done.\n";

        std::cout << "Reading passenger network from file... " << std::flush;
        using PassengerGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<>>;
        std::ifstream passengerGraphFile(passengerGraphFileName, std::ios::binary);
        if (!passengerGraphFile.good())
            throw std::invalid_argument("file not found -- '" + passengerGraphFileName + "'");
        PassengerGraph psgInputGraph(passengerGraphFile);
        passengerGraphFile.close();
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<std::tuple<LatLng, LatLng, int>> odPairs;
        int origin, destination, reqTime;
        io::CSVReader<3, io::trim_chars<' '>> reqFileReader(requestsFileName);
        reqFileReader.read_header(io::ignore_extra_column, "origin", "destination", "req_time");

        while (reqFileReader.read_row(origin, destination, reqTime)) {
            const int oPsgEdge = vehicleInputGraph.toPsgEdge(origin);
            const int dPsgEdge = vehicleInputGraph.toPsgEdge(destination);
            const auto oLatLng = psgInputGraph.latLng(psgInputGraph.edgeHead(oPsgEdge));
            const auto dLatLng = psgInputGraph.latLng(psgInputGraph.edgeHead(dPsgEdge));
            odPairs.push_back({oLatLng, dLatLng, reqTime + timeOffset});
        }
        std::cout << "done.\n";

        // Write output csv file.
        std::cout << "Writing output file... " << std::flush;
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        outputFile << "lat_origin,lon_origin,lat_destination,lon_destination,req_time\n";
        for (const auto& od : odPairs) {
            outputFile << get<0>(od).latInDeg() << ',' << get<0>(od).lngInDeg() << ','
                       << get<1>(od).latInDeg() << ',' << get<1>(od).lngInDeg() << ','
                       << get<2>(od) << '\n';
        }
        outputFile.close();
        std::cout << "done.\n";

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}