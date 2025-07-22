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

#include <random>
#include <iomanip>
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Geometry/Area.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"

inline void printUsage() {
    std::cout <<
              "Usage: CategorizeRequestsByCoreAndPeriphery -g <file> -r <file> -a <file>  -o <file>\n"
              "Given a road network and requests in the network, as well as a polygon describing a core area of the\n"
              "network, this program outputs four files, containing all journeys from core to core,\n"
              "from periphery to core, from core to periphery, and from periphery to periphery.\n"
              "  -g <file>         input graph in binary format\n"
              "  -r <file>         requests file in CSV format\n"
              "  -a <file>         core area of network in Osmosis polygon (.poly) format\n"
              "  -o <file>         place output in <file> (file ending will be ignored)\n"
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
        const auto graphFileName = clp.getValue<std::string>("g");
        const auto requestsFileName = clp.getValue<std::string>("r");
        const auto areaFileName = clp.getValue<std::string>("a");
        auto outputFileName = clp.getValue<std::string>("o");
        outputFileName.substr(0, outputFileName.find_last_of('.'));


        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute>>;
        std::ifstream inputGraphFile(graphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
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
        std::vector<karri::Request> requests;
        int origin, destination, requestTime;
        io::CSVReader<3, io::trim_chars<' '>> reqFileReader(requestsFileName);
        reqFileReader.read_header(io::ignore_no_column, "origin", "destination", "req_time");


        while (reqFileReader.read_row(origin, destination, requestTime)) {
            if (origin < 0 || origin >= origIdToSeqId.size() || origIdToSeqId[origin] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination >= origIdToSeqId.size() ||
                origIdToSeqId[destination] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            const auto originSeqId = origIdToSeqId[origin];
            const auto destSeqId = origIdToSeqId[destination];
            const int requestId = static_cast<int>(requests.size());
            assert(inputGraph.edgeTail(originSeqId) != EdgeTailAttribute::defaultValue());
            assert(inputGraph.edgeTail(destSeqId) != EdgeTailAttribute::defaultValue());
            requests.push_back({requestId, originSeqId, destSeqId, requestTime * 10});
        }
        std::cout << "done.\n";

        // Read the study area from OSM POLY file.
        std::cout << "Reading study area from OSM POLY file..." << std::flush;
        Area studyArea;
        studyArea.importFromOsmPolyFile(areaFileName);
        const auto box = studyArea.boundingBox();
        std::cout << " done.\n";

        // For each request, check if it is core-core, core-periphery, periphery-core, or periphery-periphery
        std::vector<karri::Request> coreCoreRequests;
        std::vector<karri::Request> corePeripheryRequests;
        std::vector<karri::Request> peripheryCoreRequests;
        std::vector<karri::Request> peripheryPeripheryRequests;
        for (const auto &request: requests) {
            const auto oLatLng = inputGraph.latLng(inputGraph.edgeHead(request.origin));
            const auto dLatLng = inputGraph.latLng(inputGraph.edgeHead(request.destination));
            const Point oP(oLatLng.longitude(), oLatLng.latitude());
            const Point dP(dLatLng.longitude(), dLatLng.latitude());
            bool oInCore = box.contains(oP) && studyArea.contains(oP);
            bool dInCore = box.contains(dP) && studyArea.contains(dP);
            if (oInCore && dInCore) {
                coreCoreRequests.push_back(request);
            } else if (oInCore) {
                corePeripheryRequests.push_back(request);
            } else if (dInCore) {
                peripheryCoreRequests.push_back(request);
            } else {
                peripheryPeripheryRequests.push_back(request);
            }
        }
        std::cout << "done." << std::endl;
        std::cout << "Request distribution: " << std::endl;
        std::cout << "\tCore-Core: " << coreCoreRequests.size() << std::setprecision(2) << " (" << static_cast<double>(coreCoreRequests.size()) / requests.size() * 100 << "%)" << std::endl;
        std::cout << "\tCore-Periphery: " << corePeripheryRequests.size() << std::setprecision(2) << " (" << static_cast<double>(corePeripheryRequests.size()) / requests.size() * 100 << "%)" << std::endl;
        std::cout << "\tPeriphery-Core: " << peripheryCoreRequests.size() << std::setprecision(2) << " (" << static_cast<double>(peripheryCoreRequests.size()) / requests.size() * 100 << "%)" << std::endl;
        std::cout << "\tPeriphery-Periphery: " << peripheryPeripheryRequests.size() << std::setprecision(2) << " (" << static_cast<double>(peripheryPeripheryRequests.size()) / requests.size() * 100 << "%)" << std::endl;

        // Write the requests to the output files
        std::cout << "Writing requests to output files..." << std::flush;
        std::ofstream coreCoreFile(outputFileName + "-core2core.csv");
        std::ofstream corePeripheryFile(outputFileName + "-core2periphery.csv");
        std::ofstream peripheryCoreFile(outputFileName + "-periphery2core.csv");
        std::ofstream peripheryPeripheryFile(outputFileName + "-periphery2periphery.csv");
        if (!coreCoreFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "-core2core.csv'");
        if (!corePeripheryFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "-core2periphery.csv'");
        if (!peripheryCoreFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "-periphery2core.csv'");
        if (!peripheryPeripheryFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "-periphery2periphery.csv'");
        coreCoreFile << "origin,destination,req_time,original_id\n";
        corePeripheryFile << "origin,destination,req_time,original_id\n";
        peripheryCoreFile << "origin,destination,req_time,original_id\n";
        peripheryPeripheryFile << "origin,destination,req_time,original_id\n";
        for (const auto &request: coreCoreRequests) {
            coreCoreFile << request.origin << "," << request.destination << "," << request.requestTime << "," << request.requestId << "\n";
        }
        for (const auto &request: corePeripheryRequests) {
            corePeripheryFile << request.origin << "," << request.destination << "," << request.requestTime << "," << request.requestId << "\n";
        }
        for (const auto &request: peripheryCoreRequests) {
            peripheryCoreFile << request.origin << "," << request.destination << "," << request.requestTime << "," << request.requestId << "\n";
        }
        for (const auto &request: peripheryPeripheryRequests) {
            peripheryPeripheryFile << request.origin << "," << request.destination << "," << request.requestTime << "," << request.requestId << "\n";
        }
        coreCoreFile.close();
        corePeripheryFile.close();
        peripheryCoreFile.close();
        peripheryPeripheryFile.close();
        std::cout << "done." << std::endl;

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}