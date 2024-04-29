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


#include <cassert>
#include <kassert/kassert.hpp>
#include "Tools/custom_assertion_levels.h"
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <csv.h>

#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/Logging/LogManager.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"

#include "Algorithms/FindMixedFixedFlexibleNetwork/PickupDropoffManager.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/GreedyFixedLineFinder.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/DirectDistancesFinder.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/PreliminaryPaths.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/InputConfig.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "Algorithms/KaRRi/CHEnvironment.h"
#include <vector>

inline void printUsage() {
    std::cout <<
              "Usage: mixfix -veh-g <vehicle network> -psg-g <passenger network> -r <requests> -v <vehicles> -o <file>\n"
              "Runs process to find mixed fixed-flexible bus public transport networks with given vehicle and passenger\n"
              "road networks and demand. Writes output files to specified base path."
              "  -veh-g <file>          vehicle road network in binary format.\n"
              "  -psg-g <file>          passenger road (and path) network in binary format.\n"
              "  -r <file>              requests in CSV format.\n"
              //              "  -v <file>              vehicles in CSV format.\n"
              "  -a <factor>            model parameter alpha for max trip time = a * OD-dist + b (dflt: 1.7)\n"
              "  -b <seconds>           model parameter beta for max trip time = a * OD-dist + b (dflt: 120)\n"
              "  -rho <sec>             walking radius (in s) for PD-Locs around origin/destination. (dflt: 300s)\n"
              "  -veh-h <file>          contraction hierarchy for the vehicle network in binary format.\n"
              "  -psg-h <file>          contraction hierarchy for the passenger network in binary format.\n"
              "  -veh-d <file>          separator decomposition for the vehicle network in binary format (needed for CCHs).\n"
              "  -psg-d <file>          separator decomposition for the passenger network in binary format (needed for CCHs).\n"
              "  -o <file>              generate output files at name <file> (specify name without file suffix).\n"
              "  -help                  show usage help text.\n";
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
    using namespace mixfix;
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        InputConfig &inputConfig = InputConfig::getInstance();
        inputConfig.minMaxFlowOnLine = clp.getValue<int>("min-max-line-flow", 5); // Delta1 in paper
        inputConfig.maxFlowRatioOnLine = clp.getValue<double>("max-flow-dif", 10.0); // Delta2 in paper
        inputConfig.minNumPaxPerLine = clp.getValue<int>("min-num-line-pax", 5); // Delta3 in paper

        inputConfig.walkingRadius = clp.getValue<int>("rho", 300) * 10;
        inputConfig.alpha = clp.getValue<double>("a", 1.7);
        inputConfig.beta = clp.getValue<int>("b", 120) * 10;

        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
//        const auto vehicleFileName = clp.getValue<std::string>("v");
        const auto requestFileName = clp.getValue<std::string>("r");
        const auto pathsFileName = clp.getValue<std::string>("p");
        const auto vehHierarchyFileName = clp.getValue<std::string>("veh-h");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto vehSepDecompFileName = clp.getValue<std::string>("veh-d");
        const auto psgSepDecompFileName = clp.getValue<std::string>("psg-d");
        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);

        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, CarEdgeToPsgEdgeAttribute, OsmRoadCategoryAttribute>;
        using VehicleInputGraph = StaticGraph<VehicleVertexAttributes, VehicleEdgeAttributes>;
        std::ifstream vehicleNetworkFile(vehicleNetworkFileName, std::ios::binary);
        if (!vehicleNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + vehicleNetworkFileName + "'");
        VehicleInputGraph vehicleInputGraph(vehicleNetworkFile);
        vehicleNetworkFile.close();
        std::vector<int32_t> vehGraphOrigIdToSeqId;
        if (vehicleInputGraph.numEdges() > 0 && vehicleInputGraph.edgeId(0) == INVALID_ID) {
            vehGraphOrigIdToSeqId.assign(vehicleInputGraph.numEdges(), INVALID_ID);
            std::iota(vehGraphOrigIdToSeqId.begin(), vehGraphOrigIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                    assert(vehicleInputGraph.edgeId(e) == INVALID_ID);
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        } else {
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                    assert(vehicleInputGraph.edgeId(e) != INVALID_ID);
                    if (vehicleInputGraph.edgeId(e) >= vehGraphOrigIdToSeqId.size()) {
                        const auto numElementsToBeInserted =
                                vehicleInputGraph.edgeId(e) + 1 - vehGraphOrigIdToSeqId.size();
                        vehGraphOrigIdToSeqId.insert(vehGraphOrigIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                    }
                    assert(vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] == INVALID_ID);
                    vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] = e;
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        }
        std::cout << "done.\n";

        // Read the passenger network from file.
        std::cout << "Reading passenger network from file... " << std::flush;
        using PsgVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, PsgEdgeToCarEdgeAttribute, TravelTimeAttribute>;
        using PsgInputGraph = StaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        assert(psgInputGraph.numEdges() > 0 && psgInputGraph.edgeId(0) == INVALID_ID);
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
                assert(psgInputGraph.edgeId(e) == INVALID_ID);
                psgInputGraph.edgeTail(e) = v;
                psgInputGraph.edgeId(e) = e;

                const int eInVehGraph = psgInputGraph.toCarEdge(e);
                if (eInVehGraph != PsgEdgeToCarEdgeAttribute::defaultValue()) {
                    ++numEdgesWithMappingToCar;
                    assert(eInVehGraph < vehGraphOrigIdToSeqId.size());
                    psgInputGraph.toCarEdge(e) = vehGraphOrigIdToSeqId[eInVehGraph];
                    assert(psgInputGraph.toCarEdge(e) < vehicleInputGraph.numEdges());
                    vehicleInputGraph.toPsgEdge(psgInputGraph.toCarEdge(e)) = e;

                    assert(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                           vehicleInputGraph.latLng(vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).latitude());
                    assert(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(
                            vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).longitude());
                }
            }
        unused(numEdgesWithMappingToCar);
        assert(numEdgesWithMappingToCar > 0);
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<Request> requests;
        int origin, destination, requestTime;
        io::CSVReader<3, io::trim_chars<' '>> reqFileReader(requestFileName);
        reqFileReader.read_header(io::ignore_extra_column, "origin", "destination", "req_time");

        while (reqFileReader.read_row(origin, destination, requestTime)) {
            if (origin < 0 || origin >= vehGraphOrigIdToSeqId.size() || vehGraphOrigIdToSeqId[origin] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination >= vehGraphOrigIdToSeqId.size() ||
                vehGraphOrigIdToSeqId[destination] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            const auto originSeqId = vehGraphOrigIdToSeqId[origin];
            assert(vehicleInputGraph.toPsgEdge(originSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const auto destSeqId = vehGraphOrigIdToSeqId[destination];
            assert(vehicleInputGraph.toPsgEdge(destSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const int requestId = static_cast<int>(requests.size());
            requests.push_back({requestId, originSeqId, destSeqId, requestTime * 10});
        }
        std::cout << "done.\n";

        // Read the preliminary paths from file.
        std::cout << "Reading preliminary paths from file... " << std::flush;
        int requestId;
        std::string pathString;
        io::CSVReader<2, io::trim_chars<' '>> pathFileReader(pathsFileName);
        pathFileReader.read_header(io::ignore_extra_column, "request_id", "path_as_graph_edge_ids");

        PreliminaryPaths preliminaryPaths;
        preliminaryPaths.init(requests.size());
        while (pathFileReader.read_row(requestId, pathString)) {
            if (requestId < 0 || requestId >= requests.size())
                throw std::invalid_argument("invalid request id -- '" + std::to_string(requestId) + "'");
            std::vector<int> edges = parseEdgePathString(pathString);
            assert(std::all_of(edges.begin(), edges.end(),
                               [&](const int e) { return e < vehicleInputGraph.numEdges(); }));
            if (edges.empty())
                continue;
            preliminaryPaths.addPathForRequest(requestId, std::move(edges));
        }
        std::cout << "done.\n";


        // Prepare vehicle CH environment
        using VehCHEnv = karri::CHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehHierarchyFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the CH from file.
            std::cout << "Reading CH from file... " << std::flush;
            std::ifstream vehHierarchyFile(vehHierarchyFileName, std::ios::binary);
            if (!vehHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + vehHierarchyFileName + "'");
            CH vehCh(vehHierarchyFile);
            vehHierarchyFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(std::move(vehCh));
        }

        // Prepare passenger CH environment
        using PsgCHEnv = karri::CHEnvironment<PsgInputGraph, TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgHierarchyFileName.empty()) {
            std::cout << "Building passenger CH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the passenger CH from file.
            std::cout << "Reading passenger CH from file... " << std::flush;
            std::ifstream psgHierarchyFile(psgHierarchyFileName, std::ios::binary);
            if (!psgHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + psgHierarchyFileName + "'");
            CH psgCh(psgHierarchyFile);
            psgHierarchyFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(std::move(psgCh));
        }


        const auto revVehicleGraph = vehicleInputGraph.getReverseGraph();
        const auto revPsgGraph = psgInputGraph.getReverseGraph();

        using PickupDropoffManagerImpl = PickupDropoffManager<VehicleInputGraph, PsgInputGraph>;
        PickupDropoffManagerImpl pdManager(vehicleInputGraph, psgInputGraph, revPsgGraph);

        using FixedLineFinder = GreedyFixedLineFinder<VehicleInputGraph, PreliminaryPaths, PickupDropoffManagerImpl, std::ofstream>;
        FixedLineFinder lineFinder(vehicleInputGraph, revVehicleGraph, pdManager, requests);

        std::cout << "Finding PD-Locs ..." << std::flush;
        pdManager.findPossiblePDLocsForRequests(requests);
        std::cout << "done.\n";

        std::cout << "Computing direct distances ..." << std::flush;
        DirectDistancesFinder<VehicleInputGraph, VehCHEnv> directDistancesFinder(vehicleInputGraph, *vehChEnv);
        directDistancesFinder.computeDirectDists(requests);
        std::cout << "done.\n";

        const auto numNonEmptyPaths = preliminaryPaths.numPaths();
        std::cout << "Constructing lines ..." << std::flush;
        lineFinder.findFixedLines(preliminaryPaths);
        std::cout << "done.\n";


        std::cout << "Could serve " << numNonEmptyPaths - preliminaryPaths.numPaths()  << " / " << requests.size()
                  << " requests with fixed lines." << std::endl;

        std::cout << "Writing GeoJSON to output file... " << std::flush;
        // Open the output file and write the GeoJson.
        std::ofstream geoJsonOutputFile(outputFileName + ".geojson");
        if (!geoJsonOutputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".geojson" + "'");
        geoJsonOutputFile << std::setw(2) << lineFinder.getLinesGeoJson() << std::endl;
        std::cout << "done.\n";

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}