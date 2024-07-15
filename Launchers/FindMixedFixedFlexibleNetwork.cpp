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
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"

#include "Algorithms/FindMixedFixedFlexibleNetwork/PickupDropoffManager.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/GreedyFixedLineFinder.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/DirectDistancesFinder.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/PreliminaryPaths.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/InputConfig.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "Algorithms/KaRRi/CHEnvironment.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/RequestState/FindPDLocsInRadiusQuery.h"
#include <vector>

inline void printUsage() {
    std::cout <<
              "Usage: mixfix -veh-g <vehicle network> -psg-g <passenger network> -r <requests> -p <paths> -v <vehicles> -o <file>\n"
              "Runs process to find mixed fixed-flexible bus public transport networks with given vehicle and passenger\n"
              "road networks and demand. Writes output files to specified base path."
              "  -veh-g <file>              vehicle road network in binary format.\n"
              "  -psg-g <file>              passenger road (and path) network in binary format.\n"
              "  -r <file>                  requests in CSV format.\n"
              "  -p <file>                  preliminary paths in CSV format.\n"
              "  -pd <file>                 PD-locations for requests in binary format (see ComputePDLocsForRequests).\n"
              "  -veh-h <file>              contraction hierarchy for the vehicle network in binary format (optional).\n"
              "  -psg-h <file>              contraction hierarchy for the passenger network in binary format (optional).\n"
              "  -a <factor>                model parameter alpha for max trip time = a * OD-dist + b (dflt: 1.0)\n"
              "  -b <seconds>               model parameter beta for max trip time = a * OD-dist + b (dflt: 900)\n"
              "  -min-max-line-flow <int>   stops constructing lines if flow of initial edge is < given value (dflt: 1)\n"
              "  -max-flow-dif <float>      stops extending a line if max flow / min flow on line is >= given value (dflt: unlimited)\n"
              "  -min-num-line-pax <int>    stops constructing lines if a line found has fewer passengers than given value (dflt: 1)\n"
              "  -overlap-score-exp <float> exponent for score of longer overlaps when finding next line edge (dflt: 1)\n"
              "  -o <file>                  generate output files at name <file> (specify name without file suffix).\n"
              "  -output-paths              If set, also outputs paths as GeoJSON and CSV.\n"
              "  -help                      show usage help text.\n";
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

template<typename VehInputGraphT, typename PdManagerT>
void verifyPathViability(const mixfix::PreliminaryPaths &paths, const VehInputGraphT &inputGraph,
                         const PdManagerT &pdInfo) {

//    Subset edgesInPath(inputGraph.numEdges());

    for (const auto &path: paths) {
//        edgesInPath.clear();
        int prevVertex = path.size() == 0 ? INVALID_VERTEX : inputGraph.edgeTail(path[0]);
        for (int i = 0; i < path.size() - 1; ++i) {
            const auto nextPathEdge = path[i];
            LIGHT_KASSERT(inputGraph.edgeTail(nextPathEdge) == prevVertex,
                          "Previous vertex " << prevVertex << " is not tail of next edge " << nextPathEdge
                                             << " on path.");
//            if (!edgesInPath.insert(nextPathEdge))
//                throw std::invalid_argument("Edge " + std::to_string(nextPathEdge) + " is visited twice in path for request " +
//                                            std::to_string(path.getRequestId()));
            bool found = false;
            FORALL_INCIDENT_EDGES(inputGraph, prevVertex, e) {
                if (e == nextPathEdge) {
                    found = true;
                    break;
                }
            }
            if (!found)
                throw std::invalid_argument(
                        "Cannot find edge " + std::to_string(nextPathEdge) + " incident to vertex " +
                        std::to_string(prevVertex) + " in path for request " + std::to_string(path.getRequestId()));

            prevVertex = inputGraph.edgeHead(nextPathEdge);
        }

        if (path.size() > 0) {
            const auto &pickups = pdInfo.getPossiblePickupsAt(inputGraph.edgeTail(path.front()));
            if (!contains(pickups.begin(), pickups.end(), path.getRequestId()))
                throw std::invalid_argument("Path for request " + std::to_string(path.getRequestId()) +
                                            " does not start at a possible pickup vertex for that request.");

            const auto &dropoffs = pdInfo.getPossibleDropoffsAt(inputGraph.edgeHead(path.back()));
            if (!contains(dropoffs.begin(), dropoffs.end(), path.getRequestId()))
                throw std::invalid_argument("Path for request " + std::to_string(path.getRequestId()) +
                                            " does not end at a possible dropoff vertex for that request.");
        }
    }
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
        inputConfig.minMaxFlowOnLine = clp.getValue<int>("min-max-line-flow", 1); // Delta1 in paper
        inputConfig.maxFlowRatioOnLine = clp.getValue<double>("max-flow-dif", std::numeric_limits<double>::max()); // Delta2 in paper
        inputConfig.minNumPaxPerLine = clp.getValue<int>("min-num-line-pax", 1); // Delta3 in paper
        inputConfig.overlapScoreExponent = clp.getValue<double>("overlap-score-exp", 1.0); // Exponent for weighting overlap length

        inputConfig.alpha = clp.getValue<double>("a", 1.0);
        inputConfig.beta = clp.getValue<int>("b", 900) * 10;

        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto requestFileName = clp.getValue<std::string>("r");
        const auto pathsFileName = clp.getValue<std::string>("p");
        const auto pdLocsFileName = clp.getValue<std::string>("pd");
        const auto vehHierarchyFileName = clp.getValue<std::string>("veh-h");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto vehSepDecompFileName = clp.getValue<std::string>("veh-d");
        const auto psgSepDecompFileName = clp.getValue<std::string>("psg-d");
        const auto outputFullPaths = clp.isSet("output-paths");
        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);

        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, MapToEdgeInPsgAttribute, OsmRoadCategoryAttribute>;
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
                    LIGHT_KASSERT(vehicleInputGraph.edgeId(e) == INVALID_ID);
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        } else {
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                    LIGHT_KASSERT(vehicleInputGraph.edgeId(e) != INVALID_ID);
                    if (vehicleInputGraph.edgeId(e) >= vehGraphOrigIdToSeqId.size()) {
                        const auto numElementsToBeInserted =
                                vehicleInputGraph.edgeId(e) + 1 - vehGraphOrigIdToSeqId.size();
                        vehGraphOrigIdToSeqId.insert(vehGraphOrigIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                    }
                    LIGHT_KASSERT(vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] == INVALID_ID);
                    vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] = e;
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        }

        auto revVehicleGraph = vehicleInputGraph.getReverseGraph();
        FORALL_VALID_EDGES(revVehicleGraph, u, e) {
                revVehicleGraph.edgeTail(e) = u;
            }
        std::cout << "done.\n";

        // Read the passenger network from file.
        std::cout << "Reading passenger network from file... " << std::flush;
        using PsgVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, MapToEdgeInFullVehAttribute, TravelTimeAttribute>;
        using PsgInputGraph = StaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
                assert(psgInputGraph.edgeId(e) == INVALID_ID);
                psgInputGraph.edgeTail(e) = v;
                psgInputGraph.edgeId(e) = e;

                const int eInVehGraph = psgInputGraph.mapToEdgeInFullVeh(e);
                if (eInVehGraph != MapToEdgeInFullVehAttribute::defaultValue()) {
                    ++numEdgesWithMappingToCar;
                    LIGHT_KASSERT(eInVehGraph < vehGraphOrigIdToSeqId.size());
                    psgInputGraph.mapToEdgeInFullVeh(e) = vehGraphOrigIdToSeqId[eInVehGraph];
                    LIGHT_KASSERT(psgInputGraph.mapToEdgeInFullVeh(e) < vehicleInputGraph.numEdges());
                    vehicleInputGraph.mapToEdgeInPsg(psgInputGraph.mapToEdgeInFullVeh(e)) = e;

                    LIGHT_KASSERT(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                                  vehicleInputGraph.latLng(
                                          vehicleInputGraph.edgeHead(psgInputGraph.mapToEdgeInFullVeh(e))).latitude());
                    LIGHT_KASSERT(
                            psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(
                                    vehicleInputGraph.edgeHead(psgInputGraph.mapToEdgeInFullVeh(e))).longitude());
                }
            }
        unused(numEdgesWithMappingToCar);
        LIGHT_KASSERT(numEdgesWithMappingToCar > 0);

        auto revPsgGraph = psgInputGraph.getReverseGraph();
        FORALL_VALID_EDGES(revPsgGraph, u, e) {
                revPsgGraph.edgeTail(e) = u;
            }
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
            LIGHT_KASSERT(vehicleInputGraph.mapToEdgeInPsg(originSeqId) != MapToEdgeInPsgAttribute::defaultValue());
            const auto destSeqId = vehGraphOrigIdToSeqId[destination];
            LIGHT_KASSERT(vehicleInputGraph.mapToEdgeInPsg(destSeqId) != MapToEdgeInPsgAttribute::defaultValue());
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
            LIGHT_KASSERT(std::all_of(edges.begin(), edges.end(),
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

        std::cout << "Reading PD-Locs from file... " << std::flush;

        std::ifstream pdLocsFile(pdLocsFileName, std::ios::binary);
        if (!pdLocsFile.good())
            throw std::invalid_argument("file not found -- '" + pdLocsFileName + "'");
        PickupDropoffInfo pdInfo(pdLocsFile);
        pdLocsFile.close();
        std::cout << "done.\n";


        std::cout << "Verifying input paths ..." << std::flush;
        verifyPathViability(preliminaryPaths, vehicleInputGraph, pdInfo);
        std::cout << "done.\n";

        std::cout << "Computing direct distances ..." << std::flush;
        DirectDistancesFinder<VehicleInputGraph, VehCHEnv> directDistancesFinder(vehicleInputGraph, *vehChEnv);
        directDistancesFinder.computeDirectDists(requests);
        std::cout << "done.\n";

        const auto numNonEmptyPaths = preliminaryPaths.numPaths();
        std::cout << "Constructing lines ..." << std::flush;

        using OverviewLogger = std::ofstream;
        if (!outputFullPaths) {
            using PathLogger = NullLogger;
            using FixedLineFinder = GreedyFixedLineFinder<VehicleInputGraph, PreliminaryPaths, OverviewLogger, PathLogger>;
            FixedLineFinder lineFinder(vehicleInputGraph, revVehicleGraph, pdInfo, requests);
            lineFinder.findFixedLines(preliminaryPaths, false);
            std::cout << "done.\n";
        } else {
            using PathLogger = std::ofstream;
            using FixedLineFinder = GreedyFixedLineFinder<VehicleInputGraph, PreliminaryPaths, OverviewLogger, PathLogger>;
            FixedLineFinder lineFinder(vehicleInputGraph, revVehicleGraph, pdInfo, requests);
            lineFinder.findFixedLines(preliminaryPaths, true);
            std::cout << "done.\n";

            std::cout << "Writing GeoJSON to output file... " << std::flush;
            // Open the output file and write the GeoJson.
            std::ofstream geoJsonOutputFile(outputFileName + ".geojson");
            if (!geoJsonOutputFile.good())
                throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".geojson" + "'");
            geoJsonOutputFile << std::setw(2) << lineFinder.getLinesGeoJson() << std::endl;
            std::cout << "done.\n";
        }

        std::cout << "Could serve " << numNonEmptyPaths - preliminaryPaths.numPaths() << " / " << requests.size()
                  << " requests with fixed lines." << std::endl;

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}