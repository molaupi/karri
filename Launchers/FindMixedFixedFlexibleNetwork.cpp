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
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"

#include "Algorithms/FindMixedFixedFlexibleNetwork/GreedyFixedLineFinder.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/PreliminaryPaths.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/InputConfig.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/AvoidLoopsStrategy.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/LinesGeoJson.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "Algorithms/KaRRi/CHEnvironment.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/PathStartEndInfo.h"
#include <vector>

inline void printUsage() {
    std::cout <<
              "Usage: mixfix -veh-g <vehicle network> -r <requests> -p <paths> -pd <pd-locs> -min-flow <min flow> -o <file>\n"
              "Runs process to find bus lines with given vehicle road network, OD-pairs, and according OD-paths and "
              "pickup/dropoff locations for every OD-pair. Writes output files to specified base path.\n\n"
              "  -veh-g <file>              vehicle road network in binary format.\n"
              "  -r <file>                  requests in CSV format.\n"
              "  -p <file>                  preliminary paths in CSV format.\n"
              "  -pd <file>                 PD-locations for requests in binary format (see ComputePDLocsForRequests).\n"
              "  -min-flow <int>            stops extending line if flow becomes smaller than given value, stops \n"
              "                             constructing lines if flow on initial edge is smaller than given value\n"
              "  -overlap-score-exp <float> exponent for score of longer overlaps when finding next line edge (dflt: 1.0)\n"
              "  -avoid-loops <strategy>    strategy to avoid loops in lines. Possible values are: \n"
              "                                 none                lines may visit vertices and edges multiple times (dflt)\n"
              "                                 vertex              stop extending if a vertex is visited the second time\n"
              "                                                     (strategy used in Fielbaum, Alonso-Mora paper)\n"
              "                                 edge                stop extending if a (directed) edge is visited the second time\n"
              "                                 vertex-rollback     stop extending if a vertex is visited the second time\n"
              "                                                     and roll line back to last useful edge\n"
              "                                 edge-rollback       stop extending if an edge is visited the second time\n"
              "                                                     and roll line back to last useful edge\n"
              "                                 edge-alternative    if extension edge has been visited before, attempt to find\n"
              "                                                     different viable extension\n"
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

template<typename VehInputGraphT, typename PathStartEndInfoT>
void verifyPathViability(const mixfix::PreliminaryPaths &paths, const VehInputGraphT &inputGraph,
                         const PathStartEndInfoT &pathStartEndInfo) {

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
//                                            std::to_string(path.getPathId()));
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
                        std::to_string(prevVertex) + " in path for request " + std::to_string(path.getPathId()));

            prevVertex = inputGraph.edgeHead(nextPathEdge);
        }

        if (path.size() > 0) {
            const auto &pickups = pathStartEndInfo.getPathsPossiblyBeginningAt(inputGraph.edgeTail(path.front()));
            if (!contains(pickups.begin(), pickups.end(), path.getPathId()))
                throw std::invalid_argument("Path for request " + std::to_string(path.getPathId()) +
                                            " does not start at a possible pickup vertex for that request.");

            const auto &dropoffs = pathStartEndInfo.getPathsPossiblyEndingAt(inputGraph.edgeHead(path.back()));
            if (!contains(dropoffs.begin(), dropoffs.end(), path.getPathId()))
                throw std::invalid_argument("Path for request " + std::to_string(path.getPathId()) +
                                            " does not end at a possible dropoff vertex for that request.");
        }
    }
}

template<mixfix::AvoidLoopsStrategy AvoidLoopsStrat, typename VehicleInputGraphT>
std::vector<mixfix::FixedLine>
runLineFinder(const VehicleInputGraphT &vehicleInputGraph, const VehicleInputGraphT &revVehicleGraph,
              mixfix::PathStartEndInfo &pathStartEndInfo, const std::vector<mixfix::Request> &requests,
              mixfix::PreliminaryPaths &preliminaryPaths) {

    using namespace mixfix;
    using FixedLineFinder = GreedyFixedLineFinder<VehicleInputGraphT, PreliminaryPaths, AvoidLoopsStrat, std::ofstream>;
    FixedLineFinder lineFinder(vehicleInputGraph, revVehicleGraph, pathStartEndInfo, requests);
    return lineFinder.findFixedLines(preliminaryPaths);
}


template<typename ...Args>
std::vector<mixfix::FixedLine>
decideAvoidLoopsStrategy(const mixfix::AvoidLoopsStrategy &avoidLoopsStrategy, Args &...args) {
    if (avoidLoopsStrategy == mixfix::AvoidLoopsStrategy::NONE) {
        return runLineFinder<mixfix::AvoidLoopsStrategy::NONE>(args...);
    }
    if (avoidLoopsStrategy == mixfix::AvoidLoopsStrategy::VERTEX) {
        return runLineFinder<mixfix::AvoidLoopsStrategy::VERTEX>(args...);
    }
    if (avoidLoopsStrategy == mixfix::AvoidLoopsStrategy::EDGE) {
        return runLineFinder<mixfix::AvoidLoopsStrategy::EDGE>(args...);
    }
    if (avoidLoopsStrategy == mixfix::AvoidLoopsStrategy::VERTEX_ROLLBACK) {
        return runLineFinder<mixfix::AvoidLoopsStrategy::VERTEX_ROLLBACK>(args...);
    }
    if (avoidLoopsStrategy == mixfix::AvoidLoopsStrategy::EDGE_ROLLBACK) {
        return runLineFinder<mixfix::AvoidLoopsStrategy::EDGE_ROLLBACK>(args...);
    }
    if (avoidLoopsStrategy == mixfix::AvoidLoopsStrategy::EDGE_ALTERNATIVE) {
        return runLineFinder<mixfix::AvoidLoopsStrategy::EDGE_ALTERNATIVE>(args...);
    }
    throw std::invalid_argument("No implementation for given avoid loops strategy.");
}


int main(int argc, char *argv[]) {
    using namespace mixfix;
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        if (!clp.isSet("min-flow"))
            throw std::invalid_argument("-min-flow command line argument not set!");

        // Parse the command-line options.
        InputConfig &inputConfig = InputConfig::getInstance();
        inputConfig.minFlowOnLine = clp.getValue<int>("min-flow", 100);
        inputConfig.overlapScoreExponent = clp.getValue<double>("overlap-score-exp", 1.0);

        const auto &avoidLoopsStrategy = EnumParser<AvoidLoopsStrategy>()(
                clp.getValue<std::string>("avoid-loops", "none"));

        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto requestFileName = clp.getValue<std::string>("r");
        const auto pathsFileName = clp.getValue<std::string>("p");
        const auto pdLocsFileName = clp.getValue<std::string>("pd");
        const auto vehHierarchyFileName = clp.getValue<std::string>("veh-h");
        const auto outputFullPaths = clp.isSet("output-paths");
        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);

        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, TravelTimeAttribute, MapToEdgeInPsgAttribute>;
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
            preliminaryPaths.addInitialPath(requestId, std::move(edges));
        }
        std::cout << "done.\n";

        std::cout << "Reading PD-Locs from file... " << std::flush;

        std::ifstream pdLocsFile(pdLocsFileName, std::ios::binary);
        if (!pdLocsFile.good())
            throw std::invalid_argument("file not found -- '" + pdLocsFileName + "'");
        PathStartEndInfo pathStartEndInfo(pdLocsFile);
        pdLocsFile.close();
        std::cout << "done.\n";


        std::cout << "Verifying input paths ..." << std::flush;
        verifyPathViability(preliminaryPaths, vehicleInputGraph, pathStartEndInfo);
        std::cout << "done.\n";

        uint64_t totalPathTravelTime = 0;
        for (const auto& p : preliminaryPaths)
            for (const auto& e : p)
                totalPathTravelTime += static_cast<uint64_t>(vehicleInputGraph.travelTime(e));
        std::cout << "Sum of travel times on all paths: " << totalPathTravelTime << "\n" << std::endl;
        auto& overviewLogger = LogManager<std::ofstream>::getLogger("overview.csv", "total_path_travel_time\n");
        overviewLogger << totalPathTravelTime << "\n";

        std::cout << "Constructing lines ..." << std::flush;

        const auto lines = decideAvoidLoopsStrategy(avoidLoopsStrategy, vehicleInputGraph, revVehicleGraph,
                                                    pathStartEndInfo, requests, preliminaryPaths);

        if (outputFullPaths) {

            std::cout << "Logging line paths..." << std::flush;
            auto &linePathLogger = LogManager<std::ofstream>::getLogger("line_paths.csv",
                                                                        "line_id,"
                                                                        "path_as_edge_ids,"
                                                                        "path_as_lat_lng\n");

            nlohmann::json topGeoJson;
            topGeoJson["type"] = "FeatureCollection";
            for (int lineId = 0; lineId < lines.size(); ++lineId) {
                const auto &line = lines[lineId];
                linePathLogger << lineId << ", ";
                // Log path as edge Ids:
                for (int i = 0; i < line.size(); ++i)
                    linePathLogger << line[i] << (i < line.size() - 1 ? " : " : ", ");

                int totalTravelTime = 0;
                // Log path as latlngs
                std::vector<LatLng> latLngPath;
                for (int i = 0; i < line.size(); ++i) {
                    const auto e = line[i];
                    const auto tail = vehicleInputGraph.edgeTail(e);
                    const auto latLng = vehicleInputGraph.latLng(tail);
                    linePathLogger << latLngForCsv(latLng) << " : ";
                    latLngPath.push_back(latLng);
                    totalTravelTime += vehicleInputGraph.travelTime(e);
                }
                if (!line.empty()) {
                    const auto latLng = vehicleInputGraph.latLng(vehicleInputGraph.edgeHead(line.back()));
                    linePathLogger << latLngForCsv(latLng);
                    latLngPath.push_back(latLng);
                }
                linePathLogger << "\n";

                // Add GeoJson features
                geojson::addGeoJsonFeaturesForLine(latLngPath, lineId, totalTravelTime, topGeoJson);
            }

            // Open the output file and write the GeoJson.
            std::ofstream geoJsonOutputFile(outputFileName + ".geojson");
            if (!geoJsonOutputFile.good())
                throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".geojson" + "'");
            geoJsonOutputFile << std::setw(2) << topGeoJson << std::endl;
            std::cout << "done.\n";
        }

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}