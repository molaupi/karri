/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler
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


#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <csv.h>

#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/PathStartEndInfo.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/Request.h"
#include "Algorithms/FindMixedFixedFlexibleNetwork/PickupDropoffManager.h"

inline void printUsage() {
    std::cout <<
              "Usage: ComputePDLocsForRequests -veh-g <file> -psg-g <file> -d <file> -o <file>\n\n"

              "Given a set of requests (origin and destination edges accessible to vehicles and passengers) and a road\n"
              "network, computes possible pickup and dropoff locations within a given radius and stores them.\n"
              "Writes set of requests that can be picked up/dropped off at each vertex.\n\n"
              "  -veh-g <file>     vehicle input graph in binary format\n"
              "  -psg-g <file>     passenger input graph binary format\n"
              "  -r <file>         requests file\n"
              "  -rho <radius>     walking radius\n"
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

        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto requestsFileName = clp.getValue<std::string>("r");
        const auto walkingRadius = clp.getValue<int>("rho", 300) * 10;
        auto outputFileName = clp.getValue<std::string>("o");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, MapToEdgeInPsgAttribute>;
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
        using PsgVertexAttributes = VertexAttrs<LatLngAttribute>;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, TravelTimeAttribute, MapToEdgeInFullVehAttribute>;
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
        std::vector<mixfix::Request> requests;
        int origin, destination, requestTime;
        io::CSVReader<3, io::trim_chars<' '>> reqFileReader(requestsFileName);
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
        unused(mixfix::getMaxTravelTime);
        std::cout << "done.\n";

        std::cout << "Finding PD-Locs and writing them to file..." << std::flush;
        mixfix::PathStartEndInfo pdInfo(vehicleInputGraph.numVertices());
        using PickupDropoffManagerImpl = mixfix::PickupDropoffManager<VehicleInputGraph, PsgInputGraph>;
        PickupDropoffManagerImpl pdManager(vehicleInputGraph, psgInputGraph, revPsgGraph, walkingRadius);
        pdManager.findPossiblePDLocsForRequests(requests, pdInfo);
        if (!endsWith(outputFileName, ".pdlocs.bin"))
            outputFileName += ".pdlocs.bin";
        std::ofstream pdLocsOutputFile(outputFileName);
        if (!pdLocsOutputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".pdlocs.bin" + "'");
        pdInfo.writeTo(pdLocsOutputFile);
        pdLocsOutputFile.close();
        std::cout << "done.\n";

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
