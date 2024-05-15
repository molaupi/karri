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


#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <iomanip>

#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"

#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/custom_assertion_levels.h"

#include "BuildTrafficFlowBasedSubgraph/KeptEdgesFinder.h"
#include "BuildTrafficFlowBasedSubgraph/KeptEdgesConnecter.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInReducedVehAttribute.h"

inline void printUsage() {
    std::cout <<
              "Usage: BuildTrafficFlowBasedSubgraph -veh-g <file> -psg-g <file> -f <file> -rho <radius> -o <file>\n"
              "This program takes input vehicle and passenger networks with a mapping between them as well as associated traffic\n"
              "flows for every arc in the vehicle network and builds a subnetwork of the vehicle network by removing as many\n"
              "arcs as possible while adhering to the following goals and constraints:\n"
              "\t - preserve arcs with greater traffic flows, remove arcs with smaller flows\n"
              "\t - for every shared arc e, preserve at least one shared arc e' s.t. e' can be reached from e within a radius\n"
              "\t   rho in the passenger network and a shared arc e'' for vice versa (shared = accessible in veh-g and psg-g)\n"
              "\t - ensure the subnetwork is connected\n"
              "Outputs the vehicle subnetwork and a passenger network (copy of original with new edge mappings).\n"
              "  -veh-g <file>          vehicle road network in binary format.\n"
              "  -psg-g <file>          passenger road (and path) network in binary format.\n"
              "  -f <file>              traffic flows in the vehicle network as single column CSV file\n"
              "  -rho <radius>          passenger moving radius (dflt: 300)\n"
              "  -o <file>              path to output files without file extension\n"
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
        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const int radius = clp.getValue<int>("rho", 300) * 10;
        const auto flowsFileName = clp.getValue<std::string>("f");
        const auto requestFileName = clp.getValue<std::string>("r");
        auto outputFileName = clp.getValue<std::string>("o");

        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>; // same for all graphs
        using VehicleInputEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, OsmRoadCategoryAttribute, MapToEdgeInPsgAttribute>;
        using VehicleInputGraph = StaticGraph<VertexAttributes, VehicleInputEdgeAttributes>;
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
                    KASSERT(vehicleInputGraph.edgeId(e) == INVALID_ID);
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        } else {
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                    KASSERT(vehicleInputGraph.edgeId(e) != INVALID_ID);
                    if (vehicleInputGraph.edgeId(e) >= vehGraphOrigIdToSeqId.size()) {
                        const auto numElementsToBeInserted =
                                vehicleInputGraph.edgeId(e) + 1 - vehGraphOrigIdToSeqId.size();
                        vehGraphOrigIdToSeqId.insert(vehGraphOrigIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                    }
                    KASSERT(vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] == INVALID_ID);
                    vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] = e;
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        }
        std::cout << "done.\n";

        // Read the passenger network from file.
        std::cout << "Reading passenger network from file... " << std::flush;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, TravelTimeAttribute, MapToEdgeInFullVehAttribute>;
        using PsgInputGraph = StaticGraph<VertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        KASSERT(psgInputGraph.numEdges() > 0 && psgInputGraph.edgeId(0) == INVALID_ID);
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
                KASSERT(psgInputGraph.edgeId(e) == INVALID_ID);
                psgInputGraph.edgeTail(e) = v;
                psgInputGraph.edgeId(e) = e;

                const int eInVehGraph = psgInputGraph.mapToEdgeInFullVeh(e);
                if (eInVehGraph != MapToEdgeInFullVehAttribute::defaultValue()) {
                    ++numEdgesWithMappingToCar;
                    KASSERT(eInVehGraph < vehGraphOrigIdToSeqId.size());
                    psgInputGraph.mapToEdgeInFullVeh(e) = vehGraphOrigIdToSeqId[eInVehGraph];
                    KASSERT(psgInputGraph.mapToEdgeInFullVeh(e) < vehicleInputGraph.numEdges());
                    vehicleInputGraph.mapToEdgeInPsg(psgInputGraph.mapToEdgeInFullVeh(e)) = e;

                    KASSERT(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                            vehicleInputGraph.latLng(
                                    vehicleInputGraph.edgeHead(psgInputGraph.mapToEdgeInFullVeh(e))).latitude());
                    KASSERT(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(
                            vehicleInputGraph.edgeHead(psgInputGraph.mapToEdgeInFullVeh(e))).longitude());
                }
            }
        unused(numEdgesWithMappingToCar);
        KASSERT(numEdgesWithMappingToCar > 0);

        const auto revPsgGraph = psgInputGraph.getReverseGraph();
        std::cout << "done.\n";

        // Read the flow data from file.
        std::cout << "Reading flow data from file... " << std::flush;
        std::vector<int> trafficFlows;
        int flow;
        io::CSVReader<1, io::trim_chars<' '>> flowsFileReader(flowsFileName);
        flowsFileReader.read_header(io::ignore_no_column, "flow");
        while (flowsFileReader.read_row(flow)) {
            if (flow < 0)
                throw std::invalid_argument("invalid flow -- '" + std::to_string(flow) + "'");
            trafficFlows.push_back(flow);
        }
        if (trafficFlows.size() != vehicleInputGraph.numEdges())
            throw std::invalid_argument("Number of given traffic flows (" + std::to_string(trafficFlows.size())
                                        + ") is not equal to number of edges in vehicle input graph ("
                                        + std::to_string(vehicleInputGraph.numEdges()) + ")!");
        std::cout << "done.\n";


        std::cout << "Number of edges in the original vehicle network: " << vehicleInputGraph.numEdges() << std::endl;
        int numPsgAccessible = 0;
        FORALL_EDGES(vehicleInputGraph, e)numPsgAccessible += vehicleInputGraph.mapToEdgeInPsg(e) !=
                                                              MapToEdgeInPsgAttribute::defaultValue();
        std::cout << "\tamong these passenger accessible: " << numPsgAccessible << std::endl;

        namespace tfs = traffic_flow_subnetwork;
        std::cout << "Finding high-flow edges to cover network in meeting points... " << std::endl;
        tfs::KeptEdgesFinder<VehicleInputGraph, PsgInputGraph> keptEdgesFinder(vehicleInputGraph, psgInputGraph,
                                                                               revPsgGraph, radius);
        auto edges = keptEdgesFinder.findKeptEdges(trafficFlows);
        std::cout << "done." << std::endl;

        std::cout << "Number of edges that need to be kept: " << edges.size() << std::endl;
        int numNeededPsgAccessible = std::count_if(edges.begin(), edges.end(), [&](const int &e) {
            return vehicleInputGraph.mapToEdgeInPsg(e) != MapToEdgeInPsgAttribute::defaultValue();
        });
        std::cout << "\tamong these passenger accessible: " << numNeededPsgAccessible << std::endl;

        tfs::KeptEdgesConnector<VertexAttributes, VehicleInputEdgeAttributes, TravelTimeAttribute> keptEdgesConnector(
                vehicleInputGraph);
        std::vector<int> subGraphToFullVertexIds;
        auto subGraph = keptEdgesConnector.connectEdges(edges, subGraphToFullVertexIds);
        LIGHT_KASSERT(subGraph.isDefrag());
        LIGHT_KASSERT(subGraphToFullVertexIds.size() == subGraph.numVertices());
        std::cout << "Generated connected sub-network: " << std::endl;
        std::cout << "\t|V| = " << subGraph.numVertices() << std::endl;
        std::cout << "\t|E| = " << subGraph.numEdges() << std::endl;


        std::cout << "Compute new mappings between full and reduced vehicle networks as well as passenger network..."
                  << std::flush;
        using OutputFullVehEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, OsmRoadCategoryAttribute, MapToEdgeInPsgAttribute, MapToEdgeInReducedVehAttribute>;
        using FullVehOutputGraph = StaticGraph<VertexAttributes, OutputFullVehEdgeAttributes>;
        FullVehOutputGraph fullVehOutputGraph(std::move(vehicleInputGraph));

        using OutputReducedVehEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, OsmRoadCategoryAttribute, MapToEdgeInPsgAttribute, MapToEdgeInFullVehAttribute>;
        using ReducedVehOutputGraph = StaticGraph<VertexAttributes, OutputReducedVehEdgeAttributes>;
        ReducedVehOutputGraph reducedVehOutputGraph(std::move(subGraph));

        using OutputPsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, TravelTimeAttribute, MapToEdgeInFullVehAttribute, MapToEdgeInReducedVehAttribute>;
        using PsgOutputGraph = StaticGraph<VertexAttributes, OutputPsgEdgeAttributes>;
        PsgOutputGraph psgOutputGraph(std::move(psgInputGraph));

        FORALL_VALID_EDGES(reducedVehOutputGraph, uRed, eRed) {
                const int &vRed = reducedVehOutputGraph.edgeHead(eRed);
                const int &uFull = subGraphToFullVertexIds[uRed];
                const int &vFull = subGraphToFullVertexIds[vRed];
                int eFull = INVALID_EDGE;
                FORALL_INCIDENT_EDGES(fullVehOutputGraph, uFull, e) {
                    if (fullVehOutputGraph.edgeHead(e) == vFull) {
                        eFull = e;
                        break;
                    }
                }
                LIGHT_KASSERT(eFull != INVALID_EDGE, "uRed = " << uRed << ", vRed = " << vRed << ", eRed = " << eRed << ", uFull = " << uFull << ", vFull = " << vFull);
                reducedVehOutputGraph.mapToEdgeInFullVeh(eRed) = eFull;
                fullVehOutputGraph.mapToEdgeInReducedVeh(eFull) = eRed;

                const int ePsg = fullVehOutputGraph.mapToEdgeInPsg(eFull);
                if (ePsg != MapToEdgeInPsgAttribute::defaultValue()) {
                    reducedVehOutputGraph.mapToEdgeInPsg(eRed) = ePsg;
                    psgOutputGraph.mapToEdgeInReducedVeh(ePsg) = eRed;
                }
            }

        // Validate mappings:
        FORALL_EDGES(fullVehOutputGraph, eFull) {
            const int eRed = fullVehOutputGraph.mapToEdgeInReducedVeh(eFull);
            if (eRed != MapToEdgeInReducedVehAttribute::defaultValue())
                LIGHT_KASSERT(reducedVehOutputGraph.mapToEdgeInFullVeh(eRed) == eFull);

            const int ePsg = fullVehOutputGraph.mapToEdgeInPsg(eFull);
            if (ePsg != MapToEdgeInPsgAttribute::defaultValue())
                LIGHT_KASSERT(psgOutputGraph.mapToEdgeInFullVeh(ePsg) == eFull);

            if (eRed != MapToEdgeInReducedVehAttribute::defaultValue() && ePsg != MapToEdgeInPsgAttribute::defaultValue()) {
                LIGHT_KASSERT(psgOutputGraph.mapToEdgeInReducedVeh(ePsg) == eRed);
                LIGHT_KASSERT(reducedVehOutputGraph.mapToEdgeInPsg(eRed) == ePsg);
            }
        }

        std::cout << " done." << std::endl;

        std::cout << "Write the vehicle subnetwork output file..." << std::flush;
        const auto reducedVehOutputFileName = outputFileName + ".veh_subnetwork.gr.bin";
        std::ofstream reducedVehOut(reducedVehOutputFileName, std::ios::binary);
        if (!reducedVehOut.good())
            throw std::invalid_argument("file cannot be opened -- '" + reducedVehOutputFileName + ".gr.bin'");
        reducedVehOutputGraph.writeTo(reducedVehOut, {"edge_id", "edge_tail"});
        std::cout << " done." << std::endl;

        std::cout << "Write the full vehicle network with new mappings to output file..." << std::flush;
        const auto fullVehOutputFileName = outputFileName + ".veh_full.gr.bin";
        std::ofstream fullVehOut(fullVehOutputFileName, std::ios::binary);
        if (!fullVehOut.good())
            throw std::invalid_argument("file cannot be opened -- '" + fullVehOutputFileName + ".gr.bin'");
        fullVehOutputGraph.writeTo(fullVehOut, {"edge_id", "edge_tail"});
        std::cout << " done." << std::endl;

        std::cout << "Write the passenger network with new mappings to output file..." << std::flush;
        const auto psgOutputFileName = outputFileName + ".psg_new_mappings.gr.bin";
        std::ofstream psgOut(psgOutputFileName, std::ios::binary);
        if (!psgOut.good())
            throw std::invalid_argument("file cannot be opened -- '" + psgOutputFileName + ".gr.bin'");
        psgOutputGraph.writeTo(psgOut, {"edge_id", "edge_tail"});
        std::cout << " done." << std::endl;


    } catch (std::invalid_argument &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
