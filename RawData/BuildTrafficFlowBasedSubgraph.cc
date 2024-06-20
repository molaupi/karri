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

#include "BuildTrafficFlowBasedSubgraph/KeptHighFlowEdgesFinder.h"
#include "BuildTrafficFlowBasedSubgraph/KeptHighRankVerticesFinder.h"
#include "BuildTrafficFlowBasedSubgraph/SubgraphConnector.h"
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

template<typename ...VertexAttributes, typename ...EdgeAttributes>
DynamicGraph<VertexAttrs<VertexAttributes...>, EdgeAttrs<EdgeAttributes...>>
getInitialEdgeInducedSubGraph(const StaticGraph<VertexAttrs<VertexAttributes...>, EdgeAttrs<EdgeAttributes...>> &in,
                              const std::vector<int> &keptEdges,
                              std::vector<int> &inToOutVertexIds) {
    DynamicGraph<VertexAttrs<VertexAttributes...>, EdgeAttrs<EdgeAttributes...>> out(in.numVertices(), in.numEdges());

    // Add vertices and memorize mapping from old to new vertex IDs.
    for (const auto &e: keptEdges) {
        const int u = in.edgeTail(e);
        LIGHT_KASSERT(u >= 0 && u < inToOutVertexIds.size());
        if (inToOutVertexIds[u] == INVALID_VERTEX) {
            inToOutVertexIds[u] = out.appendVertex();
            RUN_FORALL(out.template get<VertexAttributes>(
                    inToOutVertexIds[u]) = in.template get<VertexAttributes>(u));
        }

        const int v = in.edgeHead(e);
        LIGHT_KASSERT(v >= 0 && v < inToOutVertexIds.size());
        if (inToOutVertexIds[v] == INVALID_VERTEX) {
            inToOutVertexIds[v] = out.appendVertex();
            RUN_FORALL(out.template get<VertexAttributes>(
                    inToOutVertexIds[v]) = in.template get<VertexAttributes>(v));
        }
    }

    // Add edges. (Mapping from old to new edge IDs will be computed later since they change in eventual
    // defragmentation).
    for (const auto &e: keptEdges) {
        LIGHT_KASSERT(in.containsEdge(in.edgeTail(e), in.edgeHead(e)));
        const auto u = inToOutVertexIds[in.edgeTail(e)];
        const auto v = inToOutVertexIds[in.edgeHead(e)];
        LIGHT_KASSERT(u != INVALID_VERTEX && v != INVALID_VERTEX);
        LIGHT_KASSERT(out.countEdgesBetween(u, v) < in.countEdgesBetween(in.edgeTail(e), in.edgeHead(e)));
        const int idx = out.insertEdge(u, v);
        RUN_FORALL(out.template get<EdgeAttributes>(idx) = in.template get<EdgeAttributes>(e));
        out.edgeTail(idx) = u;
        LIGHT_KASSERT(out.edgeId(idx) == e);
    }

    return out;
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
        const auto requestFileName = clp.getValue<std::string>("r");
        auto outputFileName = clp.getValue<std::string>("o");

        if ((clp.isSet("f") && clp.isSet("ch")) || (!clp.isSet("f") && !clp.isSet("ch"))) {
            std::cerr << "Exactly one of -f or -ch must be set." << std::endl;
            return EXIT_FAILURE;
        }

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
        FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                LIGHT_KASSERT(vehicleInputGraph.edgeId(e) == INVALID_ID);
                vehicleInputGraph.edgeTail(e) = v;
                vehicleInputGraph.edgeId(e) = e;
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
        LIGHT_KASSERT(psgInputGraph.numEdges() > 0 && psgInputGraph.edgeId(0) == INVALID_ID);
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
                LIGHT_KASSERT(psgInputGraph.edgeId(e) == INVALID_ID);
                psgInputGraph.edgeTail(e) = v;
                psgInputGraph.edgeId(e) = e;

                const int eInVehGraph = psgInputGraph.mapToEdgeInFullVeh(e);
                if (eInVehGraph != MapToEdgeInFullVehAttribute::defaultValue()) {
                    ++numEdgesWithMappingToCar;
                    LIGHT_KASSERT(eInVehGraph < vehicleInputGraph.numEdges());
                    psgInputGraph.mapToEdgeInFullVeh(e) = eInVehGraph;
                    LIGHT_KASSERT(psgInputGraph.mapToEdgeInFullVeh(e) < vehicleInputGraph.numEdges());
                    vehicleInputGraph.mapToEdgeInPsg(eInVehGraph) = e;

                    LIGHT_KASSERT(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                                  vehicleInputGraph.latLng(
                                          vehicleInputGraph.edgeHead(psgInputGraph.mapToEdgeInFullVeh(e))).latitude());
                    LIGHT_KASSERT(
                            psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(
                                    vehicleInputGraph.edgeHead(psgInputGraph.mapToEdgeInFullVeh(e))).longitude());
                }
            }
        unused(numEdgesWithMappingToCar);
        KASSERT(numEdgesWithMappingToCar > 0);

        const auto revPsgGraph = psgInputGraph.getReverseGraph();
        std::cout << "done.\n";


        std::cout << "Number of vertices in the original passenger network: " << psgInputGraph.numVertices()
                  << std::endl;
        std::cout << "Number of edges in the original vehicle network: " << vehicleInputGraph.numEdges()
                  << std::endl;
        int numPsgAccessible = 0;
        FORALL_EDGES(vehicleInputGraph, e)numPsgAccessible += vehicleInputGraph.mapToEdgeInPsg(e) !=
                                                              MapToEdgeInPsgAttribute::defaultValue();
        std::cout << "\tamong these passenger accessible: " << numPsgAccessible << std::endl;


        namespace tfs = traffic_flow_subnetwork;

        using SubGraph = DynamicGraph<VertexAttributes, VehicleInputEdgeAttributes>;
        std::unique_ptr<SubGraph> subGraphPtr;
        std::vector<int> fullToSubgraphVertexIds(vehicleInputGraph.numVertices(), INVALID_VERTEX);
        if (clp.isSet("f")) {

            const auto flowsFileName = clp.getValue<std::string>("f");

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

            std::cout << "Finding high-flow edges to cover network in meeting points... " << std::endl;
            tfs::KeptHighFlowEdgesFinder<VehicleInputGraph, PsgInputGraph>
                    keptEdgesFinder(vehicleInputGraph, psgInputGraph, revPsgGraph, radius);
            auto edges = keptEdgesFinder.findKeptEdges(trafficFlows);
            std::cout << "done." << std::endl;

            std::cout << "Number of edges that need to be kept: " << edges.size() << std::endl;
            int numNeededPsgAccessible = std::count_if(edges.begin(), edges.end(), [&](const int &e) {
                return vehicleInputGraph.mapToEdgeInPsg(e) != MapToEdgeInPsgAttribute::defaultValue();
            });
            std::cout << "\tamong these passenger accessible: " << numNeededPsgAccessible << std::endl;

            std::cout << "Constructing initial sub-network... " << std::flush;
            // Get edge-induced subgraph
            subGraphPtr = std::make_unique<SubGraph>(
                    getInitialEdgeInducedSubGraph(vehicleInputGraph, edges, fullToSubgraphVertexIds));
            KASSERT(subGraphPtr->validate());
            std::cout << " done." << std::endl;
        } else if (clp.isSet("ch")) {

            std::cout << "Finding high-rank vertices to cover network in meeting points... " << std::endl;
            tfs::KeptHighRankVerticesFinder<VehicleInputGraph, PsgInputGraph>
                    keptVerticesFinder(vehicleInputGraph, psgInputGraph, revPsgGraph, radius);
            BitVector isVertexKept = keptVerticesFinder.findKeptVertices();
            std::cout << "done." << std::endl;

            std::cout << "Number of vertices that need to be kept: " << isVertexKept.cardinality() << std::endl;

            std::cout << "Constructing initial sub-network... " << std::flush;
            // Get vertex-induced subgraph
            subGraphPtr = std::make_unique<SubGraph>(vehicleInputGraph.getVertexInducedSubgraph(isVertexKept));
            KASSERT(subGraphPtr->validate());

            // Compute full-to-subgraph vertex ID mapping.
            int nextId = 0;
            for (int v = isVertexKept.firstSetBit(); v != -1; v = isVertexKept.nextSetBit(v))
                fullToSubgraphVertexIds[v] = nextId++;
            std::cout << " done." << std::endl;
        }

        // Compute out-to-in vertex ID mapping.
        auto &subGraph = *subGraphPtr;
        std::vector<int> subGraphToFullVertexIds(subGraph.numVertices(), INVALID_VERTEX);
        for (int vIn = 0; vIn < vehicleInputGraph.numVertices(); ++vIn) {
            const auto &vOut = fullToSubgraphVertexIds[vIn];
            if (vOut != INVALID_VERTEX)
                subGraphToFullVertexIds[vOut] = vIn;
        }

        tfs::SubgraphConnector<VertexAttributes, VehicleInputEdgeAttributes, TravelTimeAttribute>
                connector(vehicleInputGraph);
        connector.connect(subGraph, fullToSubgraphVertexIds, subGraphToFullVertexIds);
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
                // When loading input graph, we set edgeId(e) = e. Then, when constructing and transforming the output
                // subgraph, we maintain the edge attribute values of each edge, including edgeId. Thus,
                // psgOutputGraph.edgeId(eRed) is the ID of the according edge in the input graph.
                const int eFull = reducedVehOutputGraph.edgeId(eRed);
                LIGHT_KASSERT(eFull >= 0 && eFull < fullVehOutputGraph.numEdges());
                LIGHT_KASSERT(
                        reducedVehOutputGraph.mapToEdgeInFullVeh(eRed) == MapToEdgeInFullVehAttribute::defaultValue());
                LIGHT_KASSERT(fullVehOutputGraph.mapToEdgeInReducedVeh(eFull) ==
                              MapToEdgeInReducedVehAttribute::defaultValue());
                reducedVehOutputGraph.mapToEdgeInFullVeh(eRed) = eFull;
                fullVehOutputGraph.mapToEdgeInReducedVeh(eFull) = eRed;
            }

        FORALL_VALID_EDGES(psgOutputGraph, uPsg, ePsg) {
                // Since multiple psg edges may map to the same vehicle edge, we have to iterate through the psg edges
                // to construct the new mappings to the reduced vehicle network.
                const int eFull = psgOutputGraph.mapToEdgeInFullVeh(ePsg);
                if (eFull != MapToEdgeInFullVehAttribute::defaultValue()) {
                    LIGHT_KASSERT(eFull >= 0 && eFull < fullVehOutputGraph.numEdges());
                    const int eRed = fullVehOutputGraph.mapToEdgeInReducedVeh(eFull);
                    if (eRed != MapToEdgeInReducedVehAttribute::defaultValue()) {
                        LIGHT_KASSERT(eRed >= 0 && eRed < reducedVehOutputGraph.numEdges());
                        psgOutputGraph.mapToEdgeInReducedVeh(ePsg) = eRed;
                    }
                }
            }

        // Validate mappings:
        FORALL_EDGES(fullVehOutputGraph, eFull) {
            const int eRed = fullVehOutputGraph.mapToEdgeInReducedVeh(eFull);
            if (eRed != MapToEdgeInReducedVehAttribute::defaultValue()) {
                LIGHT_KASSERT(reducedVehOutputGraph.mapToEdgeInFullVeh(eRed) == eFull);
                LIGHT_KASSERT(
                        fullVehOutputGraph.mapToEdgeInReducedVeh(reducedVehOutputGraph.mapToEdgeInFullVeh(eRed)) ==
                        eRed);
            }

            const int ePsg = fullVehOutputGraph.mapToEdgeInPsg(eFull);
            if (ePsg != MapToEdgeInPsgAttribute::defaultValue()) {
                LIGHT_KASSERT(psgOutputGraph.mapToEdgeInFullVeh(ePsg) == eFull);
                LIGHT_KASSERT(fullVehOutputGraph.mapToEdgeInPsg(psgOutputGraph.mapToEdgeInFullVeh(ePsg)) == ePsg);
            }

            if (eRed != MapToEdgeInReducedVehAttribute::defaultValue() &&
                ePsg != MapToEdgeInPsgAttribute::defaultValue()) {
                LIGHT_KASSERT(psgOutputGraph.mapToEdgeInReducedVeh(ePsg) == eRed);
                LIGHT_KASSERT(reducedVehOutputGraph.mapToEdgeInPsg(eRed) == ePsg);
            }
        }

        FORALL_VALID_EDGES(reducedVehOutputGraph, uRed, eRed) {
                const int eFull = reducedVehOutputGraph.mapToEdgeInFullVeh(eRed);
                LIGHT_KASSERT(eFull != MapToEdgeInFullVehAttribute::defaultValue());
                LIGHT_KASSERT(fullVehOutputGraph.mapToEdgeInReducedVeh(eFull) == eRed);

                LIGHT_KASSERT(reducedVehOutputGraph.latLng(reducedVehOutputGraph.edgeHead(eRed)) ==
                              fullVehOutputGraph.latLng(fullVehOutputGraph.edgeHead(eFull)));

                const int ePsg = reducedVehOutputGraph.mapToEdgeInPsg(eRed);
                if (ePsg != MapToEdgeInPsgAttribute::defaultValue()) {
                    LIGHT_KASSERT(psgOutputGraph.mapToEdgeInReducedVeh(ePsg) == eRed);
                    LIGHT_KASSERT(psgOutputGraph.mapToEdgeInFullVeh(ePsg) == eFull);

                    LIGHT_KASSERT(reducedVehOutputGraph.latLng(reducedVehOutputGraph.edgeHead(eRed)) ==
                                  psgOutputGraph.latLng(psgOutputGraph.edgeHead(ePsg)));
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
