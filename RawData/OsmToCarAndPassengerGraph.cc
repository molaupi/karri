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
#include <utility>
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Import/PedestrianOsmImporter.h"
#include "DataStructures/Graph/Import/CyclistOsmImporter.h"
#include "DataStructures/Graph/Import/VehicleOsmImporter.h"
#include "DataStructures/Graph/Attributes/CoordinateAttribute.h"
#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/SequentialVertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/VertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/NumLanesAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/SpeedLimitAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/XatfRoadCategoryAttribute.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Algorithms/GraphTraversal/StronglyConnectedComponents.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"

// Graph converter specifically for generating two matching graphs, one for cars and one for pedestrians or cyclists.

inline void printUsage() {
    std::cout <<
              "Usage: OsmToCarAndPassengerGraph -a <attrs> -i <file> -co <file> -po <file>\n"
              "This program reads a graph from an OSM PBF source file and converts it into two binary graph files: a graph\n"
              "representing a network for cars and a graph representing a network for passengers moving on their own. For any edge e in the\n "
              "passenger graph that also occurs in the car graph, the passenger graph contains an attribute \n"
              "PsgEdgeToCarEdgeAttribute that maps e to the according edge e' in the car graph. Both graphs only\n"
              "represent the largest SCC of the input graph.\n"
              "  -a <attrs>             blank-separated list of vertex/edge attributes to be output in the car graph\n"
              "                           possible values:\n"
              "                             capacity coordinate edge_id free_flow_speed lat_lng\n"
              "                             length num_lanes osm_road_category\n"
              "                             road_geometry sequential_vertex_id speed_limit\n"
              "                             travel_time vertex_id xatf_road_category\n"
              "  -psg-mode <mode>       mode of transportation of the passenger\n"
              "                             possible values: pedestrian (default), cyclist\n"
              "  -no-union-nodes        If set, nodes are intersections in each individual network. Less precise mapping but smaller networks.\n"
              "  -no-veh-on-service     Disallow vehicles on roads of OSM 'SERVICE' road category.\n"
              "  -i <file>              input graph in OSM PBF format without file extension\n"
              "  -co <file>             car graph output file without file extension\n"
              "  -po <file>             passenger graph output file without file extension\n"
              "  -help                  display this help and exit\n";
}

// A graph data structure encompassing all vertex and edge attributes available for output.
using CarVertexAttributes = VertexAttrs<
        CoordinateAttribute, LatLngAttribute, SequentialVertexIdAttribute, VertexIdAttribute, OsmNodeIdAttribute
>;
using CarEdgeAttributes = EdgeAttrs<
        CapacityAttribute,
        EdgeIdAttribute,
        FreeFlowSpeedAttribute,
        LengthAttribute,
        NumLanesAttribute,
        OsmRoadCategoryAttribute,
        RoadGeometryAttribute,
        SpeedLimitAttribute,
        TravelTimeAttribute,
        XatfRoadCategoryAttribute,
        CarEdgeToPsgEdgeAttribute
>;
using CarGraphT = StaticGraph<CarVertexAttributes, CarEdgeAttributes>;

using PsgVertexAttributes = VertexAttrs<
        CoordinateAttribute,
        LatLngAttribute,
        SequentialVertexIdAttribute,
        VertexIdAttribute,
        OsmNodeIdAttribute
>;
using PsgEdgeAttributes = EdgeAttrs<
        PsgEdgeToCarEdgeAttribute,
        TravelTimeAttribute,
        OsmRoadCategoryAttribute
>;
using PsgGraphT = StaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;


template<typename PsgOsmImporterT>
void generateGraphs(const CommandLineParser &clp, const IsRoadAccessibleByCategory &isPsgAccessible) {
    unused(isPsgAccessible);

    const auto infile = clp.getValue<std::string>("i");

    const bool noVehiclesOnServiceRoads = clp.isSet("no-veh-on-service");
    const auto isVehicleAccessible = [noVehiclesOnServiceRoads](const OsmRoadCategory& cat) -> bool {
        return !(noVehiclesOnServiceRoads && cat == OsmRoadCategory::SERVICE) && defaultIsVehicleAccessible(cat);
    };

    const bool noUnionNodes = clp.isSet("no-union-nodes");
    std::function<bool(uint64_t, const RoutingKit::TagMap &)> isRoutingNodeInUnionOfNetworks = [](uint64_t,
                                                                                                  const RoutingKit::TagMap &) { return false; };
    std::unique_ptr<RoutingKit::BitVector> is_routing_node;
    if (!noUnionNodes) {

        std::cout << "Finding routing nodes in union of networks..." << std::endl;
        // If we want to map between a vehicle network and a passenger network, we want both networks to use the same
        // set of routing nodes, to allow 1 to 1 mapping of vertices and edges in the networks.
        // If we do not specify this set of routing nodes, each network will be generated with its own set of routing
        // nodes where each OSM node becomes a routing node only if it is an intersection, i.e. a part of at least
        // two ways, in _that_ network. In particular, an intersection in one network may not be an intersection in
        // the other or an interface between the networks may be missed in both networks.
        //
        // Therefore, we first determine all routing nodes (intersections) in the union of both networks (considering
        // all ways that are open for vehicles and/or passengers), and then force each routing node in the union to
        // also become a routing node in both individual networks.


        // Returns true if the specified way is open for vehicles or passengers (needed for mapping between
        // vehicle and passenger network).
        EnumParser<OsmRoadCategory> parseOsmRoadCategory;
        auto isWayOpenForVehiclesOrPassengers = [&](uint64_t, const RoutingKit::TagMap &tags) {
            const auto highway = tags["highway"];
            if (highway)
                try {
                    const auto cat = parseOsmRoadCategory(highway);
                    return isVehicleAccessible(cat) || isPsgAccessible(cat);
                } catch (std::invalid_argument & /*e*/) {}
            return false;
        };

        const std::string fileNameWithExtension = infile + (endsWith(infile, ".osm.pbf") ? "" : ".osm.pbf");
        std::unique_ptr<RoutingKit::OSMRoutingIDMapping> idMapping = std::make_unique<RoutingKit::OSMRoutingIDMapping>(
                RoutingKit::load_osm_id_mapping_from_pbf(
                        fileNameWithExtension, nullptr, isWayOpenForVehiclesOrPassengers));
        is_routing_node = std::make_unique<RoutingKit::BitVector>(std::move(idMapping->is_routing_node));
        idMapping.reset();

        isRoutingNodeInUnionOfNetworks = [&is_routing_node](uint64_t osm_node_id, const RoutingKit::TagMap &) {
            if (osm_node_id >= is_routing_node->size())
                throw std::invalid_argument(
                        "OSM node id " + std::to_string(osm_node_id) + " is larger than the largest ID " +
                        std::to_string(is_routing_node->size() - 1));
            return is_routing_node->is_set(osm_node_id);
        };

        std::cout << "done.\n";
    }

    std::cout << "Constructing passenger graph...\n" << std::flush;

    std::cout << "\tReading the input file..." << std::flush;
    auto psgImporter = std::make_unique<PsgOsmImporterT>(isPsgAccessible, isRoutingNodeInUnionOfNetworks);
    auto psgGraph = PsgGraphT::makeGraphUsingImporterRef(infile, *psgImporter);
    std::cout << "\t done." << std::endl;

    std::cout << "\tComputing strongly connected components..." << std::flush;
    StronglyConnectedComponents psgScc;
    psgScc.run(psgGraph);
    std::cout << "\t done." << std::endl;

    std::cout << "\tExtracting the largest SCC..." << std::flush;
    auto psgGraphToSccEdgeMap = std::make_unique<std::vector<int>>(psgGraph.numEdges(), INVALID_ID);
    psgGraph.extractVertexInducedSubgraph(psgScc.getLargestSccAsBitmask(), *psgGraphToSccEdgeMap);
    std::cout << "\t done." << std::endl;

    std::cout << "\tStoring relevant OSM mapping..." << std::flush;


    std::vector<std::pair<uint64_t, uint64_t>> osmWayIdAndHeadVertexId(psgGraph.numEdges());
    for (int e = 0; e < psgGraphToSccEdgeMap->size(); ++e) {
        const auto eInOrigGraph = e;
        const auto eInScc = psgGraphToSccEdgeMap->at(e);

        if (eInScc == INVALID_ID) continue;
        assert(eInScc >= 0 && eInScc < psgGraph.numEdges());

        osmWayIdAndHeadVertexId[eInScc] = psgImporter->getOsmIdsForEdgeAndHead(eInOrigGraph);
        assert(psgGraph.osmNodeId(psgGraph.edgeHead(eInScc)) == osmWayIdAndHeadVertexId[eInScc].second);
    }
    psgGraphToSccEdgeMap.reset();

    psgImporter->close();
    psgImporter.reset();
    std::cout << "\t done." << std::endl;

    std::cout << " done." << std::endl;


    std::cout << "Constructing car graph...\n" << std::flush;

    std::cout << "\tRead input file..." << std::flush;
    auto carImporter = VehicleOsmImporter(true, isVehicleAccessible, isRoutingNodeInUnionOfNetworks);
    auto carGraph = CarGraphT::makeGraphUsingImporterRef(infile, carImporter);
    std::cout << "\t done." << std::endl;

    std::cout << "\tCompute SCCs..." << std::flush;
    StronglyConnectedComponents carScc;
    carScc.run(carGraph);
    std::cout << "\t done." << std::endl;

    std::cout << "\tExtract largest SCC..." << std::flush;
    auto carGraphToSccEdgeMap = std::vector<int>(carGraph.numEdges(), INVALID_ID);
    carGraph.extractVertexInducedSubgraph(carScc.getLargestSccAsBitmask(), carGraphToSccEdgeMap);
    std::cout << "\t done." << std::endl;

    std::cout << " done." << std::endl;

    is_routing_node.reset();

    std::cout << "Map shared edges..." << std::flush;
    uint64_t numEdgesInPsgWithValidMapping = 0;
    FORALL_VALID_EDGES(psgGraph, v, e) {
            const auto cat = psgGraph.osmRoadCategory(e);
            assert(isPsgAccessible(cat));
            if (isVehicleAccessible(cat)) {
                const auto [osmWayId, osmHeadNodeId] = osmWayIdAndHeadVertexId[e];
                const auto eInFullCarGraph = carImporter.getEdgeForOSMIdOfWayAndHead(osmWayId, osmHeadNodeId);

                if (eInFullCarGraph == PsgEdgeToCarEdgeAttribute::defaultValue())
                    continue;

                assert(eInFullCarGraph < int(carGraphToSccEdgeMap.size()));
                const auto eInCarSCC = carGraphToSccEdgeMap[eInFullCarGraph];

                if (eInCarSCC == INVALID_ID)
                    continue;


                assert(carGraph.latLng(carGraph.edgeHead(eInCarSCC)) == psgGraph.latLng(psgGraph.edgeHead(e)));
                assert(carGraph.osmNodeId(carGraph.edgeHead(eInCarSCC)) == osmHeadNodeId);
                if (carGraph.osmNodeId(carGraph.edgeHead(eInCarSCC)) != osmHeadNodeId) {
                    auto carGraphHeadOsmNodeId = carGraph.osmNodeId(carGraph.edgeHead(eInCarSCC));
                    throw std::invalid_argument("For way with id " + std::to_string(osmWayId) +
                                                " the car graph has head at osm node id " +
                                                std::to_string(carGraphHeadOsmNodeId) +
                                                " and the psg graph has head at osm node id " +
                                                std::to_string(osmHeadNodeId));
                }


                const auto &psgLatLng = psgGraph.latLng(psgGraph.edgeHead(e));
                const auto &carLatLng = carGraph.latLng(carGraph.edgeHead(eInCarSCC));
                if (psgLatLng.latitude() != carLatLng.latitude() || psgLatLng.longitude() != carLatLng.longitude())
                    throw std::invalid_argument(
                            "Edge heads of edges in passenger and vehicle network are at different coordinates. Coordinates in psg network = " +
                            latLngForCsv(psgLatLng) + ", Coordinates in veh network = " +
                            latLngForCsv(carLatLng));


                psgGraph.toCarEdge(e) = eInCarSCC;
                carGraph.toPsgEdge(eInCarSCC) = e;
                if (psgGraph.toCarEdge(e) != PsgEdgeToCarEdgeAttribute::defaultValue())
                    ++numEdgesInPsgWithValidMapping;
                assert(psgGraph.toCarEdge(e) < carGraph.numEdges());
            }

        }

    FORALL_VALID_EDGES(carGraph, v, e) {
            assert(carGraph.toPsgEdge(e) == CarEdgeToPsgEdgeAttribute::defaultValue() ||
                   carGraph.toPsgEdge(e) < psgGraph.numEdges());
        }


    std::cout << " done." << std::endl;

    if (clp.isSet("co")) {
        std::cout << "Write the car graph output file..." << std::flush;
        auto outfile = clp.getValue<std::string>("co");
        if (!endsWith(outfile, ".gr.bin")) outfile += ".gr.bin";
        std::ofstream out(outfile, std::ios::binary);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outfile + ".gr.bin'");
        // Output only those attributes specified on the command line.
        std::vector<std::string> attrsToIgnore;
        std::vector<std::string> attrsToOutput = clp.getValues<std::string>("a");
        attrsToOutput.emplace_back("car_edge_to_psg_edge"); // Always output edge mapping to passenger graph
        for (const auto &attr: CarGraphT::getAttributeNames())
            if (!contains(attrsToOutput.begin(), attrsToOutput.end(), attr))
                attrsToIgnore.push_back(attr);
        carGraph.writeTo(out, attrsToIgnore);
        std::cout << " done." << std::endl;
    }

    if (clp.isSet("po")) {
        std::cout << "Write the passenger graph output file..." << std::flush;
        auto outfile = clp.getValue<std::string>("po");
        if (!endsWith(outfile, ".gr.bin")) outfile += ".gr.bin";
        std::ofstream out(outfile, std::ios::binary);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outfile + ".gr.bin'");
        psgGraph.writeTo(out);
        std::cout << " done." << std::endl;
    }
}


int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        const auto psgModeStr = clp.getValue<std::string>("psg-mode");
        if (psgModeStr == "pedestrian") {
            generateGraphs<PedestrianOsmImporter>(clp, defaultIsPedestrianAccessible);
        } else if (psgModeStr == "cyclist") {
            generateGraphs<CyclistOsmImporter>(clp, defaultIsCyclistAccessible);
        } else {
            throw std::invalid_argument("passenger mode of transport not known -- " + psgModeStr);
        }
    } catch (std::invalid_argument &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
