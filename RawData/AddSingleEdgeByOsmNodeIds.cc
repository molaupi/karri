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

#include <cstdlib>
#include <random>
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "DataStructures/Utilities/OriginDestination.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/SpeedLimitAttribute.h"
#include "DataStructures/Graph/Attributes/TraversalCostAttribute.h"
#include "DataStructures/Graph/Attributes/UnpackingInfoAttribute.h"
#include "DataStructures/Graph/Attributes/CoordinateAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/SequentialVertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/VertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInReducedVehAttribute.h"

inline void printUsage() {
    std::cout <<
              "Given a graph with stored OSM node IDs at its vertices, adds a single edge from a given OSM node ID\n"
              "to another. Travel time of the edge is computed using great-circle distance edge length and default\n"
              "speed limit for given road category.\n"
              "  -g <file>         graph file in binary format\n"
              "  -tail <ID>        OSM node ID of tail vertex\n"
              "  -head <ID>        OSM node ID of head vertex\n"
              "  -cat <road cat>   OSM road category (see OSMRoadCategoryAttribute)\n"
              "  -o <file>         path to output file\n";
}

static const std::unordered_map<OsmRoadCategory, int> defaultSpeedLimits = {
        {OsmRoadCategory::MOTORWAY,       120},
        {OsmRoadCategory::MOTORWAY_LINK,  80},
        {OsmRoadCategory::TRUNK,          80},
        {OsmRoadCategory::TRUNK_LINK,     50},
        {OsmRoadCategory::PRIMARY,        80},
        {OsmRoadCategory::PRIMARY_LINK,   60},
        {OsmRoadCategory::SECONDARY,      30},
        {OsmRoadCategory::SECONDARY_LINK, 30},
        {OsmRoadCategory::TERTIARY,       25},
        {OsmRoadCategory::TERTIARY_LINK,  25},
        {OsmRoadCategory::UNCLASSIFIED,   15},
        {OsmRoadCategory::RESIDENTIAL,    15},
        {OsmRoadCategory::LIVING_STREET,  10},
        {OsmRoadCategory::SERVICE,        10},
};

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto graphFileName = clp.getValue<std::string>("g");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using EdgeAttributes = EdgeAttrs<
                CapacityAttribute,
                FreeFlowSpeedAttribute,
                LengthAttribute,
                MapToEdgeInFullVehAttribute,
                MapToEdgeInPsgAttribute,
                MapToEdgeInReducedVehAttribute,
                NumLanesAttribute,
                OsmRoadCategoryAttribute,
                RoadGeometryAttribute,
                SpeedLimitAttribute,
                TravelTimeAttribute,
                TraversalCostAttribute,
                UnpackingInfoAttribute,
                XatfRoadCategoryAttribute
        >;
        using VertexAttributes = VertexAttrs<
                CoordinateAttribute,
                LatLngAttribute,
                OsmNodeIdAttribute,
                SequentialVertexIdAttribute,
                VertexIdAttribute
        >;
        using InputGraph = DynamicGraph<VertexAttributes, EdgeAttributes>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        std::cout << "done.\n";

        const auto tailOsmNodeId = clp.getValue<uint64_t>("tail");
        const auto headOsmNodeId = clp.getValue<uint64_t>("head");

        const auto catName = clp.getValue<std::string>("cat");
        const auto cat = EnumParser<OsmRoadCategory>()(catName);
        KASSERT(defaultIsVehicleAccessible(cat));
        const int speedLimit = defaultSpeedLimits.at(cat);

        // Find vertices:
        int head = INVALID_VERTEX;
        int tail = INVALID_VERTEX;
        FORALL_VERTICES(inputGraph, v) {
            if (inputGraph.osmNodeId(v) == tailOsmNodeId) {
                tail = v;
            }
            if (inputGraph.osmNodeId(v) == headOsmNodeId) {
                head = v;
            }
            if (tail != INVALID_VERTEX && head != INVALID_VERTEX)
                break;
        }

        if (tail == INVALID_VERTEX)
            throw std::invalid_argument("Could not find vertex with OSM node ID " + std::to_string(tailOsmNodeId));
        if (head == INVALID_VERTEX)
            throw std::invalid_argument("Could not find vertex with OSM node ID " + std::to_string(headOsmNodeId));

        // Compute great-circle distance and travel time:
        const auto gcd = inputGraph.latLng(tail).getGreatCircleDistanceTo(inputGraph.latLng(head));
        const auto travelTime = static_cast<TravelTimeAttribute::Type>(std::round(36.0 * gcd / speedLimit));

        // Insert the edge:
        const int e = inputGraph.insertEdge(tail, head);
        inputGraph.osmRoadCategory(e) = cat;
        inputGraph.speedLimit(e) = speedLimit;
        inputGraph.freeFlowSpeed(e) = speedLimit;
        inputGraph.length(e) = static_cast<LengthAttribute::Type>(std::round(gcd));
        inputGraph.travelTime(e) = travelTime;


        std::cout << "Write the passenger graph output file..." << std::flush;
        inputGraph.defrag();
        auto outfile = clp.getValue<std::string>("o");
        if (!endsWith(outfile, ".gr.bin")) outfile += ".gr.bin";
        std::ofstream out(outfile, std::ios::binary);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outfile + ".gr.bin'");
        inputGraph.writeTo(out);
        std::cout << " done." << std::endl;
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}