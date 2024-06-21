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
#include "Algorithms/KaRRi/BaseObjects/Request.h"
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
              "Checks, which vertex and edge attributes are present in a graph file in binary format.\n"
              "  -g <file>         graph file in binary format\n";
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

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using EdgeAttributes = EdgeAttrs<
                CapacityAttribute,
                EdgeIdAttribute,
                EdgeTailAttribute,
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
        using InputGraph = StaticGraph<VertexAttributes, EdgeAttributes>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        FORALL_EDGES(inputGraph, e) {
            inputGraph.edgeId(e) = e;
        }
        std::cout << "done.\n";

        std::cout << "Graph has " << inputGraph.numVertices() << " vertices and " << inputGraph.numEdges() << " edges."
                  << std::endl;


        if (inputGraph.get<CapacityAttribute>(0) != CapacityAttribute::defaultValue())
            std::cout << "Has CapacityAttribute." << std::endl;
        if (inputGraph.get<EdgeIdAttribute>(0) != EdgeIdAttribute::defaultValue())
            std::cout << "Has EdgeIdAttribute." << std::endl;
        if (inputGraph.get<EdgeTailAttribute>(0) != EdgeTailAttribute::defaultValue())
            std::cout << "Has EdgeTailAttribute." << std::endl;
        if (inputGraph.get<FreeFlowSpeedAttribute>(0) != FreeFlowSpeedAttribute::defaultValue())
            std::cout << "Has FreeFlowSpeedAttribute." << std::endl;
        if (inputGraph.get<LengthAttribute>(0) != LengthAttribute::defaultValue())
            std::cout << "Has LengthAttribute." << std::endl;
        if (inputGraph.get<MapToEdgeInFullVehAttribute>(0) != MapToEdgeInFullVehAttribute::defaultValue())
            std::cout << "Has MapToEdgeInFullVehAttribute." << std::endl;
        if (inputGraph.get<MapToEdgeInPsgAttribute>(0) != MapToEdgeInPsgAttribute::defaultValue())
            std::cout << "Has MapToEdgeInPsgAttribute." << std::endl;
        if (inputGraph.get<MapToEdgeInReducedVehAttribute>(0) != MapToEdgeInReducedVehAttribute::defaultValue())
            std::cout << "Has MapToEdgeInReducedVehAttribute." << std::endl;
        if (inputGraph.get<NumLanesAttribute>(0) != NumLanesAttribute::defaultValue())
            std::cout << "Has NumLanesAttribute." << std::endl;
        if (inputGraph.get<OsmRoadCategoryAttribute>(0) != OsmRoadCategoryAttribute::defaultValue())
            std::cout << "Has OsmRoadCategoryAttribute." << std::endl;
        if (inputGraph.get<RoadGeometryAttribute>(0) != RoadGeometryAttribute::defaultValue())
            std::cout << "Has RoadGeometryAttribute." << std::endl;
        if (inputGraph.get<SpeedLimitAttribute>(0) != SpeedLimitAttribute::defaultValue())
            std::cout << "Has SpeedLimitAttribute." << std::endl;
        if (inputGraph.get<TravelTimeAttribute>(0) != TravelTimeAttribute::defaultValue())
            std::cout << "Has TravelTimeAttribute." << std::endl;
        if (inputGraph.get<TraversalCostAttribute>(0) != TraversalCostAttribute::defaultValue())
            std::cout << "Has TraversalCostAttribute." << std::endl;
        if (inputGraph.get<UnpackingInfoAttribute>(0) != UnpackingInfoAttribute::defaultValue())
            std::cout << "Has UnpackingInfoAttribute." << std::endl;
        if (inputGraph.get<XatfRoadCategoryAttribute>(0) != XatfRoadCategoryAttribute::defaultValue())
            std::cout << "Has XatfRoadCategoryAttribute." << std::endl;


        if (inputGraph.get<CoordinateAttribute>(0) != CoordinateAttribute::defaultValue())
            std::cout << "Has CoordinateAttribute." << std::endl;
        if (inputGraph.get<LatLngAttribute>(0) != LatLngAttribute::defaultValue())
            std::cout << "Has LatLngAttribute." << std::endl;
        if (inputGraph.get<OsmNodeIdAttribute>(0) != OsmNodeIdAttribute::defaultValue())
            std::cout << "Has OsmNodeIdAttribute." << std::endl;
        if (inputGraph.get<SequentialVertexIdAttribute>(0) != SequentialVertexIdAttribute::defaultValue())
            std::cout << "Has SequentialVertexIdAttribute." << std::endl;
        if (inputGraph.get<VertexIdAttribute>(0) != VertexIdAttribute::defaultValue())
            std::cout << "Has VertexIdAttribute." << std::endl;


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}