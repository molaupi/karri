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

#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Export/OnlyAttributesAsBinaryFilesExporter.h"
#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "DataStructures/Graph/Attributes/CoordinateAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInReducedVehAttribute.h"
#include "DataStructures/Graph/Attributes/NumLanesAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/SequentialVertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/SpeedLimitAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/TraversalCostAttribute.h"
#include "DataStructures/Graph/Attributes/UnpackingInfoAttribute.h"
#include "DataStructures/Graph/Attributes/VertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/XatfRoadCategoryAttribute.h"


inline void printUsage() {
    std::cout <<
              "Usage: ExtractAttributesIntoSeparateFiles -g <file> -a <file> -o <file>\n"
              "Takes road network in binary format and extracts attributes into separate files.\n"
              "Outputs extracted attributes as new files.\n"
              "  -g <file>         input graph in binary format\n"
              "  -a <attributes>   names of attributes to extract\n"
              "  -o <path>         output base path\n"
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
        const auto attributes = clp.getValues<std::string>("a");
        auto outputBaseName = clp.getValue<std::string>("o");
        if (endsWith(outputBaseName, ".csv"))
            outputBaseName.substr(0, outputBaseName.size() - 4);

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VertexAttributes = VertexAttrs<CoordinateAttribute, LatLngAttribute, OsmNodeIdAttribute, SequentialVertexIdAttribute, VertexIdAttribute>;
        using InputGraph = StaticGraph<VertexAttributes,
                EdgeAttrs<CapacityAttribute, EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, LengthAttribute, MapToEdgeInFullVehAttribute,
                        MapToEdgeInPsgAttribute, MapToEdgeInReducedVehAttribute, NumLanesAttribute, OsmRoadCategoryAttribute, RoadGeometryAttribute, SpeedLimitAttribute,
                        TravelTimeAttribute, TraversalCostAttribute, UnpackingInfoAttribute, XatfRoadCategoryAttribute>>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        std::cout << "done.\n";

        std::cout << "Writing attributes to separate files..." << std::flush;
        // Graph exporter that writes separate binary files for attributes.
        OnlyAttributesAsBinaryFilesExporter exporter;
        for (const auto& attrName : InputGraph::getAttributeNames()) {
            if (!contains(attributes.begin(), attributes.end(), attrName))
                exporter.ignoreAttribute(attrName);
        }
        inputGraph.exportTo(outputBaseName, exporter);
        std::cout << "done.\n";


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}