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
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Geometry/LatLng.h"
#include "DataStructures/Containers/BitVector.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/CoordinateAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include <fstream>
#include <random>

inline void printUsage() {
    std::cout <<
              "Usage: GenerateIdenticalPedestrianGraph -g <file> -o <file>\n"
              "Generates a copy of a given vehicle graph which is topologically identical and uses fixed travel speed\n"
              "on all edges. Travel times are computed using length attribute of input graph.\n"
              "Outputs pedestrian graph and vehicle graph with edge mapping between them.\n"
              "  -g <file>         input vehicle road network in binary format.\n"
              "  -s <float>        travel speed in pedestrian graph in km/h (default: 4.5)\n"
              "  -a <attrs>        blank-separated list of vertex/edge attributes to be output in both graphs\n"
              "                      possible values:\n"
              "                        coordinate lat_lng length osm_node_id osm_road_category road_geometry\n"
              "  -o <dir>         place output road networks in directory <dir>\n"
              "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        auto inputGraphFileName = clp.getValue<std::string>("g");
        if (!endsWith(inputGraphFileName, ".gr.bin"))
            inputGraphFileName += ".gr.bin";
        std::string base_filename = inputGraphFileName.substr(inputGraphFileName.find_last_of("/\\") + 1);
        base_filename = base_filename.substr(0, base_filename.size() - std::string(".gr.bin").size());

        auto outputDirName = clp.getValue<std::string>("o");
        if (!endsWith(outputDirName, "/"))
            outputDirName += "/";

        const auto speed = clp.getValue<double>("s", 4.5);

        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using CommonVertexAttrs = VertexAttrs<CoordinateAttribute, LatLngAttribute, OsmNodeIdAttribute>;
        using VehEdgeAttrs = EdgeAttrs<CarEdgeToPsgEdgeAttribute, LengthAttribute,
                OsmRoadCategoryAttribute, RoadGeometryAttribute, TravelTimeAttribute>;
        using PedEdgeAttrs = EdgeAttrs<LengthAttribute,
                OsmRoadCategoryAttribute, PsgEdgeToCarEdgeAttribute, RoadGeometryAttribute, TravelTimeAttribute>;
        using VehGraph = StaticGraph<CommonVertexAttrs, VehEdgeAttrs>;
        using PedGraph = StaticGraph<CommonVertexAttrs, PedEdgeAttrs>;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        VehGraph vehGraph(inputGraphFile);
        inputGraphFile.close();
        std::cout << "done." << std::endl;

        std::cout << "Writing new travel time attribute... " << std::flush;
        PedGraph pedGraph(vehGraph);
        FORALL_VALID_EDGES(pedGraph, u, e) {
                LIGHT_KASSERT(pedGraph.length(u) != LengthAttribute::defaultValue());
                pedGraph.travelTime(e) = static_cast<TravelTimeAttribute::Type>(
                        std::round(36.0 * static_cast<double>(pedGraph.length(e)) / speed));
                pedGraph.toCarEdge(e) = e;
            }
        FORALL_VALID_EDGES(vehGraph, u, e) {
            vehGraph.toPsgEdge(e) = e;
        }
        std::cout << "done." << std::endl;

        std::cout << "Writing output graphs... " << std::flush;

        std::vector<std::string> attrsToOutput = clp.getValues<std::string>("a");
        attrsToOutput.emplace_back("travel_time");
        attrsToOutput.emplace_back("car_edge_to_psg_edge");
        attrsToOutput.emplace_back("psg_edge_to_car_edge");

        std::ofstream vehOut(outputDirName + base_filename + ".gr.bin", std::ios::binary);
        if (!vehOut.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputDirName + base_filename + ".gr.bin");
        // Output only those attributes specified on the command line.
        std::vector<std::string> attrsToIgnore;
        for (const auto &attr: VehGraph::getAttributeNames())
            if (!contains(attrsToOutput.begin(), attrsToOutput.end(), attr))
                attrsToIgnore.push_back(attr);
        vehGraph.writeTo(vehOut, attrsToIgnore);

        std::ofstream pedOut(outputDirName + base_filename + "_ped.gr.bin", std::ios::binary);
        if (!pedOut.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputDirName + base_filename + "_ped.gr.bin");
        // Output only those attributes specified on the command line.
        attrsToIgnore.clear();
        for (const auto &attr: PedGraph::getAttributeNames())
            if (!contains(attrsToOutput.begin(), attrsToOutput.end(), attr))
                attrsToIgnore.push_back(attr);
        pedGraph.writeTo(pedOut, attrsToIgnore);

        std::cout << " done." << std::endl;
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}