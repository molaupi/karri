/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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


#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Algorithms/GraphTraversal/StronglyConnectedComponents.h"
#include "DataStructures/Geometry/Area.h"
#include "DataStructures/Geometry/Point.h"
#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "DataStructures/Graph/Attributes/CoordinateAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/NumLanesAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/SequentialVertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/SpeedLimitAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/VertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/XatfRoadCategoryAttribute.h"
#include "DataStructures/Graph/Export/DefaultExporter.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Import/DimacsImporter.h"
#include "DataStructures/Graph/Import/MatSimImporter.h"
#include "DataStructures/Graph/Import/VehicleOsmImporter.h"
#include "DataStructures/Graph/Import/VisumImporter.h"
#include "DataStructures/Graph/Import/XatfImporter.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/ContainerHelpers.h"

inline void printUsage() {
  std::cout <<
      "Usage: ConvertGraph -s <fmt> -d <fmt> [-c] [-scc] -a <attrs> -i <file> -o <file>\n"
      "This program converts a graph from a source file format to a destination format,\n"
      "possibly extracting the largest strongly connected component of the input graph.\n"
      "  -s <fmt>          source file format\n"
      "                      possible values:\n"
      "                        binary default dimacs matsim osm visum xatf\n"
      "  -d <fmt>          destination file format\n"
      "                      possible values:\n"
      "                        binary default dimacs\n"
      "  -c                compress the output file(s), if available\n"
      "  -p <file>         extract a region given as an OSM POLY file\n"
      "  -scc              extract the largest strongly connected component\n"
      "  -dp <prec>        travel distances are given in 1/<prec> meters (DIMACS only)\n"
      "  -tp <prec>        travel times are given in 1/<prec> seconds (DIMACS only)\n"
      "  -cp <prec>        coordinates are given in 1/<prec> degrees (DIMACS only)\n"
      "  -ts <sys>         system whose network is to be imported (Visum/MATSim only)\n"
      "  -cs <epsg-code>   coordinate system used in files (Visum/MATSim only)\n"
      "  -cp <factor>      multiply coordinates in files by <factor> (Visum only)\n"
      "  -ap <hrs>         analysis period, capacity is in vehicles/AP (Visum only)\n"
      "  -a <attrs>        blank-separated list of vertex/edge attributes to be output\n"
      "                      possible values:\n"
      "                        capacity coordinate edge_id free_flow_speed lat_lng\n"
      "                        length num_lanes osm_node_id osm_road_category\n"
      "                        road_geometry sequential_vertex_id speed_limit\n"
      "                        travel_time vertex_id xatf_road_category\n"
      "  -i <file>         input file(s) without file extension\n"
      "  -o <file>         output file(s) without file extension\n"
      "  -help             display this help and exit\n";
}

// A graph data structure encompassing all vertex and edge attributes available for output.
using VertexAttributes = VertexAttrs<
    CoordinateAttribute, LatLngAttribute, OsmNodeIdAttribute, SequentialVertexIdAttribute, VertexIdAttribute>;
using EdgeAttributes = EdgeAttrs<
    CapacityAttribute, EdgeIdAttribute, FreeFlowSpeedAttribute, LengthAttribute,
    NumLanesAttribute, OsmRoadCategoryAttribute, RoadGeometryAttribute, SpeedLimitAttribute,
    TravelTimeAttribute, XatfRoadCategoryAttribute>;
using GraphT = StaticGraph<VertexAttributes, EdgeAttributes>;

// Imports a graph according to the input file format specified on the command line and returns it.
inline GraphT importGraph(const CommandLineParser& clp) {
  const auto format = clp.getValue<std::string>("s");
  const auto infile = clp.getValue<std::string>("i");

  // Choose the corresponding import procedure.
  if (format == "binary") {
    std::ifstream in(infile + ".gr.bin", std::ios::binary);
    if (!in.good())
      throw std::invalid_argument("file not found -- '" + infile + ".gr.bin'");
    return GraphT(in);
  } else if (format == "dimacs") {
    const auto distPrecision = clp.getValue<int>("dp", 1);
    const auto timePrecision = clp.getValue<int>("tp", 10);
    const auto coordinatePrecision = clp.getValue<int>("cp", 1000000);
    if (distPrecision <= 0) {
      const std::string what = "travel distance precision not strictly positive";
      throw std::invalid_argument(what + " -- '" + std::to_string(distPrecision) + "'");
    }
    if (timePrecision <= 0) {
      const std::string what = "travel time precision not strictly positive";
      throw std::invalid_argument(what + " -- '" + std::to_string(timePrecision) + "'");
    }
    if (coordinatePrecision <= 0) {
      const std::string what = "coordinate precision not strictly positive";
      throw std::invalid_argument(what + " -- '" + std::to_string(coordinatePrecision) + "'");
    }
    return GraphT(infile, DimacsImporter(distPrecision, timePrecision, coordinatePrecision));
  } else if (format == "matsim") {
    const auto sys = clp.getValue<std::string>("ts", "car");
    const auto crs = clp.getValue<int>("cs", 31468);
    return GraphT(infile, MatSimImporter(sys, crs));
  } else if (format == "osm") {
    return GraphT(infile, VehicleOsmImporter());
  } else if (format == "visum") {
    const auto sys = clp.getValue<std::string>("ts", "P");
    const auto crs = clp.getValue<int>("cs", 31467);
    const auto precision = clp.getValue<double>("cp", 1.0);
    const auto period = clp.getValue<int>("ap", 24);
    if (precision <= 0) {
      const auto what = "precision not strictly positive -- '" + std::to_string(precision) + "'";
      throw std::invalid_argument(what);
    }
    if (period <= 0) {
      const auto what = "analysis period not strictly positive -- '" + std::to_string(period) + "'";
      throw std::invalid_argument(what);
    }
    return GraphT(infile, VisumImporter(infile, sys, crs, precision, period));
  } else if (format == "xatf") {
    return GraphT(infile, XatfImporter());
  } else {
    throw std::invalid_argument("unrecognized input file format -- '" + format + "'");
  }
}

// Executes a graph export using the specified exporter.
template <typename ExporterT>
inline void doExport(const CommandLineParser& clp, const GraphT& graph, ExporterT ex) {
  // Output only those attributes specified on the command line.
  auto attrsToOutput = clp.getValues<std::string>("a");
  for (const auto& attr : GraphT::getAttributeNames())
    if (!contains(attrsToOutput.begin(), attrsToOutput.end(), attr))
      ex.ignoreAttribute(attr);
  graph.exportTo(clp.getValue<std::string>("o"), ex);
}

// Exports the specified graph according to the output file format specified on the command line.
inline void exportGraph(const CommandLineParser& clp, const GraphT& graph) {
  const auto format = clp.getValue<std::string>("d");
  const auto compress = clp.isSet("c");

  // Choose the corresponding export procedure.
  if (format == "binary") {
    const auto outfile = clp.getValue<std::string>("o");
    std::ofstream out(outfile + ".gr.bin", std::ios::binary);
    if (!out.good())
      throw std::invalid_argument("file cannot be opened -- '" + outfile + ".gr.bin'");
    // Output only those attributes specified on the command line.
    std::vector<std::string> attrsToIgnore;
    std::vector<std::string> attrsToOutput = clp.getValues<std::string>("a");
    for (const auto& attr : GraphT::getAttributeNames())
      if (!contains(attrsToOutput.begin(), attrsToOutput.end(), attr))
        attrsToIgnore.push_back(attr);
    graph.writeTo(out, attrsToIgnore);
  } else if (format == "default") {
    doExport(clp, graph, DefaultExporter(compress));
  } else {
    throw std::invalid_argument("unrecognized output file format -- '" + format + "'");
  }
}

int main(int argc, char* argv[]) {
  try {
    CommandLineParser clp(argc, argv);
    if (clp.isSet("help")) {
      printUsage();
      return EXIT_SUCCESS;
    }

    std::cout << "Reading the input file(s)..." << std::flush;
    auto graph = importGraph(clp);
    std::cout << " done." << std::endl;

    if (clp.isSet("p")) {
      std::cout << "Extracting the given region..." << std::flush;
      BitVector isVertexInsideRegion(graph.numVertices());
      Area area;
      area.importFromOsmPolyFile(clp.getValue<std::string>("p"));
      const auto box = area.boundingBox();
      FORALL_VERTICES(graph, v) {
        const Point p(graph.latLng(v).longitude(), graph.latLng(v).latitude());
        isVertexInsideRegion[v] = box.contains(p) && area.contains(p);
      }
      FORALL_VERTICES(graph, v)
        graph.sequentialVertexId(v) = v;
      graph.extractVertexInducedSubgraph(isVertexInsideRegion);
      std::cout << " done." << std::endl;
    }

    if (clp.isSet("scc")) {
      std::cout << "Computing strongly connected components..." << std::flush;
      StronglyConnectedComponents scc;
      scc.run(graph);
      std::cout << " done." << std::endl;

      std::cout << "Extracting the largest SCC..." << std::flush;
      graph.extractVertexInducedSubgraph(scc.getLargestSccAsBitmask());
      std::cout << " done." << std::endl;
    }

    if (clp.isSet("o")) {
      std::cout << "Writing the output file(s)..." << std::flush;
      exportGraph(clp, graph);
      std::cout << " done." << std::endl;
    }
  } catch (std::invalid_argument& e) {
    std::cerr << argv[0] << ": " << e.what() << std::endl;
    std::cerr << "Try '" << argv[0] <<" -help' for more information." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
