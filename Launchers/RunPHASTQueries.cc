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


#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <csv.h>
#include "Algorithms/CH/CH.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/PHAST/RPHASTEnvironment.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/StringHelpers.h"
#include "Tools/Timer.h"

inline void printUsage() {
  std::cout <<
      "Usage: RunPHASTQueries -g <file> -h <file> -d <file> -o <file>\n\n"

      "Runs PHAST queries.\n\n"

      "  -g <file>         input graph in binary format\n"
      "  -h <file>         weighted contraction hierarchy\n"
      "  -d <file>         file that contains source vertices (queries)\n"
      "  -o <file>         place output in <file>\n"
      "  -help             display this help and exit\n";
}

// Some helper aliases.
using VertexAttributes = VertexAttrs<LatLngAttribute>;
using EdgeAttributes = EdgeAttrs<LengthAttribute, TravelTimeAttribute>;
using InputGraph = StaticGraph<VertexAttributes, EdgeAttributes>;

int main(int argc, char* argv[]) {
  try {
    CommandLineParser clp(argc, argv);
    if (clp.isSet("help"))
      printUsage();


    const auto graphFileName = clp.getValue<std::string>("g");
    const auto chFileName = clp.getValue<std::string>("h");
    const auto demandFileName = clp.getValue<std::string>("d");
    auto outputFileName = clp.getValue<std::string>("o");

    // Open the output CSV file.
    if (!endsWith(outputFileName, ".csv"))
      outputFileName += ".csv";
    std::ofstream outputFile(outputFileName);
    if (!outputFile.good())
      throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".csv'");

    // Read the input graph.
    std::ifstream graphFile(graphFileName, std::ios::binary);
    if (!graphFile.good())
      throw std::invalid_argument("file not found -- '" + graphFileName + "'");
    InputGraph graph(graphFile);
    graphFile.close();

    // Read CH from file
    std::ifstream chFile(chFileName, std::ios::binary);
    if (!chFile.good())
      throw std::invalid_argument("file not found -- '" + chFileName + "'");
    CH ch(chFile);
    chFile.close();

    // Build PHAST data structures
    Timer timer;
    RPHASTEnvironment rphastEnv(ch);
    auto sourceSelectionPhase = rphastEnv.getSourcesSelectionPhase();
    auto targetSelectionPhase = rphastEnv.getTargetsSelectionPhase();
    std::vector<int> allVertices(graph.numVertices());
    std::iota(allVertices.begin(), allVertices.end(), 0);

    const auto fullSourceSelection = sourceSelectionPhase.runForKnownVertices(allVertices);
    const auto fullTargetSelection = targetSelectionPhase.runForKnownVertices(allVertices);
    auto fromQuery = rphastEnv.getForwardRPHASTQuery();
    auto toQuery = rphastEnv.getReverseRPHASTQuery();
    const auto initTime = timer.elapsed<std::chrono::microseconds>();
    std::cout << "Initialization time: " << initTime << " us\n";

    int vertex;
    using TrimPolicy = io::trim_chars<>;
    using QuotePolicy = io::no_quote_escape<','>;
    using OverflowPolicy = io::throw_on_overflow;
    using CommentPolicy = io::single_line_comment<'#'>;
    io::CSVReader<1, TrimPolicy, QuotePolicy, OverflowPolicy, CommentPolicy> demandFile(demandFileName);
    demandFile.read_header(io::ignore_no_column, "vertex");
    outputFile << "vertex,from_time,to_time\n";
    while (demandFile.read_row(vertex)) {
      timer.restart();
      fromQuery.run(fullTargetSelection, {vertex});
      const auto fromTime = timer.elapsed<std::chrono::nanoseconds>();
      timer.restart();
      toQuery.run(fullSourceSelection, {vertex});
      const auto toTime = timer.elapsed<std::chrono::nanoseconds>();
      outputFile << vertex << "," << fromTime << "," << toTime << "\n";
    }


  } catch (std::exception& e) {
    std::cerr << argv[0] << ": " << e.what() << std::endl;
    std::cerr << "Try '" << argv[0] <<" -help' for more information." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
