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
#include <routingkit/nested_dissection.h>

#include "Algorithms/CCH/CCH.h"
#include "Algorithms/CCH/CCHMetric.h"
#include "Algorithms/CCH/EliminationTreeQuery.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHQuery.h"
#include "Algorithms/Dijkstra/BiDijkstra.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/ParentInfo.h"
#include "DataStructures/Partitioning/SeparatorDecomposition.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/StringHelpers.h"
#include "Tools/Timer.h"
#include "Tools/CommandLine/ProgressBar.h"

inline void printUsage() {
    std::cout <<
            "Usage: RunP2PAlgo -a CH         -o <file> -g <file>\n"
            "       RunP2PAlgo -a CCH        -o <file> -g <file> [-b <balance>]\n\n"

            "       RunP2PAlgo -a CCH-custom -o <file> -g <file> -s <file> [-n <num>]\n\n"

            "       RunP2PAlgo -a Dij        -o <file> -g <file> -d <file>\n"
            "       RunP2PAlgo -a Bi-Dij     -o <file> -g <file> -d <file>\n"
            "       RunP2PAlgo -a CH         -o <file> -h <file> -d <file>\n"
            "       RunP2PAlgo -a CCH-Dij    -o <file> -g <file> -d <file> -s <file>\n"
            "       RunP2PAlgo -a CCH-tree   -o <file> -g <file> -d <file> -s <file>\n\n"

            "Runs the preprocessing, customization or query phase of various point-to-point\n"
            "shortest-path algorithms, such as Dijkstra, bidirectional search, CH, and CCH.\n\n"

            "  -l                use physical lengths as metric (default: travel times)\n"
            "  -no-stall         do not use the stall-on-demand technique\n"
            "  -a <algo>         run algorithm <algo>\n"
            "  -b <balance>      balance parameter in % for nested dissection (default: 30)\n"
            "  -n <num>          run customization <num> times (default: 1000)\n"
            "  -g <file>         input graph in binary format\n"
            "  -s <file>         separator decomposition of input graph\n"
            "  -h <file>         weighted contraction hierarchy\n"
            "  -d <file>         file that contains OD pairs (queries)\n"
            "  -o-col-name <name> name of column in demand file that contains origin locations (default: 'origin')\n"
            "  -d-col-name <name> name of column in demand file that contains destination locations (default: 'destination')\n"
            "  -edges            set if origins and destinations are edges instead of vertices\n"
            "  -o <file>         place output in <file>\n"
            "  -help             display this help and exit\n";
}

// Some helper aliases.
using VertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
using EdgeAttributes = EdgeAttrs<EdgeTailAttribute, LengthAttribute, TravelTimeAttribute>;
using InputGraph = StaticGraph<VertexAttributes, EdgeAttributes>;
using LabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;

// The query algorithms.
using Dij = Dijkstra<InputGraph, TravelTimeAttribute, LabelSet>;
using BiDij = BiDijkstra<Dij>;
template<bool useStalling>
using CCHDij = CHQuery<LabelSet, useStalling>;
using CCHTree = EliminationTreeQuery<LabelSet>;

// Writes the header line of the output CSV file.
template<typename AlgoT>
inline void writeHeaderLine(std::ofstream &out, AlgoT &) {
    out << "distance,query_time" << '\n';
}

// // Writes a record line of the output CSV file, containing statistics about a single query.
// template<typename AlgoT>
// inline void writeRecordLine(std::ofstream &out, AlgoT &algo, const int, const int offset, const int64_t elapsed) {
//     out << algo.getDistance() + offset << ',' << elapsed << '\n';
// }
//
// template<>
// inline void writeRecordLine(std::ofstream &out, Dij &algo, const int dst, const int offset, const int64_t elapsed) {
//     out << algo.getDistance(dst) + offset << ',' << elapsed << '\n';
// }

inline void writeRecordLine(std::ofstream &out, const int distance, const int64_t elapsed) {
    out << distance << ',' << elapsed << '\n';
}

struct Query {
    int src = INVALID_VERTEX;
    int dst = INVALID_VERTEX;
    int rank = -1;
    int offset = INFTY;
};

// Runs the specified P2P algorithm on the given OD pairs.
template<typename AlgoT>
inline void runQueries(AlgoT &algo, const std::string &demand,
                       const std::string &originColName,
                       const std::string &destinationColName,
                       std::ofstream &out,
                       auto translateSrc,
                       auto translateDst,
                       auto getConstantOffset,
                       auto extractResultDistance) {
    Timer timer;
    int src, dst, rank;
    using TrimPolicy = io::trim_chars<>;
    using QuotePolicy = io::no_quote_escape<','>;
    using OverflowPolicy = io::throw_on_overflow;
    using CommentPolicy = io::single_line_comment<'#'>;
    io::CSVReader<3, TrimPolicy, QuotePolicy, OverflowPolicy, CommentPolicy> demandFile(demand);
    const auto ignore = io::ignore_extra_column | io::ignore_missing_column;
    demandFile.read_header(ignore, originColName, destinationColName, "dijkstra_rank");
    const auto hasRanks = demandFile.has_column("dijkstra_rank");
    if (hasRanks) out << "dijkstra_rank,";
    writeHeaderLine(out, algo);
    std::vector<Query> queries;
    std::cout << "Reading queries..." << std::flush;
    while (demandFile.read_row(src, dst, rank)) {
        if (src == dst) {
            queries.emplace_back(src, dst, rank, 0);
            continue;
        }
        const int offset = getConstantOffset(src, dst);
        const int translatedSrc = translateSrc(src);
        const int translatedDst = translateDst(dst);
        queries.emplace_back(translatedSrc, translatedDst, rank, offset);
    }
    std::cout << " done." << std::endl;

    std::cout << "Running queries... " << std::flush;
    ProgressBar progressBar(queries.size());
    progressBar.setPercentageOutputInterval(5);
    progressBar.setDotOutputInterval(1);
    for (const auto &q: queries) {
        if (hasRanks) out << q.rank << ',';
        if (q.src == q.dst) {
            writeRecordLine(out, 0, 0);
            ++progressBar;
            continue;
        }
        timer.restart();
        algo.run(q.src, q.dst);
        const auto elapsed = timer.elapsed<std::chrono::nanoseconds>();
        const int distance = extractResultDistance(algo, q.dst) + q.offset;
        writeRecordLine(out, distance, elapsed);
        ++progressBar;
    }
    std::cout << " done." << std::endl;
}

// Invoked when the user wants to run the query phase of a P2P algorithm.
inline void runQueries(const CommandLineParser &clp) {
    const auto useLengths = clp.isSet("l");
    const auto noStalling = clp.isSet("no-stall");
    const auto algorithmName = clp.getValue<std::string>("a");
    const auto graphFileName = clp.getValue<std::string>("g");
    const auto sepFileName = clp.getValue<std::string>("s");
    const auto chFileName = clp.getValue<std::string>("h");
    const auto demandFileName = clp.getValue<std::string>("d");
    auto outputFileName = clp.getValue<std::string>("o");

    const auto originColName = clp.getValue<std::string>("o-col-name", "origin");
    const auto destinationColName = clp.getValue<std::string>("d-col-name", "destination");
    const bool locationsAreEdges = clp.isSet("edges");

    std::ifstream graphFile(graphFileName, std::ios::binary);
    if (!graphFile.good())
        throw std::invalid_argument("file not found -- '" + graphFileName + "'");
    InputGraph graph(graphFile);
    graphFile.close();
    FORALL_VERTICES(graph, u) {
        FORALL_INCIDENT_EDGES(graph, u, e) {
            graph.edgeTail(e) = u;
        }
    }

    auto getConstantOffset = [&](const int, const int dst) {
        if (!locationsAreEdges)
            return 0;
        if (useLengths)
            return graph.length(dst);
        return graph.travelTime(dst);
    };

    auto translateSrcEdge = [&](const int id) {
        if (!locationsAreEdges)
            return id;
        return graph.edgeHead(id);
    };

    auto translateDstEdge = [&](const int id) {
        if (!locationsAreEdges)
            return id;
        return graph.edgeTail(id);
    };

    // Open the output CSV file.
    if (!endsWith(outputFileName, ".csv"))
        outputFileName += ".csv";
    std::ofstream outputFile(outputFileName);
    if (!outputFile.good())
        throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".csv'");

    if (algorithmName == "Dij") {
        // Run the query phase of Dijkstra's algorithm.
        if (useLengths)
            FORALL_EDGES(graph, e)
                graph.travelTime(e) = graph.length(e);

        outputFile << "# Graph: " << graphFileName << '\n';
        outputFile << "# OD pairs: " << demandFileName << '\n';

        Dij algo(graph);
        runQueries(algo, demandFileName, originColName, destinationColName, outputFile, translateSrcEdge,
                   translateDstEdge,
                   getConstantOffset,
                   [&](Dij &a, const int dst) { return a.getDistance(dst); });
    } else if (algorithmName == "Bi-Dij") {
        // Run the query phase of bidirectional search.
        if (useLengths)
            FORALL_EDGES(graph, e)
                graph.travelTime(e) = graph.length(e);

        outputFile << "# Graph: " << graphFileName << '\n';
        outputFile << "# OD pairs: " << demandFileName << '\n';

        InputGraph reverseGraph = graph.getReverseGraph();
        BiDij algo(graph, reverseGraph);
        runQueries(algo, demandFileName, originColName, destinationColName, outputFile, translateSrcEdge,
                   translateDstEdge,
                   getConstantOffset,
                   [&](BiDij &a, const int) { return a.getDistance(); });
    } else if (algorithmName == "CH") {
        // Run the query phase of CH.
        std::ifstream chFile(chFileName, std::ios::binary);
        if (!chFile.good())
            throw std::invalid_argument("file not found -- '" + chFileName + "'");
        CH ch(chFile);
        chFile.close();

        outputFile << "# CH: " << chFileName << '\n';
        outputFile << "# OD pairs: " << demandFileName << '\n';

        if (noStalling) {
            CCHDij<false> algo(ch);
            runQueries(algo, demandFileName, originColName, destinationColName, outputFile,
                       [&](const int id) { return ch.rank(translateSrcEdge(id)); },
                       [&](const int id) { return ch.rank(translateDstEdge(id)); },
                       getConstantOffset,
                       [&](CCHDij<false> &a, const int) { return a.getDistance(); });
        } else {
            CCHDij<true> algo(ch);
            runQueries(algo, demandFileName, originColName, destinationColName, outputFile,
                       [&](const int id) { return ch.rank(translateSrcEdge(id)); },
                       [&](const int id) { return ch.rank(translateDstEdge(id)); },
                       getConstantOffset,
                       [&](CCHDij<true> &a, const int) { return a.getDistance(); });
        }
    } else if (algorithmName == "CCH-Dij") {
        // Run the Dijkstra-based query phase of CCH.
        std::ifstream sepFile(sepFileName, std::ios::binary);
        if (!sepFile.good())
            throw std::invalid_argument("file not found -- '" + sepFileName + "'");
        SeparatorDecomposition sepDecomp;
        sepDecomp.readFrom(sepFile);
        sepFile.close();

        CCH cch;
        cch.preprocess(graph, sepDecomp);
        CCHMetric metric(cch, useLengths ? &graph.length(0) : &graph.travelTime(0));
        const auto minCH = metric.buildMinimumWeightedCH();

        outputFile << "# Graph: " << graphFileName << '\n';
        outputFile << "# Separator: " << sepFileName << '\n';
        outputFile << "# OD pairs: " << demandFileName << '\n';

        if (noStalling) {
            CCHDij<false> algo(minCH);
            runQueries(algo, demandFileName, originColName, destinationColName, outputFile,
                       [&](const int id) { return minCH.rank(translateSrcEdge(id)); },
                       [&](const int id) { return minCH.rank(translateDstEdge(id)); },
                       getConstantOffset,
                       [&](CCHDij<false> &a, const int) { return a.getDistance(); });
        } else {
            CCHDij<true> algo(minCH);
            runQueries(algo, demandFileName, originColName, destinationColName, outputFile,
                       [&](const int id) { return minCH.rank(translateSrcEdge(id)); },
                       [&](const int id) { return minCH.rank(translateDstEdge(id)); },
                       getConstantOffset,
                       [&](CCHDij<true> &a, const int) { return a.getDistance(); });
        }
    } else if (algorithmName == "CCH-tree") {
        // Run the elimination-tree-based query phase of CCH.
        std::ifstream sepFile(sepFileName, std::ios::binary);
        if (!sepFile.good())
            throw std::invalid_argument("file not found -- '" + sepFileName + "'");
        SeparatorDecomposition sepDecomp;
        sepDecomp.readFrom(sepFile);
        sepFile.close();

        CCH cch;
        cch.preprocess(graph, sepDecomp);
        CCHMetric metric(cch, useLengths ? &graph.length(0) : &graph.travelTime(0));
        const auto minCH = metric.buildMinimumWeightedCH();

        outputFile << "# Graph: " << graphFileName << '\n';
        outputFile << "# Separator: " << sepFileName << '\n';
        outputFile << "# OD pairs: " << demandFileName << '\n';

        CCHTree algo(minCH, cch.getEliminationTree());
        runQueries(algo, demandFileName, originColName, destinationColName, outputFile,
                   [&](const int id) { return minCH.rank(translateSrcEdge(id)); },
                   [&](const int id) { return minCH.rank(translateDstEdge(id)); },
                   getConstantOffset,
                   [&](CCHTree &a, const int) { return a.getDistance(); });
    } else {
        throw std::invalid_argument("invalid P2P algorithm -- '" + algorithmName + "'");
    }
}

// Invoked when the user wants to run the preprocessing or customization phase of a P2P algorithm.
inline void runPreprocessing(const CommandLineParser &clp) {
    const auto useLengths = clp.isSet("l");
    const auto imbalance = clp.getValue<int>("b", 30);
    const auto numCustomRuns = clp.getValue<int>("n", 1000);
    const auto algorithmName = clp.getValue<std::string>("a");
    const auto graphFileName = clp.getValue<std::string>("g");
    const auto sepFileName = clp.getValue<std::string>("s");
    auto outputFileName = clp.getValue<std::string>("o");

    // Read the input graph.
    std::ifstream graphFile(graphFileName, std::ios::binary);
    if (!graphFile.good())
        throw std::invalid_argument("file not found -- '" + graphFileName + "'");
    InputGraph graph(graphFile);
    graphFile.close();
    if (useLengths)
        FORALL_EDGES(graph, e)
            graph.travelTime(e) = graph.length(e);

    if (algorithmName == "CH") {
        // Run the preprocessing phase of CH.
        if (!endsWith(outputFileName, ".ch.bin"))
            outputFileName += ".ch.bin";
        std::ofstream outputFile(outputFileName, std::ios::binary);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName);

        CH ch;
        ch.preprocess<TravelTimeAttribute>(graph);
        ch.writeTo(outputFile);
    } else if (algorithmName == "CCH") {
        // Run the preprocessing phase of CCH.
        if (imbalance < 0)
            throw std::invalid_argument("invalid imbalance -- '" + std::to_string(imbalance) + "'");

        // Convert the input graph to RoutingKit's graph representation.
        std::vector<float> lats(graph.numVertices());
        std::vector<float> lngs(graph.numVertices());
        std::vector<unsigned int> tails(graph.numEdges());
        std::vector<unsigned int> heads(graph.numEdges());
        FORALL_VERTICES(graph, u) {
            lats[u] = graph.latLng(u).latInDeg();
            lngs[u] = graph.latLng(u).lngInDeg();
            FORALL_INCIDENT_EDGES(graph, u, e) {
                tails[e] = u;
                heads[e] = graph.edgeHead(e);
            }
        }

        // Compute a separator decomposition for the input graph.
        const auto fragment = RoutingKit::make_graph_fragment(graph.numVertices(), tails, heads);
        auto computeSep = [&](const RoutingKit::GraphFragment &fragment) {
            const auto cut = inertial_flow(fragment, imbalance, lats, lngs);
            return derive_separator_from_cut(fragment, cut.is_node_on_side);
        };
        const auto decomp = compute_separator_decomposition(fragment, computeSep);

        // Convert the separator decomposition to our representation.
        SeparatorDecomposition sepDecomp;
        for (const auto &n: decomp.tree) {
            SeparatorDecomposition::Node node;
            node.leftChild = n.left_child;
            node.rightSibling = n.right_sibling;
            node.firstSeparatorVertex = n.first_separator_vertex;
            node.lastSeparatorVertex = n.last_separator_vertex;
            sepDecomp.tree.push_back(node);
        }
        sepDecomp.order.assign(decomp.order.begin(), decomp.order.end());

        if (!endsWith(outputFileName, ".sep.bin"))
            outputFileName += ".sep.bin";
        std::ofstream outputFile(outputFileName, std::ios::binary);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName);
        sepDecomp.writeTo(outputFile);
    } else if (algorithmName == "CCH-custom") {
        // Run the customization phase of CCH.
        std::ifstream sepFile(sepFileName, std::ios::binary);
        if (!sepFile.good())
            throw std::invalid_argument("file not found -- '" + sepFileName + "'");
        SeparatorDecomposition decomp;
        decomp.readFrom(sepFile);
        sepFile.close();

        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + ".csv'");
        outputFile << "# Graph: " << graphFileName << '\n';
        outputFile << "# Separator: " << sepFileName << '\n';
        outputFile << "basic_customization,perfect_customization,construction,total_time\n";

        CCH cch;
        cch.preprocess(graph, decomp);

        Timer timer;
        int basicCustom, perfectCustom, construct, tot;
        for (auto i = 0; i < numCustomRuns; ++i) {
            {
                CCHMetric metric(cch, &graph.travelTime(0));
                timer.restart();
                metric.customize();
                basicCustom = timer.elapsed<std::chrono::microseconds>();
                timer.restart();
                metric.runPerfectCustomization();
                perfectCustom = timer.elapsed<std::chrono::microseconds>();
            }
            {
                CCHMetric metric(cch, &graph.travelTime(0));
                timer.restart();
                metric.buildMinimumWeightedCH();
                tot = timer.elapsed<std::chrono::microseconds>();
            }
            construct = tot - basicCustom - perfectCustom;
            outputFile << basicCustom << ',' << perfectCustom << ',' << construct << ',' << tot << '\n';
        }
    } else {
        throw std::invalid_argument("invalid P2P algorithm -- '" + algorithmName + "'");
    }
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help"))
            printUsage();
        else if (clp.isSet("d"))
            runQueries(clp);
        else
            runPreprocessing(clp);
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
