/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

#include <csv.h>

#include "Algorithms/Dijkstra/BiDijkstra.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Containers/BitVector.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/ParentInfo.h"
#include "DataStructures/Geometry/Area.h"
#include "DataStructures/Geometry/Point.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/SequentialVertexIdAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "ODPairGenerator.h"
#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "Tools/Constants.h"
#include "Tools/OpenMP.h"
#include "Tools/StringHelpers.h"
#include "Tools/Timer.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"

inline void printUsage() {
    std::cout <<
              "Usage: GenerateODPairs -n <num> -g <file> -o <file>\n"
              "       GenerateODPairs -n <num> -g <file> -o <file> -r <rank> [-geom] [-l]\n"
              "       GenerateODPairs -n <num> -g <file> -o <file> -d <dist> [-geom] [-l]\n"
              "       GenerateODPairs -t <tot> -g <file> -o <file>\n"
              "Generates OD pairs with the origin chosen uniformly at random. The destination\n"
              "is either picked uniformly at random or chosen by Dijkstra rank or OD distance.\n"
              "  -l                use physical length as cost function (default: travel time)\n"
              "  -n <num>          generate <num> OD pairs (per rank/distance)\n"
              "  -t <tot>          generate OD pairs with a total length of <tot>\n"
              "  -s <seed>         start random number generator with <seed> (default: 0)\n"
              "  -r <rank>         space-separated list of (expected) Dijkstra ranks\n"
              "  -d <dist>         space-separated list of (expected) OD distances\n"
              "  -geom             choose geometrically distributed ranks/distances\n"
              "  -psg <mode>       output only vertices that are accessible to passengers too as defined by the CarEdgeToPsgEdgeAttribute\n"
              "                         possible values: incoming (vertices with incoming psg accessible edges), in-and-out (vertices with incoming and outgoing psg accessible edges)\n"
              "  -g <file>         input graph in binary format\n"
              "  -a <file>         restrict origins and destinations to polygonal study area\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

template<typename TargetGraphT>
bool hasIncomingPsgAccessibleEdge(const TargetGraphT &targetGraph, const TargetGraphT &revTargetGraph,
                                  const int vertex) {

    // Requires at least one passenger accessible incoming edge:
    FORALL_INCIDENT_EDGES(revTargetGraph, vertex, e) {
        if (targetGraph.toPsgEdge(revTargetGraph.edgeId(e)) != CarEdgeToPsgEdgeAttribute::defaultValue())
            return true;
    }
    return false;
}

template<typename TargetGraphT>
bool hasIncomingAndOutgoingPsgAccessibleEdge(const TargetGraphT &targetGraph, const TargetGraphT &revTargetGraph,
                                             const int vertex) {

    // Requires at least one passenger accessible incoming edge ...
    if (!hasIncomingPsgAccessibleEdge(targetGraph, revTargetGraph, vertex))
        return false;

    // ... and at least one passenger accessible outgoing edge.
    FORALL_INCIDENT_EDGES(targetGraph, vertex, e) {
        if (targetGraph.toPsgEdge(targetGraph.edgeId(e)) != CarEdgeToPsgEdgeAttribute::defaultValue())
            return true;
    }
    return false;
}


int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto useLengths = clp.isSet("l");
        const auto numODPairs = clp.getValue<int>("n");
        const auto totalLength = clp.getValue<int64_t>("t");
        const auto seed = clp.getValue<int>("s", 0);
        const auto expectedRanks = clp.getValues<int>("r");
        const auto expectedDists = clp.getValues<int>("d");
        const auto isGeom = clp.isSet("geom");
        const bool onlyPsgAccessibleVertices = clp.isSet("psg");
        const auto psgAccessibleMode = clp.getValue<std::string>("psg", "in-and-out");
        const auto graphFileName = clp.getValue<std::string>("g");
        const auto areaFileName = clp.getValue<std::string>("a");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";
        const auto partFileStem = "/tmp/" + outputFileName.substr(outputFileName.rfind('/') + 1);

        // Read the graph from file.
        std::cout << "Reading graph from file..." << std::flush;
        using VertexAttributes = VertexAttrs<LatLngAttribute, SequentialVertexIdAttribute>;
        using EdgeAttributes = EdgeAttrs<LengthAttribute, TravelTimeAttribute, EdgeIdAttribute, CarEdgeToPsgEdgeAttribute>;
        using Graph = StaticGraph<VertexAttributes, EdgeAttributes>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        Graph graph(graphFile);
        graphFile.close();
        if (graph.numVertices() > 0 && graph.sequentialVertexId(0) == INVALID_VERTEX)
            FORALL_VERTICES(graph, v) {
                assert(graph.sequentialVertexId(v) == INVALID_VERTEX);
                graph.sequentialVertexId(v) = v;
            }
        if (useLengths)
            FORALL_EDGES(graph, e)graph.travelTime(e) = graph.length(e);
        if (graph.numEdges() > 0 && graph.edgeId(0) == INVALID_ID)
            FORALL_EDGES(graph, e) graph.edgeId(e) = e;
        const auto revGraph = graph.getReverseGraph();
        std::cout << " done.\n";

        auto isPsgAccessible = [&](const int u) {
            if (psgAccessibleMode == "incoming")
                return hasIncomingPsgAccessibleEdge(graph, revGraph, u);
            return hasIncomingAndOutgoingPsgAccessibleEdge(graph, revGraph, u);
        };


        // Read the study area from OSM POLY file.
        BitVector isVertexEligible(graph.numVertices(), false);
        if (!areaFileName.empty()) {
            std::cout << "Reading study area from OSM POLY file..." << std::flush;
            Area studyArea;
            studyArea.importFromOsmPolyFile(areaFileName);
            const auto box = studyArea.boundingBox();
            FORALL_VERTICES(graph, u) {
                const Point p(graph.latLng(u).longitude(), graph.latLng(u).latitude());
                isVertexEligible[u] =
                        (!onlyPsgAccessibleVertices || isPsgAccessible(u)) && box.contains(p) &&
                        studyArea.contains(p);
            }
            std::cout << " done.\n";
        } else {
            FORALL_VERTICES(graph, u) {
                isVertexEligible[u] = !onlyPsgAccessibleVertices || isPsgAccessible(u);
            }
        }

        // Write header to the output file.
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        outputFile << "# Input graph: " << graphFileName << '\n';
        outputFile << "# Methodology: ";

        if (expectedRanks.size() > 0) {

            // Choose the destination by Dijkstra rank.
            if (isGeom)
                outputFile << "geometrically distributed ";
            outputFile << "Dijkstra ranks (" << expectedRanks[0];
            for (auto i = 1; i < expectedRanks.size(); ++i)
                outputFile << " " << expectedRanks[i];
            outputFile << ")\n";
            outputFile << "origin,destination,dijkstra_rank\n";

            Timer timer;
            ProgressBar bar;
#pragma omp parallel
            {
                std::minstd_rand rand(seed + omp_get_thread_num() + 1);
                ODPairGenerator<Graph, TravelTimeAttribute> g(graph, isVertexEligible, seed);

                for (auto i = 0; i < expectedRanks.size(); ++i) {
#pragma omp master
                    {
                        std::cout << "Generating OD pairs (" << expectedRanks[i] << "): ";
                        bar.init(numODPairs);
                    }

                    std::geometric_distribution<> rankDistribution(1.0 / (expectedRanks[i] + 1));
                    const auto partNo = i * omp_get_num_threads() + omp_get_thread_num();
                    const auto partFileName = partFileStem + ".part" + std::to_string(partNo);
                    std::ofstream partFile(partFileName);
                    if (!partFile.good())
                        throw std::invalid_argument("file cannot be opened -- '" + partFileName + "'");
                    partFile << "origin,destination,dijkstra_rank\n";

#pragma omp for schedule(static, 1)
                    for (auto j = 0; j < numODPairs; ++j) {
                        auto rank = isGeom ? rankDistribution(rand) : expectedRanks[i];
                        auto odPairWithRank = g.getRandomODPairChosenByRank(rank);
                        while (odPairWithRank.first.origin == odPairWithRank.first.destination ||
                               odPairWithRank.second < rank ||
                               !isVertexEligible[odPairWithRank.first.destination]) {
                            rank = isGeom ? rankDistribution(rand) : expectedRanks[i];
                            odPairWithRank = g.getRandomODPairChosenByRank(rank, odPairWithRank.first.origin);
                        }
                        const auto src = odPairWithRank.first.origin;
                        const auto dst = odPairWithRank.first.destination;
                        partFile << src << ',' << dst << ',' << odPairWithRank.second << '\n';
                        ++bar;
                    }

#pragma omp master
                    {
                        bar.finish();
                        std::cout << "done.\n";
                    }
                }
            }
            std::cout << "Total time: " << timer.elapsed() << "ms\n";

        } else if (expectedDists.size() > 0) {

            // Choose the destination by distance.
            if (isGeom)
                outputFile << "geometrically distributed ";
            outputFile << "distances (" << expectedDists[0];
            for (auto i = 1; i < expectedDists.size(); ++i)
                outputFile << " " << expectedDists[i];
            outputFile << ")\n";
            outputFile << "origin,destination,distance\n";

            Timer timer;
            ProgressBar bar;
#pragma omp parallel
            {
                std::minstd_rand rand(seed + omp_get_thread_num() + 1);
                ODPairGenerator<Graph, TravelTimeAttribute> g(graph, isVertexEligible, seed);

                for (auto i = 0; i < expectedDists.size(); ++i) {
#pragma omp master
                    {
                        std::cout << "Generating OD pairs (" << expectedDists[i] << "): ";
                        bar.init(numODPairs);
                    }

                    std::geometric_distribution<> distDistribution(1.0 / (expectedDists[i] + 1));
                    const auto partNo = i * omp_get_num_threads() + omp_get_thread_num();
                    const auto partFileName = partFileStem + ".part" + std::to_string(partNo);
                    std::ofstream partFile(partFileName);
                    if (!partFile.good())
                        throw std::invalid_argument("file cannot be opened -- '" + partFileName + "'");
                    partFile << "origin,destination,distance\n";

#pragma omp for schedule(static, 1)
                    for (auto j = 0; j < numODPairs; ++j) {
                        auto dist = isGeom ? distDistribution(rand) : expectedDists[i];
                        auto odPairWithDist = g.getRandomODPairChosenByDistance(dist);
                        while (odPairWithDist.first.origin == odPairWithDist.first.destination ||
                               odPairWithDist.second < dist ||
                               !isVertexEligible[odPairWithDist.first.destination]) {
                            dist = isGeom ? distDistribution(rand) : expectedDists[i];
                            odPairWithDist = g.getRandomODPairChosenByDistance(dist, odPairWithDist.first.origin);
                        }
                        const auto src = odPairWithDist.first.origin;
                        const auto dst = odPairWithDist.first.destination;
                        partFile << src << ',' << dst << ',' << odPairWithDist.second << '\n';
                        ++bar;
                    }

#pragma omp master
                    {
                        bar.finish();
                        std::cout << "done.\n";
                    }
                }
            }
            std::cout << "Total time: " << timer.elapsed() << "ms\n";

        } else if (clp.isSet("n")) {

            // Choose the destination uniformly at random.
            outputFile << "random\n";
            outputFile << "origin,destination\n";

            Timer timer;
            ProgressBar bar;
#pragma omp parallel
            {
                ODPairGenerator<Graph, TravelTimeAttribute> g(graph, isVertexEligible, seed);
#pragma omp master
                {
                    std::cout << "Generating OD pairs: ";
                    bar.init(numODPairs);
                }

                const auto partFileName = partFileStem + ".part" + std::to_string(omp_get_thread_num());
                std::ofstream partFile(partFileName);
                if (!partFile.good())
                    throw std::invalid_argument("file cannot be opened -- '" + partFileName + "'");
                partFile << "origin,destination\n";

#pragma omp for schedule(static, 1)
                for (auto i = 0; i < numODPairs; ++i) {
                    const auto odPair = g.getRandomODPair();
                    partFile << odPair.origin << ',' << odPair.destination << '\n';
                    ++bar;
                }

#pragma omp master
                {
                    bar.finish();
                    std::cout << "done.\n";
                }
            }
            std::cout << "Total time: " << timer.elapsed() << "ms\n";

        } else {

            // Choose random OD pairs with a specified total length.
            const auto reverseGraph = graph.getReverseGraph();
            outputFile << "random with a total length of " << totalLength << "\n";
            outputFile << "origin,destination\n";

            Timer timer;
            ProgressBar bar;
#pragma omp parallel
            {
                using LabelSet = karri::BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
                BiDijkstra<Dijkstra<Graph, TravelTimeAttribute, LabelSet>> biDijkstra(graph, reverseGraph);
                ODPairGenerator<Graph, TravelTimeAttribute> g(graph, isVertexEligible, seed);

                const int64_t totalLengthPerThread = std::ceil(1.0 * totalLength / omp_get_num_threads());
                int64_t totalLen = 0;

#pragma omp master
                {
                    std::cout << "Generating OD pairs: ";
                    bar.init(omp_get_num_threads() * totalLengthPerThread);
                }

                const auto partFileName = partFileStem + ".part" + std::to_string(omp_get_thread_num());
                std::ofstream partFile(partFileName);
                if (!partFile.good())
                    throw std::invalid_argument("file cannot be opened -- '" + partFileName + "'");
                partFile << "origin,destination\n";

                while (totalLen < totalLengthPerThread) {
                    const auto odPair = g.getRandomODPair();
                    partFile << odPair.origin << ',' << odPair.destination << '\n';
                    biDijkstra.run(odPair.origin, odPair.destination);
                    bar += std::min(int64_t{biDijkstra.getDistance()}, totalLengthPerThread - totalLen);
                    totalLen += biDijkstra.getDistance();
                }

#pragma omp master
                {
                    bar.finish();
                    std::cout << "done.\n";
                }
            }
            std::cout << "Total time: " << timer.elapsed() << "ms\n";

        }

        // Merge the part files into a single output file.
        std::cout << "Merging part files into single output file..." << std::flush;
        int src, dst, rank = -1, dist = -1;
        for (auto i = 0; true; ++i) {
            const auto partFileName = partFileStem + ".part" + std::to_string(i);
            std::ifstream partFile(partFileName);
            if (!partFile.good())
                break;
            io::CSVReader<4> partFileReader(partFileName, partFile);
            partFileReader.read_header(
                    io::ignore_missing_column, "origin", "destination", "dijkstra_rank", "distance");
            while (partFileReader.read_row(src, dst, rank, dist)) {

                const auto seqSrc = graph.sequentialVertexId(src);
                const auto seqDst = graph.sequentialVertexId(dst);
                if (onlyPsgAccessibleVertices && !isPsgAccessible(seqSrc)) {
                    throw std::invalid_argument("Trying to output origin that is inaccessible to passengers.");
                }
                if (onlyPsgAccessibleVertices && !isPsgAccessible(seqDst)) {
                    throw std::invalid_argument("Trying to output destination that is inaccessible to passengers.");
                }

                outputFile << seqSrc << ',' << seqDst;
                if (rank != -1) outputFile << ',' << rank;
                if (dist != -1) outputFile << ',' << dist;
                outputFile << '\n';
            }
            partFile.close();
            std::remove(partFileName.c_str());
        }
        std::cout << " done.\n";
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
