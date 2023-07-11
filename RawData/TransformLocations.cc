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
#include <iomanip>
#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "DataStructures/Utilities/OriginDestination.h"
#include "DataStructures/Geometry/Area.h"
#include "DataStructures/Geometry/KDTree.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "RawData/LocationMapper/LatLngToTargetVertexMapper.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "LocationMapper/InputLocationToLatLngMapper.h"
#include "LocationMapper/LocationMapper.h"
#include "LocationMapper/TargetVertexToOutputLocation.h"
#include "LocationMapper/Utils.h"
#include "LocationMapper/CloseEligibleVertexChooser.h"

inline void printUsage() {
    std::cout <<
              "Usage: \n"
              "     TransformLocations -src-g <file> -tar-g <file> -p <file> -o <file>\n"
              "     TransformLocations -src-g <file> -tar-g <file> -v <file> -o <file>\n"
              "     TransformLocations -src-g <file> -p <file> -in-repr <edge|vertex> -out-repr <vertex|edge> -o <file>\n"
              "     TransformLocations -src-g <file> -v <file> -in-repr <edge|vertex> -out-repr <vertex|edge> -o <file>\n"
              "Takes a set of origin-destination pairs or initial vehicle locations and transforms the locations to the "
              "geographically closest locations in a target network. The target network should geographically encompass "
              "all locations.\n"
              "Input locations may be specified as vertices or edges in a source network or coordinates in a GCS (see -in-repr).\n"
              "Output locations may be vertices or edges in the target network (see -out-repr).\n"
              "Vertices can be transformed to incident edges in the same network and vice versa by specifying identical source and\n"
              "target networks (mapping done with vertex/edge IDs).\n"
              "  -tar-g <file>          target graph file in binary format\n"
              "  -d <dist>              maximum geographical distance (in m) for transformation from one vertex to another.\n"
              "                              Set to 0 to allow unlimited radius (dflt).\n"
              "  -psg                   if set, restrict eligible locations in target graph to those accessible by vehicles and passengers.\n"
              "  -p <file>              path to CSV file containing input OD-pairs\n"
              "  -o-col-name <name>     name of origin column in OD-pairs file (dflt: 'origin')\n"
              "  -d-col-name <name>     name of destination column in OD-pairs file (dflt: 'destination')\n"
              "  -v <file>              path to CSV file containing input vehicles\n"
              "  -l-col-name <name>     name of initial location column in vehicles file (dflt: 'initial_location')\n"
              "  -in-repr <repr>        Representation of locations in input. Possible values: "
              "                             vertex-id   (ID of vertex in source graph; requires -src-g; default)\n"
              "                             edge-id     (ID of edge in source graph; requires -src-g)\n"
              "                             lat-lng     (Latitude and longitude in format '(lat|lng)')\n"
              "                             epsg-31467  (Easting (=X) and northing (=Y) in EPSG 31467 in format '(X|Y)')\n"
              "  -src-g <file>          source graph file in binary format (required for certain values of -in-repr)\n"
              "  -out-repr <repr>       Representation of locations in output. Possible values: "
              "                             vertex-id (dflt)\n"
              "                             edge-id\n"
              "  -a <file>              optional .poly file describing area encompassing all OD-pairs/all vehicle locations.\n"
              "  -o <file>              place transformed OD-pairs in <file>\n"
              "  -help                  display this help and exit\n";
}

template<typename TargetGraphT>
struct IsEdgePsgEligible {
    explicit IsEdgePsgEligible(const TargetGraphT &graph) : targetGraph(graph) {}

    bool operator()(const int edgeId) const {
        return targetGraph.toPsgEdge(edgeId) != CarEdgeToPsgEdgeAttribute::defaultValue();
    }

    const TargetGraphT &targetGraph;
};

template<typename TargetGraphT>
struct IsVertexPsgEligible {

    IsVertexPsgEligible(IsEdgePsgEligible<TargetGraphT> isEdgeEligible, const TargetGraphT &revTargetGraph)
            : isEdgeEligible(isEdgeEligible), revTargetGraph(revTargetGraph) {}

    bool operator()(const int v) const {
        FORALL_INCIDENT_EDGES(revTargetGraph, v, e) {
            if (isEdgeEligible(revTargetGraph.edgeId(e)))
                return true;
        }
        return false;
    }

    IsEdgePsgEligible<TargetGraphT> isEdgeEligible;
    const TargetGraphT &revTargetGraph;
};


template<typename InputLocsT, typename LocationMapperT>
void transformPairs(const InputLocsT &inputPairs,
                    LocationMapperT &locationMapper,
                    const std::string &outputFileName) {

    std::cout << "Transforming OD-pairs ... " << std::flush;
    std::vector<OriginDestination> outputPairs;
    for (const auto& curPair : inputPairs) {

        int mappingOfOrigin = INVALID_VERTEX;
        bool success = locationMapper.mapLocation(curPair.first, mappingOfOrigin);
        int mappingOfDestination = INVALID_VERTEX;
        success &= locationMapper.mapLocation(curPair.second, mappingOfDestination, mappingOfOrigin);

        if (!success)
            continue;

        outputPairs.emplace_back(mappingOfOrigin, mappingOfDestination);
    }
    std::cout << " done.\n";

    std::cout << "Writing " << outputPairs.size() << " pairs to output..." << std::flush;
    std::ofstream out(outputFileName + ".csv");
    if (!out.good())
        throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
    out << "origin,destination\n";
    for (const auto &pair: outputPairs) {
        out << pair.origin << ',' << pair.destination << '\n';
    }
    out.close();
    std::cout << "done.\n";

}


template<typename InputLocsT, typename LocationMapperT>
void transformInitialVehicleLocations(const InputLocsT &inputInitialVehLocations,
                                      LocationMapperT &locationMapper,
                                      const std::string &outputFileName) {

    std::cout << "Transforming initial vehicle locations ... " << std::flush;
    std::vector<int> outputInitialVehicleLocations;
    for (const auto& loc : inputInitialVehLocations) {

        int mappingOfLoc = INVALID_VERTEX;
        if (!locationMapper.mapLocation(loc, mappingOfLoc))
            continue;

        outputInitialVehicleLocations.emplace_back(mappingOfLoc);
    }
    std::cout << " done.\n";

    std::cout << "Writing " << outputInitialVehicleLocations.size() << " initial vehicle locations to output..."
              << std::flush;
    std::ofstream out(outputFileName + ".csv");
    if (!out.good())
        throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
    out << "initial_location\n";
    for (const auto &location: outputInitialVehicleLocations) {
        out << location << '\n';
    }
    out.close();
    std::cout << "done.\n";
}

template<TransformMode mode,
        typename InputLocsT,
        typename LocationMapperT, typename ...Args>
void callTransformWithRightMode(const InputLocsT &inputLocs,
                                LocationMapperT &locationMapper,
                                Args &...args) {
    if constexpr (mode == ODPairs) {
        transformPairs(inputLocs, locationMapper, args...);
    } else {
        transformInitialVehicleLocations(inputLocs, locationMapper, args...);
    }
}

template<TransformMode mode,
        typename InputLocsT,
        typename GraphT,
        typename InputMapperT,
        typename OutputMapperT,
        typename IsVertexEligibleT,
        typename ...Args>
void constructLocationMapper(InputLocsT &inputLocs,
                             InputMapperT &inputMapper,
                             OutputMapperT &outputMapper,
                             const IsVertexEligibleT &isVertexEligible,
                             const GraphT &targetGraph,
                             const double &maxVertexMatchDist,
                             const std::unique_ptr<Area> &studyArea,
                             Args &... args) {

    using VertexMatchesLogger = std::ofstream;
    using ProxMapper = LatLngToTargetVertexMapper<GraphT, IsVertexEligibleT, VertexMatchesLogger>;
    ProxMapper proxMapper(targetGraph, maxVertexMatchDist, isVertexEligible, studyArea.get());

    using LocMapper = LocationMapper<InputMapperT, ProxMapper, OutputMapperT>;
    LocMapper locMapper(inputMapper, proxMapper, outputMapper);

    callTransformWithRightMode<mode>(inputLocs, locMapper, args...);
}


template<TransformMode mode, LocType inLocType,
        typename GraphT,
        typename OutputMapperT,
        typename IsVertexEligibleT,
        typename InputReaderT,
        typename ...Args>
void decideInputMapper(OutputMapperT &outputMapper,
                       const IsVertexEligibleT &isVertexEligible,
                       const GraphT &targetGraph,
                       const std::unique_ptr<GraphT> &sourceGraph,
                       InputReaderT& inputReader,
                       Args &...args) {

    if constexpr (inLocType == VERTEX_ID) {
        assert(sourceGraph);
        using InputMapper = input_location_to_lat_lng::VertexToLatLngMapper<GraphT>;
        InputMapper inputMapper(*sourceGraph);
        const auto inputLocs = inputReader.template readInput<VERTEX_ID>();
        constructLocationMapper<mode>(inputLocs, inputMapper, outputMapper, isVertexEligible, targetGraph, args...);
    } else if constexpr (inLocType == EDGE_ID) {
        assert(sourceGraph);
        using InputMapper = input_location_to_lat_lng::EdgeToLatLngMapper<GraphT>;
        InputMapper inputMapper(*sourceGraph);
        const auto inputLocs = inputReader.template readInput<EDGE_ID>();
        constructLocationMapper<mode>(inputLocs, inputMapper, outputMapper, isVertexEligible, targetGraph, args...);
    } else if constexpr (inLocType == LATLNG) {
        using InputMapper = input_location_to_lat_lng::LatLngToLatLngMapper;
        InputMapper inputMapper;
        const auto inputLocs = inputReader.template readInput<LATLNG>();
        constructLocationMapper<mode>(inputLocs, inputMapper, outputMapper, isVertexEligible, targetGraph, args...);
    } else if constexpr (inLocType == EPSG_31467) {
        using InputMapper = input_location_to_lat_lng::Epsg31467ToLatLngMapper;
        InputMapper inputMapper;
        const auto inputLocs = inputReader.template readInput<EPSG_31467>();
        constructLocationMapper<mode>(inputLocs, inputMapper, outputMapper, isVertexEligible, targetGraph, args...);
    } else {
        assert(false);
    }
}


template<TransformMode mode, LocType inLocType, LocType outLocType,
        typename GraphT,
        typename IsVertexEligibleT,
        typename IsEdgeEligibleT,
        typename ...Args>
void decideOutputMapper(const IsVertexEligibleT &isVertexEligible,
                        const IsEdgeEligibleT &isEdgeEligible,
                        const GraphT &targetGraph,
                        const GraphT &revTargetGraph,
                        const std::unique_ptr<GraphT> &sourceGraph,
                        Args &...args) {

    using CloseEligibleVertexChooserT = CloseEligibleVertexChooser<GraphT, TravelTimeAttribute, IsVertexEligibleT>;
    CloseEligibleVertexChooserT closeEligibleVertexChooser(targetGraph, isVertexEligible);

    if constexpr (outLocType == VERTEX_ID) {
        using OutputMapper = target_vertex_to_output_location::AvoidVertex<CloseEligibleVertexChooserT>;
        OutputMapper outputMapper(closeEligibleVertexChooser);
        decideInputMapper<mode, inLocType>(outputMapper, isVertexEligible, targetGraph, sourceGraph,
                                           args...);
    } else if constexpr (outLocType == EDGE_ID && inLocType != EDGE_ID) {
        using OutputMapper = target_vertex_to_output_location::RandomEligibleIncidentEdge<GraphT, IsEdgeEligibleT, CloseEligibleVertexChooserT>;
        OutputMapper outputMapper(revTargetGraph, isEdgeEligible, closeEligibleVertexChooser);
        decideInputMapper<mode, inLocType>(outputMapper, isVertexEligible, targetGraph, sourceGraph,
                                           args...);
    } else if constexpr (outLocType == EDGE_ID && inLocType == EDGE_ID) {
        using OutputMapper = target_vertex_to_output_location::EligibleIncidentEdgeWithClosestTail<GraphT, IsEdgeEligibleT, CloseEligibleVertexChooserT>;
        OutputMapper outputMapper(*sourceGraph, targetGraph, revTargetGraph, isEdgeEligible,
                                  closeEligibleVertexChooser);
        decideInputMapper<mode, inLocType>(outputMapper, isVertexEligible, targetGraph, sourceGraph,
                                           args...);
    } else {
        assert(false);
    }
}


template<TransformMode mode, LocType inLocType, LocType outLocType,
        typename GraphT, typename ...Args>
void decideTargetLocationEligibility(const bool onlyPsgAccessible,
                                     const GraphT &targetGraph,
                                     Args &...args) {
    const GraphT revTargetGraph = targetGraph.getReverseGraph();

    if (onlyPsgAccessible) {
        IsEdgePsgEligible<GraphT> isEdgePsgEligible(targetGraph);
        IsVertexPsgEligible<GraphT> isVertexPsgEligible(isEdgePsgEligible, revTargetGraph);
        decideOutputMapper<mode, inLocType, outLocType>( isVertexPsgEligible, isEdgePsgEligible, targetGraph,
                                                        revTargetGraph, args...);
    } else {
        AlwaysEligible alwaysEligible;
        decideOutputMapper<mode, inLocType, outLocType>( alwaysEligible, alwaysEligible, targetGraph,
                                                        revTargetGraph,
                                                        args...);
    }
}

template<TransformMode mode,
        LocType inLocType,
        typename ...Args>
void decideOutputLocType(const LocType &outLocType, Args &...args) {
    if (outLocType == VERTEX_ID) {
        decideTargetLocationEligibility<mode, inLocType, VERTEX_ID>(args...);
    } else if (outLocType == EDGE_ID) {
        decideTargetLocationEligibility<mode, inLocType, EDGE_ID>( args...);
    } else {
        assert(false);
    }
}


template<TransformMode mode,
        typename ...Args>
void decideInputLocType(const LocType &inLocType,
                        Args &...args) {
    if (inLocType == VERTEX_ID) {
        decideOutputLocType<mode, VERTEX_ID>(args...);
    } else if (inLocType == EDGE_ID) {
        decideOutputLocType<mode, EDGE_ID>(args...);
    } else if (inLocType == LATLNG) {
        decideOutputLocType<mode, LATLNG>(args...);
    } else {
        decideOutputLocType<mode, EPSG_31467>(args...);
    }

}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        if ((clp.isSet("p") && clp.isSet("v")) || (!clp.isSet("p") && !clp.isSet("v")))
            throw std::invalid_argument("Either -p or -v need to be set and not both at the same time.");

        const auto sourceGraphFileName = clp.getValue<std::string>("src-g");
        const auto targetGraphFileName = clp.getValue<std::string>("tar-g");
        const bool onlyPsgAccessible = clp.isSet("psg");
        const auto inRepr = clp.getValue<std::string>("in-repr", "vertex-id");
        const auto outRepr = clp.getValue<std::string>("out-repr", "vertex-id");
        auto maxDist = clp.getValue<double>("d", 0.0f);
        if (maxDist == 0.0f) maxDist = std::numeric_limits<double>::max();
        const auto areaFileName = clp.getValue<std::string>("a");
        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);
        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        using Graph = StaticGraph<VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>, EdgeAttrs<EdgeIdAttribute, CarEdgeToPsgEdgeAttribute, EdgeTailAttribute, TravelTimeAttribute>>;

        // Read target network
        std::cout << "Reading target network from file... " << std::flush;
        std::ifstream targetGraphFile(targetGraphFileName, std::ios::binary);
        if (!targetGraphFile.good())
            throw std::invalid_argument("file not found -- '" + targetGraphFileName + "'");
        Graph targetGraph(targetGraphFile);
        targetGraphFile.close();
        FORALL_VALID_EDGES(targetGraph, v, e) {
                targetGraph.edgeId(e) = e;
                targetGraph.edgeTail(e) = v;
            }
        std::cout << "done.\n";


        EnumParser<LocType> locTypeParser;
        const LocType inLocType = locTypeParser(inRepr);
        const LocType outLocType = locTypeParser(outRepr);


        if ((inLocType == VERTEX_ID || inLocType == EDGE_ID) && sourceGraphFileName.empty())
            throw std::invalid_argument(
                    "Need to specify -src-g if input locations are given as vertex or edge ids of a source network.");
        if (!sourceGraphFileName.empty() && !(inLocType == VERTEX_ID || inLocType == EDGE_ID))
            std::cerr
                    << "WARNING: Source network is given but input locations are given as coordinates (lat-lng or epsg-31467). Source network will be ignored."
                    << std::endl;


        std::unique_ptr<Graph> sourceGraphPtr;
        std::unique_ptr<std::vector<int32_t>> sourceGraphOrigToSeqIdPtr;
        if (!sourceGraphFileName.empty() && (inLocType == VERTEX_ID || inLocType == EDGE_ID)) {
            // Read the source network from file.
            std::cout << "Reading source network from file... " << std::flush;
            std::ifstream sourceGraphFile(sourceGraphFileName, std::ios::binary);
            if (!sourceGraphFile.good())
                throw std::invalid_argument("file not found -- '" + sourceGraphFileName + "'");
            sourceGraphPtr = std::make_unique<Graph>(sourceGraphFile);
            sourceGraphFile.close();
            Graph &sourceGraph = *sourceGraphPtr;
            sourceGraphOrigToSeqIdPtr = std::make_unique<std::vector<int32_t>>();
            std::vector<int32_t> &origIdToSeqId = *sourceGraphOrigToSeqIdPtr;
            if (inLocType != EDGE_ID || (sourceGraph.numEdges() > 0 && sourceGraph.edgeId(0) == INVALID_ID)) {
                origIdToSeqId.assign(sourceGraph.numEdges(), INVALID_ID);
                std::iota(origIdToSeqId.begin(), origIdToSeqId.end(), 0);
                FORALL_VALID_EDGES(sourceGraph, v, e) {
                        assert(sourceGraph.edgeId(e) == INVALID_ID);
                        sourceGraph.edgeId(e) = e;
                        sourceGraph.edgeTail(e) = v;
                    }
            } else {
                FORALL_VALID_EDGES(sourceGraph, v, e) {
                        assert(sourceGraph.edgeId(e) != INVALID_ID);
                        if (sourceGraph.edgeId(e) >= origIdToSeqId.size()) {
                            const auto numElementsToBeInserted = sourceGraph.edgeId(e) + 1 - origIdToSeqId.size();
                            origIdToSeqId.insert(origIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                        }
                        assert(origIdToSeqId[sourceGraph.edgeId(e)] == INVALID_ID);
                        origIdToSeqId[sourceGraph.edgeId(e)] = e;
                        sourceGraph.edgeId(e) = e;
                        sourceGraph.edgeTail(e) = v;
                    }
            }
            std::cout << "done.\n";
        }


        std::unique_ptr<Area> studyArea;
        if (!areaFileName.empty()) {
            studyArea = std::make_unique<Area>();
            studyArea->importFromOsmPolyFile(areaFileName);
        }

        if (clp.isSet("p")) {
            // Transform OD-pairs:
            const auto sourcePairsFileName = clp.getValue<std::string>("p");
            const auto oColName = clp.getValue<std::string>("o-col-name", "origin");
            const auto dColName = clp.getValue<std::string>("d-col-name", "destination");

            using InputReader = transform_locations_input::ODPairReader<Graph>;
            InputReader inputReader(sourceGraphPtr, sourceGraphOrigToSeqIdPtr, sourcePairsFileName, oColName, dColName);

            decideInputLocType<ODPairs>(inLocType, outLocType, onlyPsgAccessible, targetGraph,
                                        sourceGraphPtr, inputReader,  maxDist, studyArea,
                                        outputFileName);

        } else if (clp.isSet("v")) {

            // Transform Vehicle Locations
            const auto sourceVehiclesFileName = clp.getValue<std::string>("v");
            const auto lColName = clp.getValue<std::string>("l-col-name", "initial_location");

            using InputReader = transform_locations_input::InitialVehicleLocationReader<Graph>;
            InputReader inputReader(sourceGraphPtr, sourceGraphOrigToSeqIdPtr, sourceVehiclesFileName, lColName);

            decideInputLocType<InitialVehicleLocations>(inLocType, outLocType, onlyPsgAccessible,
                                                        targetGraph, sourceGraphPtr, inputReader,  maxDist,
                                                        studyArea, outputFileName);
        }


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}