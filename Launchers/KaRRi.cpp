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


#include <cassert>
#include <kassert/kassert.hpp>
#include "Tools/custom_assertion_levels.h"
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <csv.h>

#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/Logging/LogManager.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"

#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"
#include "Algorithms/KaRRi/EllipticBCH/FeasibleEllipticDistances.h"
#include "Algorithms/KaRRi/EllipticBCH/EllipticBucketsEnvironment.h"
#include "Algorithms/KaRRi/EllipticBCH/RadiusEllipticBucketsEnvironment.h"
#include "Algorithms/KaRRi/EllipticBCH/EllipticBCHSearches.h"
#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocsFilter.h"
#include "Algorithms/KaRRi/OrdinaryAssignments/OrdinaryAssignmentsFinder.h"
#include "Algorithms/KaRRi/PbnsAssignments/PBNSAssignmentsFinder.h"
#include "Algorithms/KaRRi/PalsAssignments/PALSAssignmentsFinder.h"
#include "Algorithms/KaRRi/DalsAssignments/DALSAssignmentsFinder.h"
#include "Algorithms/KaRRi/LastStopSearches/SortedLastStopBucketsEnvironment.h"
#include "Algorithms/KaRRi/LastStopSearches/SimpleSortedLastStopBucketsEnvironment.h"
#include "Algorithms/KaRRi/LastStopSearches/UnsortedLastStopBucketsEnvironment.h"
#include "Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h"
#include "Algorithms/KaRRi/RequestState/RequestStateInitializer.h"
#include "Algorithms/KaRRi/AssignmentFinder.h"
#include "Algorithms/KaRRi/SystemStateUpdater.h"
#include "Algorithms/KaRRi/EventSimulation.h"

#ifdef KARRI_USE_CCHS
#include "Algorithms/KaRRi/CCHEnvironment.h"
#else

#include "Algorithms/KaRRi/CHEnvironment.h"

#endif

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT

#include "Algorithms/KaRRi/PDDistanceQueries/BCHStrategy.h"

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
#include "Algorithms/KaRRi/PDDistanceQueries/CHStrategy.h"
#endif


#if KARRI_PALS_STRATEGY == KARRI_COL

#include "Algorithms/KaRRi/PalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_PALS_STRATEGY == KARRI_IND

#include "Algorithms/KaRRi/PalsAssignments/IndividualBCHStrategy.h"

#else // KARRI_PALS_STRATEGY == KARRI_DIJ

#include "Algorithms/KaRRi/PalsAssignments/DijkstraStrategy.h"

#endif

#if KARRI_DALS_STRATEGY == KARRI_COL

#include "Algorithms/KaRRi/DalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_DALS_STRATEGY == KARRI_IND

#include "Algorithms/KaRRi/DalsAssignments//IndividualBCHStrategy.h"

#else // KARRI_DALS_STRATEGY == KARRI_DIJ

#include "Algorithms/KaRRi/DalsAssignments/DijkstraStrategy.h"

#endif


inline void printUsage() {
    std::cout <<
              "Usage: karri -full-veh-g <vehicle network> -red-veh-g <vehicle subnetwork> -psg-g <passenger network> -r <requests> -v <vehicles> -o <file>\n"
              "Runs Karlsruhe Rapid Ridesharing (KaRRi) simulation with given vehicle and passenger road networks,\n"
              "requests, and vehicles. Writes output files to specified base path."
              "  -full-veh-g <file>     full vehicle road network in binary format.\n"
              "  -red-veh-g <file>      reduced vehicle road network (subgraph) in binary format\n"
              "  -psg-g <file>          passenger road (and path) network in binary format.\n"
              "  -r <file>              requests in CSV format.\n"
              "  -v <file>              vehicles in CSV format.\n"
              "  -s <sec>               stop time (in s). (dflt: 60s)\n"
              "  -w <sec>               maximum wait time (in s). (dflt: 300s)\n"
              "  -a <factor>            model parameter alpha for max trip time = a * OD-dist + b (dflt: 1.7)\n"
              "  -b <seconds>           model parameter beta for max trip time = a * OD-dist + b (dflt: 120)\n"
              "  -p-radius <sec>        walking radius (in s) for pickup locations around origin. (dflt: 300s)\n"
              "  -d-radius <sec>        walking radius (in s) for dropoff locations around destination. (dflt: 300s)\n"
              "  -max-num-p <int>       max number of pickup locations to consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
              "  -max-num-d <int>       max number of dropoff locations to consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
              "  -always-veh            if set, the rider is not allowed to walk to their destination without a vehicle trip.\n"
              "  -full-veh-h <file>     contraction hierarchy for the full vehicle network in binary format.\n"
              "  -red-veh-h <file>      contraction hierarchy for the reduced vehicle network in binary format.\n"
              "  -psg-h <file>          contraction hierarchy for the passenger network in binary format.\n"
              "  -full-veh-d <file>     separator decomposition for the full vehicle network in binary format (needed for CCHs).\n"
              "  -red-veh-d <file>      separator decomposition for the reduced vehicle network in binary format (needed for CCHs).\n"
              "  -psg-d <file>          separator decomposition for the passenger network in binary format (needed for CCHs).\n"
              "  -csv-in-LOUD-format    if set, assumes that input files are in the format used by LOUD.\n"
              "  -o <file>              generate output files at name <file> (specify name without file suffix).\n"
              "  -help                  show usage help text.\n";
}

int main(int argc, char *argv[]) {
    using namespace karri;
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }


        // Parse the command-line options.
        InputConfig &inputConfig = InputConfig::getInstance();
        inputConfig.stopTime = clp.getValue<int>("s", 60) * 10;
        inputConfig.maxWaitTime = clp.getValue<int>("w", 300) * 10;
        inputConfig.pickupRadius = clp.getValue<int>("p-radius", inputConfig.maxWaitTime / 10) * 10;
        inputConfig.dropoffRadius = clp.getValue<int>("d-radius", inputConfig.maxWaitTime / 10) * 10;
        inputConfig.maxNumPickups = clp.getValue<int>("max-num-p", INFTY);
        inputConfig.maxNumDropoffs = clp.getValue<int>("max-num-d", INFTY);
        inputConfig.alwaysUseVehicle = clp.isSet("always-veh");
        if (inputConfig.maxNumPickups == 0) inputConfig.maxNumPickups = INFTY;
        if (inputConfig.maxNumDropoffs == 0) inputConfig.maxNumDropoffs = INFTY;
        inputConfig.alpha = clp.getValue<double>("a", 1.7);
        inputConfig.beta = clp.getValue<int>("b", 120) * 10;
        const auto fullVehicleNetworkFileName = clp.getValue<std::string>("full-veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto vehicleFileName = clp.getValue<std::string>("v");
        const auto requestFileName = clp.getValue<std::string>("r");
        const auto fullVehHierarchyFileName = clp.getValue<std::string>("full-veh-h");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto vehSepDecompFileName = clp.getValue<std::string>("veh-d");
        const auto psgSepDecompFileName = clp.getValue<std::string>("psg-d");
        const bool csvFilesInLoudFormat = clp.isSet("csv-in-LOUD-format");
        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);


        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");


        // All networks use same set of vertex attributes
        using VertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;

        // Read the full vehicle network from file.
        std::cout << "Reading full vehicle network from file... " << std::flush;
        using FullVehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, TraversalCostAttribute, MapToEdgeInPsgAttribute, MapToEdgeInReducedVehAttribute, OsmRoadCategoryAttribute>;
        using FullVehicleInputGraph = StaticGraph<VertexAttributes, FullVehicleEdgeAttributes>;
        std::ifstream fullVehicleNetworkFile(fullVehicleNetworkFileName, std::ios::binary);
        if (!fullVehicleNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + fullVehicleNetworkFileName + "'");
        FullVehicleInputGraph fullVehInputGraph(fullVehicleNetworkFile);
        fullVehicleNetworkFile.close();

        if (fullVehInputGraph.numEdges() == 0)
            throw std::invalid_argument("Full vehicle input graph cannot have zero edges.");
        FORALL_VALID_EDGES(fullVehInputGraph, v, e) {
                LIGHT_KASSERT(fullVehInputGraph.edgeId(e) == EdgeIdAttribute::defaultValue());
                LIGHT_KASSERT(fullVehInputGraph.traversalCost(e) >= 0 && fullVehInputGraph.traversalCost(e) < INFTY);
                fullVehInputGraph.edgeTail(e) = v;
                fullVehInputGraph.edgeId(e) = e;
            }
        std::cout << "done.\n";

        using RedVehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, TraversalCostAttribute, MapToEdgeInPsgAttribute, MapToEdgeInFullVehAttribute, OsmRoadCategoryAttribute>;
        using RedVehicleInputGraph = StaticGraph<VertexAttributes, RedVehicleEdgeAttributes>;
        std::unique_ptr<RedVehicleInputGraph> redVehInputGraphPtr;
        if (!clp.isSet("red-veh-g")) {
            std::cout << "Using full vehicle network as reduced vehicle network.\n";
            // If no reduced vehicle network is given, we use the full vehicle network as reduced vehicle network.
            redVehInputGraphPtr = std::make_unique<RedVehicleInputGraph>(fullVehInputGraph);
            FORALL_VALID_EDGES((*redVehInputGraphPtr), v, e) {
                    redVehInputGraphPtr->mapToEdgeInFullVeh(e) = e;
                }
            FORALL_VALID_EDGES(fullVehInputGraph, v, e) {
                    fullVehInputGraph.mapToEdgeInReducedVeh(e) = e;
                }
        } else {
            // Read the reduced vehicle network from file.
            std::cout << "Reading reduced vehicle network from file... " << std::flush;
            const auto redVehicleNetworkFileName = clp.getValue<std::string>("red-veh-g");
            std::ifstream redVehicleNetworkFile(redVehicleNetworkFileName, std::ios::binary);
            if (!redVehicleNetworkFile.good())
                throw std::invalid_argument("file not found -- '" + redVehicleNetworkFileName + "'");
            redVehInputGraphPtr = std::make_unique<RedVehicleInputGraph>(redVehicleNetworkFile);
            redVehicleNetworkFile.close();

            if (redVehInputGraphPtr->numEdges() == 0)
                throw std::invalid_argument("Reduced vehicle input graph cannot have zero edges.");
            FORALL_VALID_EDGES((*redVehInputGraphPtr), v, e) {
                    LIGHT_KASSERT(redVehInputGraphPtr->edgeId(e) == EdgeIdAttribute::defaultValue());
                    redVehInputGraphPtr->edgeTail(e) = v;
                    redVehInputGraphPtr->edgeId(e) = e;
                    LIGHT_KASSERT(redVehInputGraphPtr->mapToEdgeInFullVeh(e) != MapToEdgeInFullVehAttribute::defaultValue());
                    LIGHT_KASSERT(fullVehInputGraph.mapToEdgeInReducedVeh(redVehInputGraphPtr->mapToEdgeInFullVeh(e)) == e, "mapToEdgeInFullVeh(e) = " << redVehInputGraphPtr->mapToEdgeInFullVeh(e));
                }
            std::cout << "done.\n";
        }

        const auto& redVehInputGraph = *redVehInputGraphPtr;
        const auto revFullVehicleGraph = fullVehInputGraph.getReverseGraph();
        const auto revRedVehicleGraph = redVehInputGraph.getReverseGraph();
        unused(revRedVehicleGraph);

        // Read the passenger network from file.
        std::cout << "Reading passenger network from file... " << std::flush;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, MapToEdgeInFullVehAttribute, MapToEdgeInReducedVehAttribute, TravelTimeAttribute>;
        using PsgInputGraph = StaticGraph<VertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        if (psgInputGraph.numEdges() == 0)
            throw std::invalid_argument("Passenger input graph cannot have zero edges.");
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
                LIGHT_KASSERT(psgInputGraph.edgeId(e) == INVALID_ID);
                psgInputGraph.edgeTail(e) = v;
                psgInputGraph.edgeId(e) = e;

                const int eInFullVehGraph = psgInputGraph.mapToEdgeInFullVeh(e);
                // If there is no explicit reduced vehicle network, we use the full vehicle network as reduced vehicle network. Set the mapping accordingly.
                if (!clp.isSet("red-veh-g"))
                    psgInputGraph.mapToEdgeInReducedVeh(e) = eInFullVehGraph;



                if (eInFullVehGraph != MapToEdgeInFullVehAttribute::defaultValue()) {
                    ++numEdgesWithMappingToCar;
                    LIGHT_KASSERT(eInFullVehGraph < fullVehInputGraph.numEdges());

                    LIGHT_KASSERT(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                                  fullVehInputGraph.latLng(fullVehInputGraph.edgeHead(eInFullVehGraph)).latitude());
                    LIGHT_KASSERT(
                            psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == fullVehInputGraph.latLng(
                                    fullVehInputGraph.edgeHead(eInFullVehGraph)).longitude());


                    const int eInRedVehGraph = psgInputGraph.mapToEdgeInReducedVeh(e);
                    LIGHT_KASSERT(fullVehInputGraph.mapToEdgeInReducedVeh(eInFullVehGraph) == eInRedVehGraph);
                    if (eInRedVehGraph != MapToEdgeInReducedVehAttribute::defaultValue()) {
                        LIGHT_KASSERT(eInRedVehGraph < redVehInputGraph.numEdges());
                        LIGHT_KASSERT(redVehInputGraph.mapToEdgeInFullVeh(eInRedVehGraph) == eInFullVehGraph);

                        LIGHT_KASSERT(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                                      redVehInputGraph.latLng(redVehInputGraph.edgeHead(eInRedVehGraph)).latitude());
                        LIGHT_KASSERT(
                                psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == redVehInputGraph.latLng(
                                        redVehInputGraph.edgeHead(eInRedVehGraph)).longitude());
                    }
                }
            }
        unused(numEdgesWithMappingToCar);
        LIGHT_KASSERT(numEdgesWithMappingToCar > 0);
        std::cout << "done.\n";


        // Read the vehicle data from file.
        std::cout << "Reading vehicle data from file... " << std::flush;
        Fleet fleet;
        int location, capacity, startOfServiceTime, endOfServiceTime;
        io::CSVReader<4, io::trim_chars<' '>> vehiclesFileReader(vehicleFileName);

        if (csvFilesInLoudFormat) {
            vehiclesFileReader.read_header(io::ignore_no_column, "initial_location", "seating_capacity",
                                           "start_service_time", "end_service_time");
        } else {
            vehiclesFileReader.read_header(io::ignore_no_column,
                                           "initial_location", "start_of_service_time",
                                           "end_of_service_time", "capacity");
        }

        int maxCapacity = 0;
        while ((csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, capacity, startOfServiceTime, endOfServiceTime)) ||
               (!csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, startOfServiceTime, endOfServiceTime, capacity))) {
            if (location < 0 || location >= fullVehInputGraph.numEdges())
                throw std::invalid_argument("invalid location -- '" + std::to_string(location) + "'");
            if (endOfServiceTime <= startOfServiceTime)
                throw std::invalid_argument("start of service time needs to be before end of service time");
            const int vehicleId = static_cast<int>(fleet.size());
            fleet.push_back({vehicleId, location, startOfServiceTime * 10, endOfServiceTime * 10, capacity});
            maxCapacity = std::max(maxCapacity, capacity);
        }
        std::cout << "done.\n";

        // Create Route State for empty routes.
        RouteState routeState(fleet);



        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<Request> requests;
        int origin, destination, requestTime, numRiders;
        io::CSVReader<4, io::trim_chars<' '>> reqFileReader(requestFileName);

        if (csvFilesInLoudFormat) {
            reqFileReader.read_header(io::ignore_missing_column, "pickup_spot", "dropoff_spot", "min_dep_time",
                                      "num_riders");
        } else {
            reqFileReader.read_header(io::ignore_missing_column, "origin", "destination", "req_time", "num_riders");
        }

        numRiders = -1;
        while (reqFileReader.read_row(origin, destination, requestTime, numRiders)) {
            // Origin and destination are expected as edge IDs in the full graph
            if (origin < 0 || origin >= fullVehInputGraph.numEdges())
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination > fullVehInputGraph.numEdges())
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            if (numRiders > maxCapacity)
                throw std::invalid_argument(
                        "number of riders '" + std::to_string(numRiders) + "' is larger than max vehicle capacity (" +
                        std::to_string(maxCapacity) + ")");
            KASSERT(fullVehInputGraph.mapToEdgeInPsg(origin) != MapToEdgeInPsgAttribute::defaultValue());
            KASSERT(fullVehInputGraph.mapToEdgeInPsg(destination) != MapToEdgeInPsgAttribute::defaultValue());

            const int requestId = static_cast<int>(requests.size());
            if (numRiders == -1) // If number of riders was not specified, assume one rider
                numRiders = 1;
            requests.push_back({requestId, origin, destination, requestTime * 10, numRiders});
            numRiders = -1;
        }
        std::cout << "done.\n";

#ifdef KARRI_USE_CCHS

        // Prepare vehicle CH environment
        using VehCHEnv = CCHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize CCH.
            std::cout << "Reading Seperator Decomposition from file and building CCH... " << std::flush;
            std::ifstream vehSepDecompFile(vehSepDecompFileName, std::ios::binary);
            if (!vehSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + vehSepDecompFileName + "'");
            SeparatorDecomposition vehSepDecomp;
            vehSepDecomp.readFrom(vehSepDecompFile);
            vehSepDecompFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph, vehSepDecomp);
        }

        // Prepare passenger CH environment
        using PsgCHEnv = CCHEnvironment<PsgInputGraph , TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv >(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize CCH.
            std::cout << "Reading Seperator Decomposition from file and building CCH... " << std::flush;
            std::ifstream psgSepDecompFile(psgSepDecompFileName, std::ios::binary);
            if (!psgSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + psgSepDecompFileName + "'");
            SeparatorDecomposition psgSepDecomp;
            psgSepDecomp.readFrom(psgSepDecompFile);
            psgSepDecompFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv >(psgInputGraph, psgSepDecomp);
        }

#else
        // Prepare full vehicle CH environment
        using FullVehTravelTimeCHEnv = CHEnvironment<FullVehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<FullVehTravelTimeCHEnv> fullVehChEnv;
        if (fullVehHierarchyFileName.empty()) {
            std::cout << "Building full vehicle CH... " << std::flush;
            fullVehChEnv = std::make_unique<FullVehTravelTimeCHEnv>(fullVehInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the CH from file.
            std::cout << "Reading full vehicle CH from file... " << std::flush;
            std::ifstream fullVehHierarchyFile(fullVehHierarchyFileName, std::ios::binary);
            if (!fullVehHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + fullVehHierarchyFileName + "'");
            CH fullVehCh(fullVehHierarchyFile);
            fullVehHierarchyFile.close();
            std::cout << "done.\n";
            fullVehChEnv = std::make_unique<FullVehTravelTimeCHEnv>(std::move(fullVehCh));
        }

        // Prepare full vehicle CH environment
        using RedVehTraversalCostCHEnv = CHEnvironment<RedVehicleInputGraph, TraversalCostAttribute>;
        std::unique_ptr<RedVehTraversalCostCHEnv> redVehCostChEnv;
        std::cout << "Building reduced vehicle CH according to traversal cost... " << std::flush;
        redVehCostChEnv = std::make_unique<RedVehTraversalCostCHEnv>(redVehInputGraph);
        std::cout << "done.\n";

        using RedVehTravelTimeCHEnv = CHEnvironment<RedVehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<RedVehTravelTimeCHEnv> redVehTimeChEnv;
        std::cout << "Building reduced vehicle CH according to travel time... " << std::flush;
        redVehTimeChEnv = std::make_unique<RedVehTravelTimeCHEnv>(redVehInputGraph);
        std::cout << "done.\n";

#endif


        using VehicleLocatorImpl = VehicleLocator<RedVehicleInputGraph, RedVehTraversalCostCHEnv>;
        VehicleLocatorImpl locator(redVehInputGraph, *redVehCostChEnv, routeState);

        CostCalculator calc(routeState);
        RequestState reqState(calc);

        // Construct Elliptic BCH bucket environment:
//        static constexpr bool ELLIPTIC_SORTED_BUCKETS = KARRI_ELLIPTIC_BCH_SORTED_BUCKETS;
        using EllipticBucketsEnv = EllipticBucketsEnvironment<RedVehicleInputGraph, RedVehTravelTimeCHEnv, RedVehTraversalCostCHEnv>;
        EllipticBucketsEnv ellipticBucketsEnv(redVehInputGraph, *redVehTimeChEnv, *redVehCostChEnv, routeState, reqState.stats().updateStats);

        // If we use any BCH queries in the PALS or DALS strategies, we construct the according bucket data structure.
        // Otherwise, we use a last stop buckets substitute that only stores which vehicles' last stops are at a vertex.
#if KARRI_PALS_STRATEGY == KARRI_COL || KARRI_PALS_STRATEGY == KARRI_IND || \
    KARRI_DALS_STRATEGY == KARRI_COL || KARRI_DALS_STRATEGY == KARRI_IND

        static constexpr bool LAST_STOP_SORTED_BUCKETS = KARRI_LAST_STOP_BCH_SORTED_BUCKETS;
        using LastStopBucketsEnv = std::conditional_t<LAST_STOP_SORTED_BUCKETS,
                SimpleSortedLastStopBucketsEnvironment<RedVehicleInputGraph, RedVehTraversalCostCHEnv>,
                UnsortedLastStopBucketsEnvironment<RedVehicleInputGraph, RedVehTraversalCostCHEnv>
        >;
        LastStopBucketsEnv lastStopBucketsEnv(redVehInputGraph, *redVehCostChEnv, routeState,
                                              reqState.stats().updateStats);

#else
        using LastStopBucketsEnv = OnlyLastStopsAtVerticesBucketSubstitute;
        LastStopBucketsEnv lastStopBucketsEnv(redVehInputGraph, routeState, fleet.size());
#endif
        // Last stop bucket environment (or substitute) also serves as a source of information on the last stops at vertices.
        using LastStopAtVerticesInfo = LastStopBucketsEnv;

        using EllipticBCHLabelSet = std::conditional_t<KARRI_ELLIPTIC_BCH_USE_SIMD,
                SimdLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using FeasibleEllipticDistancesImpl = FeasibleEllipticDistances<EllipticBCHLabelSet>;
        FeasibleEllipticDistancesImpl feasibleEllipticPickups(fleet.size(), routeState);
        FeasibleEllipticDistancesImpl feasibleEllipticDropoffs(fleet.size(), routeState);


        using EllipticBCHSearchesImpl = EllipticBCHSearches<RedVehicleInputGraph, RedVehTraversalCostCHEnv,
                EllipticBucketsEnv, LastStopAtVerticesInfo, FeasibleEllipticDistancesImpl, EllipticBCHLabelSet>;
        EllipticBCHSearchesImpl ellipticSearches(redVehInputGraph, fleet, ellipticBucketsEnv, lastStopBucketsEnv,
                                                 *redVehCostChEnv, routeState, feasibleEllipticPickups,
                                                 feasibleEllipticDropoffs, reqState);


        // Construct remaining request state
        RelevantPDLocs relOrdinaryPickups(fleet.size());
        RelevantPDLocs relOrdinaryDropoffs(fleet.size());
        RelevantPDLocs relPickupsBeforeNextStop(fleet.size());
        RelevantPDLocs relDropoffsBeforeNextStop(fleet.size());
        using RelevantPDLocsFilterImpl = RelevantPDLocsFilter<FeasibleEllipticDistancesImpl, RedVehicleInputGraph, RedVehTraversalCostCHEnv>;
        RelevantPDLocsFilterImpl relevantPdLocsFilter(fleet, redVehInputGraph, *redVehCostChEnv, calc, reqState, routeState,
                                                      feasibleEllipticPickups, feasibleEllipticDropoffs,
                                                      relOrdinaryPickups, relOrdinaryDropoffs, relPickupsBeforeNextStop,
                                                      relDropoffsBeforeNextStop);

//
//        using VehicleToPDLocQueryImpl = VehicleToPDLocQuery<FullVehicleInputGraph>;
//        VehicleToPDLocQueryImpl vehicleToPdLocQuery(fullVehInputGraph, revFullVehicleGraph);


        // Construct PD-distance query
        using PDDistancesLabelSet = std::conditional_t<KARRI_PD_DISTANCES_USE_SIMD,
                SimdLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using PDDistancesImpl = PDDistances<PDDistancesLabelSet>;
        PDDistancesImpl pdDistances(reqState);

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT
        using PDDistanceQueryImpl = PDDistanceQueryStrategies::BCHStrategy<RedVehicleInputGraph, RedVehTraversalCostCHEnv, PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceQuery(redVehInputGraph, *redVehCostChEnv, pdDistances, reqState);

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
        using PDDistanceQueryImpl = PDDistanceQueryStrategies::CHStrategy<RedVehicleInputGraph, VehCHEnv, PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceQuery(redVehInputGRaph, *vehChEnv, pdDistances, reqState);
#endif

        // Construct ordinary assignments finder:
        using OrdinaryAssignmentsFinderImpl = OrdinaryAssignmentsFinder<PDDistancesImpl>;
        OrdinaryAssignmentsFinderImpl ordinaryInsertionsFinder(relOrdinaryPickups, relOrdinaryDropoffs,
                                                               pdDistances, fleet, calc, routeState, reqState);

        // Construct PBNS assignments finder:
        using CurVehLocToPickupLabelSet = PDDistancesLabelSet;
        using CurVehLocToPickupSearchesImpl = CurVehLocToPickupSearches<RedVehicleInputGraph, VehicleLocatorImpl, RedVehTraversalCostCHEnv, CurVehLocToPickupLabelSet>;
        CurVehLocToPickupSearchesImpl curVehLocToPickupSearches(redVehInputGraph, locator, *redVehCostChEnv, routeState,
                                                                reqState, fleet.size());


        using PBNSInsertionsFinderImpl = PBNSAssignmentsFinder<PDDistancesImpl, CurVehLocToPickupSearchesImpl>;
        PBNSInsertionsFinderImpl pbnsInsertionsFinder(relPickupsBeforeNextStop, relOrdinaryDropoffs,
                                                      relDropoffsBeforeNextStop, pdDistances, curVehLocToPickupSearches,
                                                      fleet, calc, routeState, reqState);

        // Construct PALS strategy and assignment finder:
        using PALSLabelSet = std::conditional_t<KARRI_PALS_USE_SIMD,
                SimdLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;


#if KARRI_PALS_STRATEGY == KARRI_COL
        // Use Collective-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::CollectiveBCHStrategy<RedVehicleInputGraph, RedVehTraversalCostCHEnv, LastStopBucketsEnv, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(redVehInputGraph, fleet, *redVehCostChEnv, lastStopBucketsEnv, pdDistances, calc,
                                  routeState, reqState);
#elif KARRI_PALS_STRATEGY == KARRI_IND
        // Use Individual-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::IndividualBCHStrategy<RedVehicleInputGraph, RedVehCHEnv, LastStopBucketsEnv, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(redVehInputGraph, fleet, *redVehChEnv, calc, lastStopBucketsEnv, pdDistances,
                                  routeState, reqState, reqState.getBestCost());
#else // KARRI_PALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::DijkstraStrategy<RedVehicleInputGraph, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(redVehInputGraph, revRedVehicleGraph, fleet, routeState, lastStopsAtVertices, calc, pdDistances, reqState);
#endif

        using PALSInsertionsFinderImpl = PALSAssignmentsFinder<RedVehicleInputGraph, PDDistancesImpl, PALSStrategy, LastStopAtVerticesInfo>;
        PALSInsertionsFinderImpl palsInsertionsFinder(palsStrategy, redVehInputGraph, fleet, calc, lastStopBucketsEnv,
                                                      routeState, pdDistances, reqState);


        // Construct DALS strategy and assignment finder:

#if KARRI_DALS_STRATEGY == KARRI_COL
        // Use Collective-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::CollectiveBCHStrategy<RedVehicleInputGraph, RedVehTraversalCostCHEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl>;
        DALSStrategy dalsStrategy(redVehInputGraph, fleet, routeState, *redVehCostChEnv, lastStopBucketsEnv, calc,
                                  curVehLocToPickupSearches, reqState, relOrdinaryPickups, relPickupsBeforeNextStop);

#elif KARRI_DALS_STRATEGY == KARRI_IND
        // Use Individual-BCH DALS Strategy
        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
                SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using DALSStrategy = DropoffAfterLastStopStrategies::IndividualBCHStrategy<RedVehicleInputGraph, RedVehCHEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        DALSStrategy dalsStrategy(redVehInputGraph, fleet, *redVehChEnv, calc, lastStopBucketsEnv,
                                  curVehLocToPickupSearches, routeState, reqState, relOrdinaryPickups,
                                  relPickupsBeforeNextStop);
#else // KARRI_DALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra DALS Strategy
        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
                SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using DALSStrategy = DropoffAfterLastStopStrategies::DijkstraStrategy<RedVehicleInputGraph, CurVehLocToPickupSearchesImpl , DALSLabelSet>;
        DALSStrategy dalsStrategy(redVehInputGraph, revRedVehicleGraph, fleet, calc, curVehLocToPickupSearches, routeState, lastStopsAtVertices, reqState, relOrdinaryPickups, relPickupsBeforeNextStop);
#endif

        using DALSInsertionsFinderImpl = DALSAssignmentsFinder<DALSStrategy>;
        DALSInsertionsFinderImpl dalsInsertionsFinder(dalsStrategy);

        using RequestStateInitializerImpl = RequestStateInitializer<FullVehicleInputGraph, PsgInputGraph, FullVehTravelTimeCHEnv>;
        RequestStateInitializerImpl requestStateInitializer(fullVehInputGraph, psgInputGraph, *fullVehChEnv, reqState);

        using InsertionFinderImpl = AssignmentFinder<RequestStateInitializerImpl,
                EllipticBCHSearchesImpl,
                PDDistanceQueryImpl,
                OrdinaryAssignmentsFinderImpl,
                PBNSInsertionsFinderImpl,
                PALSInsertionsFinderImpl,
                DALSInsertionsFinderImpl,
                RelevantPDLocsFilterImpl>;
        InsertionFinderImpl insertionFinder(reqState, requestStateInitializer, ellipticSearches, pdDistanceQuery,
                                            ordinaryInsertionsFinder, pbnsInsertionsFinder, palsInsertionsFinder,
                                            dalsInsertionsFinder, relevantPdLocsFilter);


#if KARRI_OUTPUT_VEHICLE_PATHS
        using VehPathTracker = PathTracker<RedVehicleInputGraph, RedVehCHEnv, std::ofstream>;
        VehPathTracker pathTracker(redVehInputGraph, *redVehChEnv, reqState, routeState, fleet);
#else
        using VehPathTracker = NoOpPathTracker;
        VehPathTracker pathTracker;
#endif


        using SystemStateUpdaterImpl = SystemStateUpdater<RedVehicleInputGraph, EllipticBucketsEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, VehPathTracker, std::ofstream>;
        SystemStateUpdaterImpl systemStateUpdater(redVehInputGraph, reqState, curVehLocToPickupSearches,
                                                  pathTracker, routeState, ellipticBucketsEnv, lastStopBucketsEnv);

        // Initialize last stop state for initial locations of vehicles
        for (const auto &veh: fleet) {
            lastStopBucketsEnv.generateBucketEntries(veh);
        }

        // Run simulation:
        using EventSimulationImpl = EventSimulation<InsertionFinderImpl, SystemStateUpdaterImpl, RouteState>;
        EventSimulationImpl eventSimulation(fleet, requests, insertionFinder, systemStateUpdater,
                                            routeState, true);
        eventSimulation.run();

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}