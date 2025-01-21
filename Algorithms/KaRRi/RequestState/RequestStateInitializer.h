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


#pragma once

#include "FindClosestPDLocInVehicleNetworkQuery.h"

namespace karri {

// Initializes the request state for a new request.
    template<typename FullVehInputGraphT,
            typename PsgInputGraphT,
            typename FullVehCHEnvT,
            typename PsgCHEnvT,
            typename VehicleToPDLocQueryT>
    class RequestStateInitializer {

    public:
        RequestStateInitializer(const FullVehInputGraphT &fullVehInputGraph, const PsgInputGraphT &psgInputGraph,
                                const FullVehCHEnvT &fullVehChEnv, const PsgCHEnvT &psgChEnv,
                                RequestState &requestState, VehicleToPDLocQueryT &vehicleToPdLocQuery)
                : fullVehInputGraph(fullVehInputGraph), psgInputGraph(psgInputGraph),
                  revPsgGraph(psgInputGraph.getReverseGraph()),
                  fullVehCh(fullVehChEnv.getCH()), psgCh(psgChEnv.getCH()),
                  fullVehChQuery(fullVehChEnv.template getFullCHQuery<>()), psgChQuery(psgChEnv.template getFullCHQuery<>()),
                  requestState(requestState),
                  findPdLocsInRadiusQuery(psgInputGraph, revPsgGraph, requestState.pickups,
                                          requestState.dropoffs),
                  vehicleToPdLocQuery(vehicleToPdLocQuery) {}


        void initializeRequestState(const Request &req) {
            Timer timer;

            requestState.reset();

            requestState.originalRequest = req;

            assert(psgInputGraph.mapToEdgeInFullVeh(fullVehInputGraph.mapToEdgeInPsg(req.origin)) == req.origin);
            const auto originInPsgGraph = fullVehInputGraph.mapToEdgeInPsg(req.origin);

            assert(psgInputGraph.mapToEdgeInFullVeh(fullVehInputGraph.mapToEdgeInPsg(req.destination)) == req.destination);
            const auto destInPsgGraph = fullVehInputGraph.mapToEdgeInPsg(req.destination);

            if (!InputConfig::getInstance().alwaysUseVehicle) {
                // Try pseudo-assignment for passenger walking to destination without using vehicle
                timer.restart();

                const int originHeadRank = psgCh.rank(psgInputGraph.edgeHead(originInPsgGraph));
                const int destTailRank = psgCh.rank(psgInputGraph.edgeTail(destInPsgGraph));
                const int destOffset = psgInputGraph.travelTime(destInPsgGraph);
                psgChQuery.run(originHeadRank, destTailRank);
                const auto totalDist = psgChQuery.getDistance() + destOffset;
                requestState.tryNotUsingVehicleAssignment(totalDist, destOffset);

                const auto notUsingVehiclesTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().initializationStats.notUsingVehicleTime = notUsingVehiclesTime;
            }

            // Find PDLocs in radius
            timer.restart();
            findPdLocsInRadiusQuery.findPDLocs(originInPsgGraph, destInPsgGraph);

            // Log road categories of PDLocs
            for (const auto &p: requestState.pickups)
                requestState.allPDLocsRoadCategoryStats().incCountForCat(fullVehInputGraph.osmRoadCategory(p.fullVehLoc));
            for (const auto &d: requestState.dropoffs)
                requestState.allPDLocsRoadCategoryStats().incCountForCat(fullVehInputGraph.osmRoadCategory(d.fullVehLoc));

            const auto findHaltingSpotsTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.findPDLocsInRadiusTime = findHaltingSpotsTime;
            requestState.stats().numPickups = requestState.numPickups();
            requestState.stats().numDropoffs = requestState.numDropoffs();

            // If there are no pickups or dropoffs, we can skip the rest of the initialization as no vehicle
            // assignment is possible
            if (requestState.numPickups() == 0 || requestState.numDropoffs() == 0)
                return;

            // Precalculate the vehicle distances from pickups to origin and from destination to dropoffs for upper bounds on PD distances
            timer.restart();

            vehicleToPdLocQuery.runReverse(req.origin, requestState.pickups);
            vehicleToPdLocQuery.runForward(req.destination, requestState.dropoffs);

            const auto findVehicleToPdLocsDistancesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.findVehicleToPdLocsDistancesTime = findVehicleToPdLocsDistancesTime;

            // Calculate the direct distance between the requests origin and destination in full vehicle graph
            timer.restart();
            const auto source = fullVehCh.rank(fullVehInputGraph.edgeHead(req.origin));
            const auto target = fullVehCh.rank(fullVehInputGraph.edgeTail(req.destination));
            fullVehChQuery.run(source, target);
            requestState.directDistInFullVeh = fullVehChQuery.getDistance() + fullVehInputGraph.travelTime(req.destination);
            KASSERT(requestState.directDistInFullVeh < INFTY);

            const auto directSearchTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.computeODDistanceTime = directSearchTime;
        }


    private:

        using FullVehCHQuery = typename FullVehCHEnvT::template FullCHQuery<>;
        using PsgCHQuery = typename PsgCHEnvT::template FullCHQuery<>;

        const FullVehInputGraphT &fullVehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        PsgInputGraphT revPsgGraph;
        const CH &fullVehCh;
        const CH &psgCh;
        FullVehCHQuery fullVehChQuery;
        PsgCHQuery psgChQuery;

        RequestState &requestState;

        using PdLocsQuery = FindPDLocsInRadiusQuery<PsgInputGraphT>;
//        using PdLocsQuery = FindClosestPDLocInVehicleNetworkQuery<PsgInputGraphT>;

        PdLocsQuery findPdLocsInRadiusQuery;
        VehicleToPDLocQueryT &vehicleToPdLocQuery;

    };
}