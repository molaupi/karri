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
namespace karri {

// Initializes the request state for a new request.
    template<typename VehInputGraphT,
            typename PsgInputGraphT,
            typename VehCHEnvT,
            typename PsgCHEnvT,
            typename VehicleToPDLocQueryT>
    class RequestStateInitializer {

    public:
        RequestStateInitializer(const VehInputGraphT &vehInputGraph, const PsgInputGraphT &psgInputGraph,
                                const VehCHEnvT &vehChEnv, const PsgCHEnvT &psgChEnv,
                                RequestState &requestState, const InputConfig &inputConfig,
                                VehicleToPDLocQueryT &vehicleToPdLocQuery)
                : vehInputGraph(vehInputGraph), psgInputGraph(psgInputGraph),
                  revPsgGraph(psgInputGraph.getReverseGraph()),
                  vehCh(vehChEnv.getCH()), psgCh(psgChEnv.getCH()),
                  vehChQuery(vehChEnv.template getFullCHQuery<>()), psgChQuery(psgChEnv.template getFullCHQuery<>()),
                  requestState(requestState),
                  inputConfig(inputConfig),
                  findPdLocsInRadiusQuery(psgInputGraph, revPsgGraph, inputConfig, requestState.pickups,
                                          requestState.dropoffs),
                  vehicleToPdLocQuery(vehicleToPdLocQuery) {}


        void initializeRequestState(const Request &req) {
            Timer timer;

            requestState.reset();

            requestState.originalRequest = req;

            assert(psgInputGraph.toCarEdge(vehInputGraph.toPsgEdge(req.origin)) == req.origin);
            const auto originInPsgGraph = vehInputGraph.toPsgEdge(req.origin);

            assert(psgInputGraph.toCarEdge(vehInputGraph.toPsgEdge(req.destination)) == req.destination);
            const auto destInPsgGraph = vehInputGraph.toPsgEdge(req.destination);

            findPdLocsInRadiusQuery.findPDLocs(originInPsgGraph, destInPsgGraph);

            // Log road categories of PDLocs
            for (const auto &p: requestState.pickups)
                requestState.allPDLocsRoadCategoryStats().incCountForCat(vehInputGraph.osmRoadCategory(p.loc));
            for (const auto &d: requestState.dropoffs)
                requestState.allPDLocsRoadCategoryStats().incCountForCat(vehInputGraph.osmRoadCategory(d.loc));

            const auto findHaltingSpotsTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.findPDLocsInRadiusTime = findHaltingSpotsTime;
            requestState.stats().numPickups = requestState.numPickups();
            requestState.stats().numDropoffs = requestState.numDropoffs();
            timer.restart();

            // Precalculate the vehicle distances from pickups to origin and from destination to dropoffs for upper bounds on PD distances
            vehicleToPdLocQuery.runReverse(requestState.pickups);
            vehicleToPdLocQuery.runForward(requestState.dropoffs);

            const auto findVehicleToPdLocsDistancesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.findVehicleToPdLocsDistancesTime = findVehicleToPdLocsDistancesTime;

            // Calculate the direct distance between the requests origin and destination
            timer.restart();
            const auto source = vehCh.rank(vehInputGraph.edgeHead(req.origin));
            const auto target = vehCh.rank(vehInputGraph.edgeTail(req.destination));
            vehChQuery.run(source, target);
            requestState.originalReqDirectDist = vehChQuery.getDistance() + vehInputGraph.travelTime(req.destination);

            const auto directSearchTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.computeODDistanceTime = directSearchTime;


            if (!inputConfig.alwaysUseVehicle) {
                // Try pseudo-assignment for passenger walking to destination without using vehicle
                timer.restart();

                const int originHeadRank = psgCh.rank(psgInputGraph.edgeHead(requestState.pickups[0].psgLoc));
                const int destTailRank = psgCh.rank(psgInputGraph.edgeTail(requestState.dropoffs[0].psgLoc));
                const int destOffset = psgInputGraph.travelTime(requestState.dropoffs[0].psgLoc);
                psgChQuery.run(originHeadRank, destTailRank);
                const auto totalDist = psgChQuery.getDistance() + destOffset;
                requestState.tryNotUsingVehicleAssignment(totalDist, destOffset);

                const auto notUsingVehiclesTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().initializationStats.notUsingVehicleTime = notUsingVehiclesTime;
            }
        }


    private:

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;
        using PsgCHQuery = typename PsgCHEnvT::template FullCHQuery<>;

        const VehInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        PsgInputGraphT revPsgGraph;
        const CH &vehCh;
        const CH &psgCh;
        VehCHQuery vehChQuery;
        PsgCHQuery psgChQuery;

        RequestState &requestState;
        const InputConfig &inputConfig;

        FindPDLocsInRadiusQuery<PsgInputGraphT> findPdLocsInRadiusQuery;
        VehicleToPDLocQueryT &vehicleToPdLocQuery;

    };
}