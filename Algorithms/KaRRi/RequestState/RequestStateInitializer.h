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
    template<typename FullVehInputGraphT,
            typename PsgInputGraphT,
            typename FullVehTravelTimeCHEnvT>
    class RequestStateInitializer {

    public:
        RequestStateInitializer(const FullVehInputGraphT &fullVehInputGraph, const PsgInputGraphT &psgInputGraph,
                                const FullVehTravelTimeCHEnvT &fullVehTravelTimeChEnv,
                                RequestState &requestState)
                : fullVehInputGraph(fullVehInputGraph),
                  revPsgGraph(psgInputGraph.getReverseGraph()),
                  fullVehTravelTimeCh(fullVehTravelTimeChEnv.getCH()),
                  fullVehTravelTimeChQuery(fullVehTravelTimeChEnv.template getFullCHQuery<>()),
                  requestState(requestState),
                  findPdLocsInRadiusQuery(psgInputGraph, revPsgGraph, requestState.pickups,
                                          requestState.dropoffs) {}


        void initializeRequestState(const Request &req) {
            Timer timer;

            requestState.reset();

            requestState.originalRequest = req;

            const auto originInPsgGraph = fullVehInputGraph.mapToEdgeInPsg(req.origin);
            const auto destInPsgGraph = fullVehInputGraph.mapToEdgeInPsg(req.destination);

            // Find PDLocs in radius
            timer.restart();
            findPdLocsInRadiusQuery.findPDLocs(originInPsgGraph, destInPsgGraph);

            // Log road categories of PDLocs
            for (const auto &p: requestState.pickups)
                requestState.allPDLocsRoadCategoryStats().incCountForCat(
                        fullVehInputGraph.osmRoadCategory(p.fullVehLoc));
            for (const auto &d: requestState.dropoffs)
                requestState.allPDLocsRoadCategoryStats().incCountForCat(
                        fullVehInputGraph.osmRoadCategory(d.fullVehLoc));

            const auto findHaltingSpotsTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.findPDLocsInRadiusTime = findHaltingSpotsTime;
            requestState.stats().numPickups = requestState.numPickups();
            requestState.stats().numDropoffs = requestState.numDropoffs();

            // If there are no pickups or dropoffs, we can skip the rest of the initialization as no vehicle
            // assignment is possible
            if (requestState.numPickups() == 0 || requestState.numDropoffs() == 0)
                return;

            // Calculate the direct distance between the requests origin and destination in full vehicle graph
            timer.restart();
            const auto source = fullVehTravelTimeCh.rank(fullVehInputGraph.edgeHead(req.origin));
            const auto target = fullVehTravelTimeCh.rank(fullVehInputGraph.edgeTail(req.destination));
            fullVehTravelTimeChQuery.run(source, target);
            requestState.directTravelTimeInFullVeh =
                    fullVehTravelTimeChQuery.getDistance() + fullVehInputGraph.travelTime(req.destination);
            KASSERT(requestState.directTravelTimeInFullVeh < INFTY);

            const auto directSearchTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().initializationStats.computeODDistanceTime = directSearchTime;
        }


    private:

        using FullVehTravelTimeCHQuery = typename FullVehTravelTimeCHEnvT::template FullCHQuery<>;

        const FullVehInputGraphT &fullVehInputGraph;
        PsgInputGraphT revPsgGraph;
        const CH &fullVehTravelTimeCh;
        FullVehTravelTimeCHQuery fullVehTravelTimeChQuery;

        RequestState &requestState;

        FindPDLocsInRadiusQuery<PsgInputGraphT> findPdLocsInRadiusQuery;

    };
}