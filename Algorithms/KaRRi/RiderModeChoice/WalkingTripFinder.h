#pragma once
#include <chrono>
#include "Tools/Timer.h"
#include "WalkingResult.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

    // Computes a walking trip for a given request, i.e., a trip that consists of only walking from origin to destination.
    template<
        typename VehInputGraphT,
        typename PsgInputGraphT,
        typename PsgCHEnvT
    >
    class WalkingTripFinder {
    public:
        WalkingTripFinder(const VehInputGraphT &vehInputGraph,
                          const PsgInputGraphT &psgInputGraph,
                          const PsgCHEnvT &psgChEnv)
            : vehInputGraph(vehInputGraph),
              psgInputGraph(psgInputGraph),
              psgCh(psgChEnv.getCH()),
              psgChQuery(psgChEnv.template getFullCHQuery<>()) {
        }

        WalkingResult findWalkingTrip(const RequestState &requestState, stats::InitializationPerformanceStats &stats) {
            Timer timer;
            const auto &request = requestState.originalRequest;
            const int originPsgEdge = vehInputGraph.toPsgEdge(request.origin);
            const int destPsgEdge = vehInputGraph.toPsgEdge(request.destination);
            int walkingTravelTime = INFTY;
            int travelTimeOfDestEdge = INFTY;
            if (originPsgEdge == destPsgEdge) {
                walkingTravelTime = 0;
                travelTimeOfDestEdge = 0;
            } else {
                const int originHeadRank = psgCh.rank(psgInputGraph.edgeHead(originPsgEdge));
                const int destTailRank = psgCh.rank(psgInputGraph.edgeTail(destPsgEdge));
                const int destOffset = psgInputGraph.length(destPsgEdge);
                KASSERT(destOffset >= 0 && destOffset < INFTY);
                psgChQuery.run(originHeadRank, destTailRank);
                const auto totalWalkingDist = psgChQuery.getDistance() + destOffset; // Distance in m
                KASSERT(totalWalkingDist >= 0 && totalWalkingDist < INFTY);
                // Convert distances to travel times according to rider-specific walking speed
                walkingTravelTime = static_cast<int>(
                    std::round(10.0 * static_cast<double>(totalWalkingDist) / request.walkingSpeed));
                travelTimeOfDestEdge = static_cast<int>(
                    std::round(10.0 * static_cast<double>(destOffset) / request.walkingSpeed));
            }
            const WalkingResult res = {walkingTravelTime, CostCalculator::calcCostForNotUsingVehicle(walkingTravelTime, travelTimeOfDestEdge, requestState)};
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.notUsingVehicleTime += time;
            return res;
        }

    private:
        const VehInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        PsgCHEnvT::template FullCHQuery<> psgChQuery;
    };
}
