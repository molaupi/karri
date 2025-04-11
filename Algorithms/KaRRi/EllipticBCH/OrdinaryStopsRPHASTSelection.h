

#pragma once

#include "Tools/Timer.h"
#include "Algorithms/PHAST/RPHASTEnvironment.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "StopWithRank.h"

namespace karri {

// Runs the RPHAST selection phase for all ordinary stops.
// Can be called once before processing a request batch to update the RPHAST selection for the current route state
// and then be used by all threads for all requests during the batch.
    template<typename InputGraphT>
    class OrdinaryStopsRPHASTSelection {

    public:

        OrdinaryStopsRPHASTSelection(const InputGraphT& inputGraph, const CH& ch,
                                       const Fleet& fleet, const RouteState& routeState,
                                       RPHASTEnvironment& rphastEnv) :
                inputGraph(inputGraph),
                ch(ch),
                fleet(fleet),
                routeState(routeState),
                rphastEnv(rphastEnv) {}

        // Run RPHAST target selection for all ordinary stops.
        void runSelectionPhaseForOrdinaryStops() {

            initSourceStopLocations();
            std::vector<int> stopRanks;
            for (const auto &sourceStop: sourceStopsByRank) {
                stopRanks.push_back(sourceStop.rank);
            }
            rphastEnv.runSourceSelection(stopRanks);

            initTargetStopLocations();
            stopRanks.clear();
            for (const auto &targetStop: targetStopsByRank) {
                stopRanks.push_back(targetStop.rank);
            }
            rphastEnv.runTargetSelection(stopRanks);

            // Store source / target stops ordered by decreasing rank to allow fast access when reading results of
            // RPHAST query for stops.
            std::sort(sourceStopsByRank.begin(), sourceStopsByRank.end(), [](const auto &a, const auto &b) {
                return a.rank > b.rank;
            });
            std::sort(targetStopsByRank.begin(), targetStopsByRank.end(), [](const auto &a, const auto &b) {
                return a.rank > b.rank;
            });
        }


        const std::vector<StopWithRank>& getSourceStopsByRank() const {
            return sourceStopsByRank;
        }

        const std::vector<StopWithRankAndOffset>& getTargetStopsByRank() const {
            return targetStopsByRank;
        }


    private:


        void initSourceStopLocations() {
            sourceStopsByRank.clear();
            sourceStopsByRank.reserve(routeState.getMaxStopId() + 1);
            for (int vehId = 0; vehId < fleet.size(); ++vehId) {
                const auto numStops = routeState.numStopsOf(vehId);
                const auto &stopIds = routeState.stopIdsFor(vehId);
                const auto &stopLocations = routeState.stopLocationsFor(vehId);
                for (int stopIndex = 0; stopIndex < numStops - 1; ++stopIndex) {
                    const auto stopId = stopIds[stopIndex];
                    const auto loc = stopLocations[stopIndex];
                    const auto rank = ch.rank(inputGraph.edgeHead(loc));
                    sourceStopsByRank.push_back({.stopId = stopId, .rank = rank});
                }
            }
        }

        void initTargetStopLocations() {
            targetStopsByRank.clear();
            for (int vehId = 0; vehId < fleet.size(); ++vehId) {
                const auto numStops = routeState.numStopsOf(vehId);
                const auto &stopIds = routeState.stopIdsFor(vehId);
                const auto &stopLocations = routeState.stopLocationsFor(vehId);
                for (int stopIndex = 1; stopIndex < numStops; ++stopIndex) {
                    const auto prevStopId = stopIds[stopIndex - 1];
                    const auto loc = stopLocations[stopIndex];
                    const auto rank = ch.rank(inputGraph.edgeTail(loc));
                    const auto offset = inputGraph.travelTime(loc);
                    targetStopsByRank.push_back({.prevStopId = prevStopId, .rank = rank, .offset = offset});
                }
            }
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const Fleet &fleet;
        const RouteState &routeState;
        RPHASTEnvironment& rphastEnv;

        std::vector<StopWithRank> sourceStopsByRank;
        std::vector<StopWithRankAndOffset> targetStopsByRank;

    };
}