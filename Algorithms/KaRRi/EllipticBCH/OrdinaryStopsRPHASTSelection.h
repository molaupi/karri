

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

        struct PruneSelectionIfDistanceGreaterZero {
            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                static DistLabelT ZeroLabel = 0;
                return allSet(distToV > ZeroLabel);
            }
        };

//        using SelectionPruningCriterion = PruneSelectionIfDistanceGreaterZero;
        using SelectionPruningCriterion = dij::NoCriterion;
        using SelectionPhase = RPHASTSelectionPhase<SelectionPruningCriterion>;

        struct PruneQueryIfMaxRemainingLeewayExceeded {

            PruneQueryIfMaxRemainingLeewayExceeded(const std::vector<int> &remainingLeeways)
                    : remainingLeeways(remainingLeeways) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                KASSERT(v >= 0 && v < remainingLeeways.size());
                KASSERT(remainingLeeways[v] >= 0);
                const DistLabelT maxRemLeeway = remainingLeeways[v];
                return allSet(distToV > maxRemLeeway);
            }

        private:
            const std::vector<int> &remainingLeeways;
        };

    public:

//        using QueryPruningCriterion = PruneQueryIfMaxRemainingLeewayExceeded;
        using QueryPruningCriterion = dij::NoCriterion;

        OrdinaryStopsRPHASTSelection(const InputGraphT &inputGraph, const CH &ch,
                                     const Fleet &fleet, const RouteState &routeState,
                                     const RPHASTEnvironment &rphastEnv) :
                inputGraph(inputGraph),
                ch(ch),
                fleet(fleet),
                routeState(routeState),
                sourcesSelectionPhase(rphastEnv.getSourcesSelectionPhase<SelectionPruningCriterion>()),
                sourcesQueryPrune(),
                targetsSelectionPhase(rphastEnv.getTargetsSelectionPhase<SelectionPruningCriterion>()),
                targetsQueryPrune() {}

        // Run RPHAST target selection for all ordinary stops.
        void runSelectionPhaseForOrdinaryStops() {

            initSourceStopLocations();
            std::vector<int> stopRanks;
            std::vector<int> offsets;
            stopRanks.reserve(sourceStopsByRank.size());
            offsets.reserve(sourceStopsByRank.size());
            for (const auto &sourceStop: sourceStopsByRank) {
                stopRanks.push_back(sourceStop.rank);
                offsets.push_back(-routeState.leewayOfLegStartingAt(sourceStop.stopId));
            }
            // Run RPHAST selection phase for sources. Offsets are -1 * leeway so whenever selection search
            // settles a vertex v, its distance label is -1 * maximum remaining leeway for any stop from which v is
            // reached. The pruning criterion uses this to prune if the maximum remaining leeway becomes negative.
            // Further, we can use the known maximum remaining leeway later to prune queries.
            sourcesSelection = sourcesSelectionPhase.run(stopRanks, offsets);
//            setRemainingLeewaysForSubgraphVertices(sourcesSelection, sourcesSelectionPhase, sourcesRemainingLeeways);


            initTargetStopLocations();
            stopRanks.clear();
            offsets.clear();
            stopRanks.reserve(targetStopsByRank.size());
            offsets.reserve(targetStopsByRank.size());
            for (const auto &targetStop: targetStopsByRank) {
                stopRanks.push_back(targetStop.rank);
                offsets.push_back(-(routeState.leewayOfLegStartingAt(targetStop.prevStopId) - targetStop.offset));
            }
            // Run RPHAST selection phase for targets. Offsets are -1 * leeway so whenever selection search
            // settles a vertex v, its distance label is -1 * maximum remaining leeway for any stop from which v is
            // reached. The pruning criterion uses this to prune if the maximum remaining leeway becomes negative.
            // Further, we can use the known maximum remaining leeway later to prune queries.
            targetsSelection = targetsSelectionPhase.run(stopRanks, offsets);
//            setRemainingLeewaysForSubgraphVertices(targetsSelection, targetsSelectionPhase, targetsRemainingLeeways);

            // Store source / target stops ordered by decreasing rank to allow fast access when reading results of
            // RPHAST query for stops.
            std::sort(sourceStopsByRank.begin(), sourceStopsByRank.end(), [](const auto &a, const auto &b) {
                return a.rank > b.rank;
            });
            std::sort(targetStopsByRank.begin(), targetStopsByRank.end(), [](const auto &a, const auto &b) {
                return a.rank > b.rank;
            });
        }

        const RPHASTSelection &getSourcesSelection() const {
            return sourcesSelection;
        }

        const std::vector<StopWithRank> &getSourceStopsByRank() const {
            return sourceStopsByRank;
        }

        const QueryPruningCriterion &getSourcesQueryPrune() const {
            return sourcesQueryPrune;
        }

        const RPHASTSelection &getTargetsSelection() const {
            return targetsSelection;
        }

        const std::vector<StopWithRankAndOffset> &getTargetStopsByRank() const {
            return targetStopsByRank;
        }

        const QueryPruningCriterion &getTargetsQueryPrune() const {
            return targetsQueryPrune;
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

//        void setRemainingLeewaysForSubgraphVertices(const RPHASTSelection &selection,
//                                                    const SelectionPhase &selectionPhase,
//                                                    std::vector<int> &remainingLeeways) {
//            KASSERT(selection.subGraph.numVertices() == selection.subToFullMapping.size());
//            const auto numVertices = selection.subGraph.numVertices();
//            remainingLeeways.resize(numVertices);
//            for (int i = 0; i < numVertices; ++i) {
//                const auto v = selection.subToFullMapping[i];
//                remainingLeeways[i] = -selectionPhase.getDistanceToClosestTarget(v);
//            }
//        }


        const InputGraphT &inputGraph;
        const CH &ch;
        const Fleet &fleet;
        const RouteState &routeState;


        std::vector<StopWithRank> sourceStopsByRank;
        SelectionPhase sourcesSelectionPhase;
        RPHASTSelection sourcesSelection;
        std::vector<int> sourcesRemainingLeeways;
        QueryPruningCriterion sourcesQueryPrune;

        RPHASTSelection targetsSelection;
        SelectionPhase targetsSelectionPhase;
        std::vector<StopWithRankAndOffset> targetStopsByRank;
        std::vector<int> targetsRemainingLeeways;
        QueryPruningCriterion targetsQueryPrune;

    };
}