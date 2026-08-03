/// *******************************************************************************
/// Individual BCH Strategy for Repositioning Assignments (batched implementation)
/// *******************************************************************************
#pragma once

#include "Tools/Timer.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/LastStopSearches/RepositioningBucketsEnvironment.h"
#include "Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include <array>

namespace karri {

    template<typename InputGraphT, typename CHEnvT, typename RepositioningBucketsEnvT, typename PDDistancesT, typename CurVehLocToPickupSearchesT, typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    class IndividualBCHStrategyRepositioning {
    public:
        using PDLocsT = PDLocs;
        using LabelSet = LabelSetT;
        using DistanceLabel = typename LabelSet::DistanceLabel;
        static constexpr int K = LabelSet::K;

        IndividualBCHStrategyRepositioning(const InputGraphT &inputGraph,
                                           const Fleet &fleet,
                                           const CHEnvT &chEnv,
                                           const CostCalculator &calculator,
                                           const RepositioningBucketsEnvT &repositionBucketsEnv,
                                           const RouteState &routeState,
                                           CurVehLocToPickupSearchesT& curVehLocToPickupSearches,
                                           RequestState &requestState)
                : inputGraph(inputGraph), fleet(fleet), chEnv(chEnv), ch(chEnv.getCH()), chFullQuery(chEnv.template getFullCHQuery<>()),
                  calculator(calculator), repositionBuckets(repositionBucketsEnv.getBuckets()), routeState(routeState),
                  curVehLocToPickupSearches(curVehLocToPickupSearches),
                  requestState(requestState), vehicleLocator(inputGraph, chEnv, routeState), candidateSubset(static_cast<int>(fleet.size())),
                  reverseSearch(chEnv.template getReverseSearch<ScanBucket, dij::NoCriterion, LabelSet>(ScanBucket(*this), dij::NoCriterion())) {}

        void init() {
            // no-op
        }

        void tryRepositioningAssignments(const PDDistancesT &pdDistances, const PDLocs &pdLocs) {
            Timer timer;
            // Use lightweight subset for deduplicated candidates
            candidateSubset.clear();

            // Run BCH searches in batches of K pickups
            const int numPickups = static_cast<int>(pdLocs.pickups.size());
            int pickupIdx = 0;
            numEntriesScanned = 0;
            int64_t numVerticesSettled = 0;
            int64_t numEdgesRelaxed = 0;

            while (pickupIdx < numPickups) {
                // Prepare batch arrays
                std::array<int, K> sources_ranks{};
                std::array<int, K> offsets{};
                std::array<const PDLoc*, K> batchPickups{};
                std::array<int, K> batchMinPDDists{};

                int j = 0;
                for (; j < K && pickupIdx < numPickups; ++j, ++pickupIdx) {
                    const auto &p = pdLocs.pickups[pickupIdx];
                    sources_ranks[j] = ch.rank(inputGraph.edgeTail(p.loc));
                    offsets[j] = inputGraph.travelTime(p.loc);
                    batchPickups[j] = &p;
                    batchMinPDDists[j] = pdDistances.getMinDirectDistanceForPickup(p.id);
                }
                if (j < K) {
                    // fill remaining slots with copies of the first in batch
                    for (int r = j; r < K; ++r) {
                        sources_ranks[r] = sources_ranks[0];
                        offsets[r] = offsets[0];
                        batchPickups[r] = batchPickups[0];
                        batchMinPDDists[r] = batchMinPDDists[0];
                    }
                }

                // Run one bundled BCH for the batch
                runBchFromPickups(batchPickups, batchMinPDDists, sources_ranks, offsets, numVerticesSettled, numEdgesRelaxed);
            }

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().repositioningAssignmentsStats.searchTime += searchTime;
            requestState.stats().repositioningAssignmentsStats.numEntriesScanned += numEntriesScanned;
            requestState.stats().repositioningAssignmentsStats.numVerticesOrLabelsSettled += numVerticesSettled;
            requestState.stats().repositioningAssignmentsStats.numEdgeRelaxationsInSearchGraph += numEdgesRelaxed;
            timer.restart();

            // For each candidate vehicle, compute exact location and exact CH distances to pickups, then try assignments
            int64_t numAssignmentsTried = 0;
            computeExactAndTryAssignments(pdDistances, pdLocs, numAssignmentsTried);

            const auto tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().repositioningAssignmentsStats.numCandidateVehicles += candidateSubset.size();
            requestState.stats().repositioningAssignmentsStats.numAssignmentsTried += numAssignmentsTried;
            requestState.stats().repositioningAssignmentsStats.tryAssignmentsTime += tryAssignmentsTime;
        }

    private:

        // Scan functor used by reverse CH search. Reads per-search context from parent fields current*.
        struct ScanBucket {
            explicit ScanBucket(IndividualBCHStrategyRepositioning &parent) : parent(parent) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContT &) {
                // Require that at least the first pickup in the batch is set
                if (parent.currentPickups[0] == nullptr)
                    return true;
                // distFromV[i] is the distance from pickup i head to v in the CH downward graph
                for (const auto &entry : parent.repositionBuckets.getBucketOf(v)) {
                    ++parent.numEntriesScanned;
                    const int vehId = entry.targetId;
                    // lbDistToPickup is a DistanceLabel: vectorized distances for the K pickups
                    const DistanceLabel lbDistToPickup = distFromV + entry.distToTarget;
                    const DistanceLabel &minDistToDropoff = parent.currentMinPDDistances;
                    // Compute vectorized lower-bound cost for all K pickups at once
                    const DistanceLabel lowerBoundCost = parent.calculator.calcCostLowerBoundForKPickupsAfterLastStopIndependentOfVehicle(
                            lbDistToPickup, minDistToDropoff, parent.requestState);
                    // If any of the K lower bounds is better than the current best, add vehicle to candidates
                    const auto mask = lowerBoundCost < parent.requestState.getBestCost();
                    if (!anySet(mask)) {
                        // no label in this entry can beat the best -> due to sortedness, no further entries will either
                        break;
                    }
                    // at least one label beat the best -> add vehicle as candidate and continue
                    parent.candidateSubset.insert(vehId);
                }
                return false;
            }

            IndividualBCHStrategyRepositioning &parent;
        };

        // Run a bundled reverse BCH for K pickups at once. pickups and minPDDists are arrays of length K.
        void runBchFromPickups(const std::array<const PDLoc*, K> &pickups,
                               const std::array<int, K> &minPDDists,
                               const std::array<int, K> &sources_ranks,
                               const std::array<int, K> &offsets,
                               int64_t &numVerticesSettled,
                               int64_t &numEdgesRelaxed) {
            // set current context arrays for ScanBucket
            for (int i = 0; i < K; ++i) {
                currentPickups[i] = pickups[i];
                currentMinPDDistances[i] = minPDDists[i];
            }

            // Run bundled reverse search
            reverseSearch.runWithOffset(sources_ranks, offsets);
            numVerticesSettled += reverseSearch.getNumVerticesSettled();
            numEdgesRelaxed += reverseSearch.getNumEdgeRelaxations();

            // clear context
            for (int i = 0; i < K; ++i) {
                currentPickups[i] = nullptr;
            }
            // reset currentMinPDDistances to INFTY
            currentMinPDDistances = DistanceLabel();
        }

        void computeExactAndTryAssignments(const PDDistancesT &pdDistances, const PDLocs &pdLocs,
            int64_t &numAssignmentsTried) {
            Assignment asgn;
            for (const auto vehId : candidateSubset) {
                const auto &veh = fleet[vehId];

                for (const auto &p : pdLocs.pickups) {
                    // Technically, the second argument should be the distance from the repositioning vehicle's
                    // previous idle position to the pickup location. But we don't know this distance. It is only
                    // relevant if the vehicle is still at that position.
                    curVehLocToPickupSearches.addPickupForProcessing(p.id, INFTY);
                }

                curVehLocToPickupSearches.computeExactDistancesVia(veh, pdLocs.pickups);

                for (const auto &p : pdLocs.pickups) {

                    asgn.vehicle = &veh;
                    asgn.pickup = p;
                    KASSERT(curVehLocToPickupSearches.knowsDistance(veh.vehicleId, p.id));
                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, p.id);
                    asgn.pickupStopIdx = 0;
                    asgn.dropoffStopIdx = 0;

                    for (const auto &d : pdLocs.dropoffs) {
                        asgn.dropoff = d;
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup.id, asgn.dropoff.id);
                        requestState.tryAssignment(asgn);
                        ++numAssignmentsTried;
                    }
                }
            }
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CHEnvT &chEnv;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chFullQuery;
        const CostCalculator &calculator;
        const RepositioningBucketsEnvT::BucketContainer &repositionBuckets;
        const RouteState &routeState;
        CurVehLocToPickupSearchesT& curVehLocToPickupSearches;
        RequestState &requestState;

        VehicleLocator<InputGraphT, CHEnvT> vehicleLocator;
        LightweightSubset candidateSubset;

        // Current per-search context accessed by ScanBucket: K pickups in a batch
        std::array<const PDLoc*, K> currentPickups{};
        DistanceLabel currentMinPDDistances{};
        int64_t numEntriesScanned;

        using ReverseSearchType = typename CHEnvT::template UpwardSearch<ScanBucket, dij::NoCriterion, LabelSet>;
        ReverseSearchType reverseSearch;
    };

}
