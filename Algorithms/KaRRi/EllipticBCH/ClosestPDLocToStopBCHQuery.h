#pragma once

#include "DataStructures/Containers/TimestampedVector.h"
#include "Algorithms/CH/CHQuery.h"

namespace karri {

// Given a set H of halting spots, for each pair of consecutive stops (s,s') of each vehicle veh, this search finds the
// smallest distance from s to any halting spot min_{h \in H} dist(s, h) as well as the smallest distance from any
// halting spot to s' min_{h \in H} dist(h,s').#
    template<typename InputGraphT, typename CHEnvT, typename SourceBucketsT, typename TargetBucketsT, typename PruningCriterionT = dij::NoCriterion, typename LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>
    class ClosestHaltingSpotToRegularStopBCHQuery {

        using LabelSet = LabelSetT;
        using PruningCriterion = PruningCriterionT; // Criterion to prune the Dijkstra searches that constitute the BCHs.

    public:

        ClosestHaltingSpotToRegularStopBCHQuery(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                                const SourceBucketsT &sourceBuckets,
                                                const TargetBucketsT &targetBuckets,
                                                const int initialNumStops,
                                                const int &maxStopId, const int &maxLeeway,
                                                const RouteState &routeState,
                                                PruningCriterion pruneForwardSearch = {},
                                                PruningCriterion pruneReverseSearch = {}) :
                inputGraph(inputGraph),
                ch(chEnv.getCH()),
                sourceBuckets(sourceBuckets),
                targetBuckets(targetBuckets),
                maxStopId(maxStopId),
                maxLeeway(maxLeeway),
                routeState(routeState),
                toSearch(ch.downwardGraph(), {}, pruneReverseSearch),
                fromSearch(ch.upwardGraph(), {}, pruneForwardSearch),
                stopsInToSearchSpace(initialNumStops),
                stopsInFromSearchSpace(initialNumStops),
                minDistFromStop(initialNumStops, INFTY),
                minDistToNextStop(initialNumStops, INFTY) {}

        template<typename HaltingSpotsT, typename HaltingSpotsAtExistingStopsT>
        void run(const HaltingSpotsT &haltingSpots, const HaltingSpotsAtExistingStopsT &haltingSpotsAtExistingStops) {

            initToSearch(haltingSpots);
            initFromSearch(haltingSpots);
            initHaltingSpotsAtExistingStops(haltingSpotsAtExistingStops);

            // Run to search, finding the minimum distance from each regular stop s to any halting spot.
            while (!stopToSearch()) {
                const auto v = toSearch.settleNextVertex();
                scanSourceBucketAtRank(v);
            }

            // Run from search, finding the minimum distance to each regular stop s from any halting spot.
            while (!stopFromSearch()) {
                const auto v = fromSearch.settleNextVertex();
                scanTargetBucketAtRank(v);
            }
        }

        Subset &getStopsInToSearchSpace() {
            return stopsInToSearchSpace;
        }

        Subset &getStopsInFromSearchSpace() {
            return stopsInFromSearchSpace;
        }


        // Get the smallest distance from stop s to any halting spot, min_{h \in H} dist(s, h).
        int getDistToClosestSpotFromStop(const int stopId) {
            assert(stopId >= 0);
            assert(stopId <= maxStopId);
            return minDistFromStop[stopId];
        }

        // Get the smallest distance from any halting spot to the stop s' that comes after stop s in the route of s, min_{h \in H} dist(h, s').
        int getDistFromClosestSpotToNextStop(const int stopId) {
            assert(stopId >= 0);
            assert(stopId <= maxStopId);
            return minDistToNextStop[stopId];
        }

    private:

        template<typename HaltingSpotsT>
        void initToSearch(const HaltingSpotsT &haltingSpots) {
            toSearch.distanceLabels.init();
            toSearch.queue.clear();
            stopsInToSearchSpace.clear();
            stopsInToSearchSpace.resizeUnderlyingSet(maxStopId + 1);
            minDistFromStop.clear();
            minDistFromStop.resize(maxStopId + 1);

            for (const auto &haltingSpot: haltingSpots) {

                // Introduce rank of tail of halting spot as source for toSearch:
                const auto tailRank = ch.rank(inputGraph.edgeTail(haltingSpot.loc));
                const auto offset = inputGraph.travelTime(haltingSpot.loc);
                if (offset < toSearch.distanceLabels[tailRank][0]) {
                    toSearch.distanceLabels[tailRank][0] = offset;

                    if (!toSearch.queue.contains(tailRank)) {
                        toSearch.queue.insert(tailRank, offset);
                    } else {
                        toSearch.queue.decreaseKey(tailRank, offset);
                    }
                }
            }
        }

        template<typename HaltingSpotsT>
        void initFromSearch(const HaltingSpotsT &haltingSpots) {

            fromSearch.distanceLabels.init();
            fromSearch.queue.clear();
            stopsInFromSearchSpace.clear();
            stopsInFromSearchSpace.resizeUnderlyingSet(maxStopId + 1);
            minDistToNextStop.clear();
            minDistToNextStop.resize(maxStopId + 1);

            for (const auto &haltingSpot: haltingSpots) {
                // Introduce rank of head of halting spot as source for fromSearch:
                const auto headRank = ch.rank(inputGraph.edgeHead(haltingSpot.loc));
                if (!fromSearch.queue.contains(headRank)) {
                    fromSearch.distanceLabels[headRank][0] = 0;
                    fromSearch.queue.insert(headRank, 0);
                } else {
                    assert(fromSearch.distanceLabels[headRank][0] == 0);
                }
            }
        }

        template<typename HaltingSpotsAtExistingStopsT>
        void initHaltingSpotsAtExistingStops(const HaltingSpotsAtExistingStopsT &haltingSpotsAtExistingStops) {
            for (const auto &h: haltingSpotsAtExistingStops) {
                assert(h.stopIndex < routeState.numStopsOf(h.vehId));
                const auto &stopId = routeState.stopIdsFor(h.vehId)[h.stopIndex];
                minDistFromStop[stopId] = 0;
            }
        }

        bool stopToSearch() const {
            return toSearch.queue.empty() || toSearch.queue.minKey() > maxLeeway;
        }

        bool stopFromSearch() const {
            return fromSearch.queue.empty() || fromSearch.queue.minKey() > maxLeeway;
        }

        void scanSourceBucketAtRank(const int v) {
            const auto distVToSpot = toSearch.getDistance(v);
            auto bucket = sourceBuckets.getBucketOf(v);
            for (const auto &entry: bucket) {
                const auto distToSpotViaV = entry.distToTarget + distVToSpot;

#if LOUD_SORTED_REGULAR_BUCKETS
                // Entries in a bucket are ordered by the remaining leeway, i.e. the leeway minus the distance from the
                // stop to this vertex.
                // If the remaining leeway is broken for this vertex, then the remaining leeway is broken for the rest of
                // the entries in this bucket, and we can stop scanning the bucket.
                if (distToSpotViaV > entry.leeway)
                    break;
#endif

                const auto stopId = entry.targetId;
                if (distToSpotViaV < minDistFromStop[stopId]) {
                    minDistFromStop[stopId] = distToSpotViaV;
                    stopsInToSearchSpace.insert(stopId);
                }
            }
        }

        void scanTargetBucketAtRank(const int v) {
            const auto distSpotToV = fromSearch.getDistance(v);
            auto bucket = targetBuckets.getBucketOf(v);
            for (const auto &entry: bucket) {
                const auto distFromSpotViaV = distSpotToV + entry.distToTarget;

#if LOUD_SORTED_REGULAR_BUCKETS
                // Entries in a bucket are ordered by the remaining leeway, i.e. the leeway minus the distance to the
                // stop from this vertex.
                // If the remaining leeway is broken for this vertex, then the remaining leeway is broken for the rest of
                // the entries in this bucket, and we can stop scanning the bucket.
                if (distFromSpotViaV > entry.leeway)
                    break;
#endif

                const auto &prevStopId = routeState.idOfPreviousStopOf(entry.targetId);

                // If the given stop is the first stop in the vehicle's route, there is no previous stop.
                if (prevStopId == INVALID_ID)
                    continue;

                if (distFromSpotViaV < minDistToNextStop[prevStopId]) {
                    minDistToNextStop[prevStopId] = distFromSpotViaV;
                    stopsInFromSearchSpace.insert(prevStopId);
                }
            }
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const SourceBucketsT &sourceBuckets;
        const TargetBucketsT &targetBuckets;
        const int &maxStopId;
        const int &maxLeeway;
        const RouteState &routeState;

        using Search = Dijkstra<typename CH::SearchGraph, typename CH::Weight, LabelSet, dij::NoCriterion, PruningCriterion>;
        Search toSearch;
        Search fromSearch;

        Subset stopsInToSearchSpace;
        Subset stopsInFromSearchSpace;
        TimestampedVector<int> minDistFromStop;
        TimestampedVector<int> minDistToNextStop;

    };

    template<typename InputGraphT, typename CHProviderT, typename SourceBucketsT, typename TargetBucketsT>
    using ClosestHaltingSpotToRegularStopBCHQueryWithStallOnDemand = ClosestHaltingSpotToRegularStopBCHQuery<
            InputGraphT,
            CHProviderT,
            SourceBucketsT,
            TargetBucketsT,
            typename CHQuery<BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>::PruningCriterion
    >;
}


// // Run searches that filter out stops that cannot have feasible pickups associated with them.
//            // Pre-allocate entries for feasible halting spots for all remaining stops.
//            // If ALLOCATE_ENTRIES_FOR_FEASIBLE_HALTING_SPOTS_STATICALLY is not set, the entries for feasible halting spots will
//            // instead be allocated dynamically during the BCH searches.
//            auto &stopsInToSearchSpace = closestHaltingSpotToRegularStopQuery.getStopsInToSearchSpace();
//            auto &stopsInFromSearchSpace = closestHaltingSpotToRegularStopQuery.getStopsInFromSearchSpace();
//
//            // Find minimum distance from each stop to any pickup and from any pickup to each stop. Use these minimum
//            // distances to find out which stops will have any relevant pickups.
//            closestHaltingSpotToRegularStopQuery.run(requestState.pickups, pickupsAtExistingStops);
//            for (const auto &stopId: stopsInToSearchSpace) {
//                if (!stopsInFromSearchSpace.contains(stopId))
//                    continue;
//
//                const auto &minDistToPickup = closestHaltingSpotToRegularStopQuery.getDistToClosestSpotFromStop(stopId);
//                const auto &minDistFromPickup = closestHaltingSpotToRegularStopQuery.getDistFromClosestSpotToNextStop(
//                        stopId);
//                if (minDistToPickup < INFTY && minDistFromPickup < INFTY
//                    && minDistToPickup + minDistFromPickup <= stopData.leewayOfLegStartingAt(stopId)) {
//                    feasibleDistancesForPickups.preallocateEntriesFor(stopId);
//                }
//            }