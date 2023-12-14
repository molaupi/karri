//
// Created by tim on 14.12.23.
//

#pragma once

#include <type_traits>
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Timer.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT, bool SORTED_BUCKETS>
    class LastStopBucketsUpdater {


        using Entry = BucketEntry;

        struct IsDistSmaller {
            bool operator()(const Entry &e1, const Entry &e2) const {
                // In last stop bucket entries .targetId refers to the vehicle's id and .distToTarget is the distance from the
                // vehicles last stop.
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:
        static constexpr bool SORTED_BY_DIST = SORTED_BUCKETS;

        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                SortedBucketContainer<Entry, IsDistSmaller>,
                DynamicBucketContainer<Entry>
        >;

        using BucketType = RouteStateDataType;

    private:

        struct StopWhenDistanceExceeded {
            explicit StopWhenDistanceExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                return distToV[0] > maxDist;
            }

        private:
            const int &maxDist;
        };

        struct GenerateEntry {
            explicit GenerateEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0]);
                bucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
        };

        struct DeleteEntry {
            explicit DeleteEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited,
                                 int &entriesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited),
                      entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0]);
                bool removed = bucketContainer.remove(v, entry);
                ++verticesVisited;
                entriesVisited += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
            int &entriesVisited;
        };

    public:

        LastStopBucketsUpdater(const InputGraphT &inputGraph, const CHEnvT &chEnv, karri::stats::UpdatePerformanceStats &stats)
        : inputGraph(inputGraph),
        ch(chEnv.getCH()),
        searchGraph(ch.upwardGraph()),
        variableBucketContainer(searchGraph.numVertices()),
        fixedBucketContainer(searchGraph.numVertices()),
        variableEntryGenSearch(chEnv.getForwardSearch(GenerateEntry(variableBucketContainer, vehicleId, verticesVisitedInSearch),
                                                      StopWhenDistanceExceeded(maxDetourUntilEndOfServiceTime))),
        variableEntryDelSearch(chEnv.getForwardSearch(DeleteEntry(variableBucketContainer, vehicleId, verticesVisitedInSearch, entriesVisitedInSearch))),
        fixedEntryGenSearch(chEnv.getForwardSearch(GenerateEntry(fixedBucketContainer, vehicleId, verticesVisitedInSearch),
                                                   StopWhenDistanceExceeded(maxDetourUntilEndOfServiceTime))),
        fixedEntryDelSearch(chEnv.getForwardSearch(DeleteEntry(fixedBucketContainer, vehicleId, verticesVisitedInSearch, entriesVisitedInSearch))),
        stats(stats) {}


        const BucketContainer &getBuckets(BucketType type) const {
            return (type == BucketType::VARIABLE) ? variableBucketContainer : fixedBucketContainer;
        }


        void generateBucketEntries(const Vehicle &veh, const int stopIndex, const RouteStateData &routeStateData) {
            assert(stopIndex == routeStateData.numStopsOf(veh.vehicleId) - 1);

            Timer timer;
            const auto stopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            maxDetourUntilEndOfServiceTime =
                    veh.endOfServiceTime - routeStateData.schedDepTimesFor(vehicleId)[routeStateData.numStopsOf(vehicleId) - 1];
            verticesVisitedInSearch = 0;

            if (routeStateData.getTypeOfData() == BucketType::VARIABLE) {
                variableEntryGenSearch.run(ch.rank(stopVertex));
            } else {
                fixedEntryGenSearch.run(ch.rank(stopVertex));
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsGenerateEntriesTime += time;

//        bucketGenLogger << verticesVisitedInSearch << ',' << time << '\n';
        }

        void removeBucketEntries(const Vehicle &veh, const int stopIndex, const RouteStateData &routeStateData) {
            assert(stopIndex >= 0);
            assert(stopIndex < routeStateData.numStopsOf(veh.vehicleId));

            Timer timer;
            const auto stopLoc = routeStateData.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            entriesVisitedInSearch = 0;
            maxDetourUntilEndOfServiceTime = INFTY;
            verticesVisitedInSearch = 0;

            if (routeStateData.getTypeOfData() == RouteStateDataType::VARIABLE) {
                variableEntryDelSearch.run(ch.rank(stopVertex));
            } else {
                fixedEntryDelSearch.run(ch.rank(stopVertex));
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsDeleteEntriesTime += time;

//        bucketDelLogger << verticesVisitedInSearch << ',' << entriesVisitedInSearch << ',' << time << '\n';
        }

    private:

        using GenerateEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateEntry, StopWhenDistanceExceeded>;
        using DeleteEntriesSearch = typename CHEnvT::template UpwardSearch<DeleteEntry>;


        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;

        BucketContainer variableBucketContainer;
        BucketContainer fixedBucketContainer;

        int vehicleId;
        int maxDetourUntilEndOfServiceTime;
        int entriesVisitedInSearch;
        int verticesVisitedInSearch;

        GenerateEntriesSearch variableEntryGenSearch;
        DeleteEntriesSearch variableEntryDelSearch;

        GenerateEntriesSearch fixedEntryGenSearch;
        DeleteEntriesSearch fixedEntryDelSearch;

        karri::stats::UpdatePerformanceStats &stats;

    };

    struct NoOpLastStopBucketsEnvironment {

        inline void generateBucketEntries(const Vehicle &, const int) {/* no op */}

        inline void removeBucketEntries(const Vehicle &, const int) {/* no op */}
    };
}