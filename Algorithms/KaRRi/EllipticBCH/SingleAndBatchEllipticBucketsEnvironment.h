#pragma once
#include "SingleUpdatesEllipticBucketsEnvironment.h"

namespace karri {
    // Facade around SingleUpdatesEllipticBucketsEnvironment and BatchUpdatesEllipticBucketsEnvironment to have one
    // interface for both single and batched operations on elliptic buckets.
    template<typename InputGraphT, typename CHEnvT, typename EllipticBucketsT>
    class SingleAndBatchEllipticBucketsEnvironment {

    public:

        using BucketContainer = EllipticBucketsT;

        SingleAndBatchEllipticBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                                 const RouteState &routeState,
                                                 BucketContainer &sourceBuckets,
                                                 BucketContainer &targetBuckets)
            : singleEnv(inputGraph, chEnv, routeState, sourceBuckets, targetBuckets),
              batchEnv(inputGraph, chEnv, routeState, sourceBuckets, targetBuckets) {
        }

        // Single buckets interface

        void generateSourceBucketEntries(const Vehicle &veh, const int stopIndex,
                                         stats::UpdatePerformanceStats &stats) {
            singleEnv.generateSourceBucketEntries(veh, stopIndex, stats);
        }

        void generateTargetBucketEntries(const Vehicle &veh, const int stopIndex,
                                         stats::UpdatePerformanceStats &stats) {
            singleEnv.generateTargetBucketEntries(veh, stopIndex, stats);
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const Vehicle &veh, stats::UpdatePerformanceStats &stats) {
            singleEnv.updateLeewayInSourceBucketsForAllStopsOf(veh, stats);
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const Vehicle &veh, stats::UpdatePerformanceStats &stats) {
            singleEnv.updateLeewayInTargetBucketsForAllStopsOf(veh, stats);
        }

        void deleteSourceBucketEntries(const Vehicle &veh, const int stopIndex, stats::UpdatePerformanceStats &stats) {
            singleEnv.deleteSourceBucketEntries(veh, stopIndex, stats);
        }

        void deleteTargetBucketEntries(const Vehicle &veh, const int stopIndex, stats::UpdatePerformanceStats &stats) {
            singleEnv.deleteTargetBucketEntries(veh, stopIndex, stats);
        }

        using RankWithEntry = SingleUpdatesEllipticBucketsEnvironment<InputGraphT, CHEnvT, EllipticBucketsT>::RankWithEntry;

        std::vector<RankWithEntry> enumerateRanksWithSourceBucketEntries(const int vehId, const int stopIndex, Subset& searchSpaceHelper) const {
            return singleEnv.enumerateRanksWithSourceBucketEntries(vehId, stopIndex, searchSpaceHelper);
        }

        std::vector<RankWithEntry> enumerateRanksWithTargetBucketEntries(const int vehId, const int stopIndex, Subset &searchSpaceHelper) const {
            return singleEnv.enumerateRanksWithTargetBucketEntries(vehId, stopIndex, searchSpaceHelper);
        }

        // Batched buckets interface
        size_t totalNumSourceEntries() const {
            return batchEnv.totalNumSourceEntries();
        }

        size_t totalNumTargetEntries() const {
            return batchEnv.totalNumTargetEntries();
        }

        bool noPendingEntryInsertionsOrDeletions() const {
            return batchEnv.noPendingEntryInsertionsOrDeletions();
        }

        int numPendingEntryInsertions() const {
            return batchEnv.numPendingEntryInsertions();
        }

        int numPendingEntryDeletions() const {
            return batchEnv.numPendingEntryDeletions();
        }

        // Adds entry insertions to source buckets for the given stop index of the given vehicle.
        // Memorizes insertions to be performed in next batch update of the buckets.
        // Safe to call in a parallel environment, insertions are stored in thread local storage.
        void addSourceBucketEntryInsertions(const int vehId, const int stopIndex
                                            //, karri::stats::UpdatePerformanceStats &stats
        ) {
            batchEnv.addSourceBucketEntryInsertions(vehId, stopIndex
                                                    //, stats
            );
        }

        // Adds entry insertions to target buckets for the given stop index of the given vehicle.
        // Memorizes insertions to be performed in next batch update of the buckets.
        // Safe to call in a parallel environment, insertions are stored in thread local storage.
        void addTargetBucketEntryInsertions(const int vehId, const int stopIndex
                                            //                                            , karri::stats::UpdatePerformanceStats &stats
        ) {
            batchEnv.addTargetBucketEntryInsertions(vehId, stopIndex
                                                    //                                            , stats
            );
        }

        void updateLeewayInSourceBucketsForAllStopsOf(const int vehId, karri::stats::UpdatePerformanceStats &stats) {
            batchEnv.updateLeewayInSourceBucketsForAllStopsOf(vehId, stats);
        }

        // Adds leeways updates for entries in source buckets of the given vehicles.
        // Memorizes vertices that have source bucket entries for the vehicle to be performed in next batch leeway
        // update of the buckets.
        void addSourceBucketLeewayUpdates(const Subset &vehIds, int64_t &numEntriesScanned,
                                          int64_t &numVerticesVisited) {
            batchEnv.addSourceBucketLeewayUpdates(vehIds, numEntriesScanned, numVerticesVisited);
        }

        void commitSourceBucketLeewayUpdatesParallel(const Subset &vehIds) {
            batchEnv.commitSourceBucketLeewayUpdatesParallel(vehIds);
        }

        void updateLeewayInTargetBucketsForAllStopsOf(const int vehId, karri::stats::UpdatePerformanceStats &stats) {
            batchEnv.updateLeewayInTargetBucketsForAllStopsOf(vehId, stats);
        }

        // Adds leeways updates for entries in target buckets of the given vehicles.
        // Memorizes vertices that have target bucket entries for the vehicle to be performed in next batch leeway
        // update of the buckets.
        void addTargetBucketLeewayUpdates(const Subset &vehIds, int64_t &numEntriesScanned,
                                          int64_t &numVerticesVisited) {
            batchEnv.addTargetBucketLeewayUpdates(vehIds, numEntriesScanned, numVerticesVisited);
        }

        void commitTargetBucketLeewayUpdatesParallel(const Subset &vehIds) {
            batchEnv.commitTargetBucketLeewayUpdatesParallel(vehIds);
        }


        void addSourceBucketEntryDeletions(const Vehicle &veh, const int stopIndex) {
            batchEnv.addSourceBucketEntryDeletions(veh, stopIndex);
        }

        void addTargetBucketEntryDeletions(const Vehicle &veh, const int stopIndex) {
            batchEnv.addTargetBucketEntryDeletions(veh, stopIndex);
        }

        void commitEntryInsertions() {
            batchEnv.commitEntryInsertions();
        }

        void commitSourceBucketEntryInsertions() {
            batchEnv.commitSourceBucketEntryInsertions();
        }

        void commitTargetBucketEntryInsertions() {
            batchEnv.commitTargetBucketEntryInsertions();
        }

        void commitEntryDeletions() {
            batchEnv.commitEntryDeletions();
        }

        void commitSourceBucketEntryDeletions() {
            batchEnv.commitSourceBucketEntryDeletions();
        }

        void commitTargetBucketEntryDeletions() {
            batchEnv.commitTargetBucketEntryDeletions();
        }

    private:
        SingleUpdatesEllipticBucketsEnvironment<InputGraphT, CHEnvT, EllipticBucketsT> singleEnv;
        BatchUpdatesEllipticBucketsEnvironment<InputGraphT, CHEnvT, EllipticBucketsT> batchEnv;
    };
}
