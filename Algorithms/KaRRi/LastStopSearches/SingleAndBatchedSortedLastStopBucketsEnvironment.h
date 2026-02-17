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

#include "SingleUpdatesSortedLastStopBucketsEnvironment.h"
#include "BatchUpdatesSortedLastStopBucketsEnvironment.h"

namespace karri {

    // Facade around SingleUpdatesSortedLastStopBucketsEnvironment and BatchUpdatesSortedLastStopBucketsEnvironment to have one
    // interface for both single and batched operations on sorted last stop buckets.
    template<typename InputGraphT, typename CHEnvT>
    class SingleAndBatchedSortedLastStopBucketsEnvironment {

    public:
        using BucketContainer = LastStopBucketContainer<LastStopBucketEntry, CompareLastStopBucketEntries, CompareLastStopBucketEntries>;

        SingleAndBatchedSortedLastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                         const RouteState &routeState,
                                         BucketContainer &bucketContainer)
                : singleEnv(inputGraph, chEnv, routeState, bucketContainer),
        batchEnv(inputGraph, chEnv, routeState, bucketContainer) {}

        // Single interface

        void generateIdleBucketEntries(const Vehicle &veh, stats::UpdatePerformanceStats& stats) {
            singleEnv.generateIdleBucketEntries(veh, stats);
        }

        void generateNonIdleBucketEntries(const Vehicle &veh, stats::UpdatePerformanceStats& stats) {
            singleEnv.generateNonIdleBucketEntries(veh, stats);
        }

        // An update to the bucket entries may become necessary in two situations:
        // Case 1: An insertion caused the departure time of a last stop to change (while not changing its location).
        //      The vehicle was non-idle and remains non-idle, so we simply update the entries for the new arrival time
        //      at the corresponding vertex.
        // Case 2: A vehicle has reached its last stop and has become idle.
        //      Its arrival time at each vertex is now dependent on the current time (request time of each request).
        //      We visit every bucket and move the vehicle's entry from the non-idle bucket to the idle bucket for
        //      more precise bucket sorting and pruning.
        // When a vehicle becomes non-idle after being idle, it's last stop always changes, so the update is expressed
        // as a delete operation for the old location and a generate operation for the new location instead.
        void updateBucketEntries(const Vehicle &veh, const int stopIndex, stats::UpdatePerformanceStats& stats) {
            singleEnv.updateBucketEntries(veh, stopIndex, stats);
        }

        void removeIdleBucketEntries(const Vehicle &veh, const int prevLastStopIdx, stats::UpdatePerformanceStats& stats) {
            singleEnv.removeIdleBucketEntries(veh, prevLastStopIdx, stats);
        }

        void removeNonIdleBucketEntries(const Vehicle &veh, const int prevLastStopIdx, stats::UpdatePerformanceStats& stats) {
            singleEnv.removeNonIdleBucketEntries(veh, prevLastStopIdx, stats);
        }

        // Batched interface

        void addIdleBucketEntryInsertions(const int vehId) {
            batchEnv.addIdleBucketEntryInsertions(vehId);
        }

        void addNonIdleBucketEntryInsertions(const int vehId) {
            batchEnv.addNonIdleBucketEntryInsertions(vehId);
        }

        void addIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            batchEnv.addIdleBucketEntryDeletions(vehId, stopIndex);
        }

        void addNonIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            batchEnv.addNonIdleBucketEntryDeletions(vehId, stopIndex);
        }

        // Update the bucket entries for a non-idle vehicle whose last stop has remained the same but the departure
        // time at that stop has changed.
        void addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(const int vehId, const int stopIndex) {
            batchEnv.addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(vehId, stopIndex);
        }

        // Update bucket entries for vehicle that has become idle.
        void addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(const int vehId) {
            batchEnv.addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(vehId);
        }

        int numPendingEntryInsertions() const {
            return batchEnv.numPendingEntryInsertions();
        }

        int numPendingEntryDeletions() const {
            return batchEnv.numPendingEntryDeletions();
        }

        bool noPendingEntryInsertionsOrDeletions() const {
            return batchEnv.noPendingEntryInsertionsOrDeletions();
        }

        void commitEntryInsertionsAndDeletions(LastStopBucketUpdateStats& stats) {
            batchEnv.commitEntryInsertionsAndDeletions(stats);
        }

        bool verifyIdleAndNonIdleBorders() const {
            return batchEnv.verifyIdleAndNonIdleBorders();
        }

        // Implement the last stops at vertices interface.
        std::vector<int> vehiclesWithLastStopAt(const int vertex) const {
            return singleEnv.vehiclesWithLastStopAt(vertex);
        }

    private:

        SingleUpdatesSortedLastStopBucketsEnvironment<InputGraphT, CHEnvT> singleEnv;
        BatchUpdatesSortedLastStopBucketsEnvironment<InputGraphT, CHEnvT> batchEnv;

    };
}