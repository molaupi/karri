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

#include <cassert>
#include <vector>
#include "Tools/Workarounds.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Utilities/IteratorRange.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/Stats/LastStopBucketUpdateStats.h"

namespace karri {

    // Stand-in for last stop bucket environment when no last stop buckets are needed.
    // Can only provide information on the last stops at a vertex itself.
    template<typename InputGraphT>
    class OnlyLastStopsAtVerticesBucketSubstitute {

        using VehiclesOrderedByLastStop = std::vector<int>;
        using VehiclesOrderedByLastStopIt = VehiclesOrderedByLastStop::const_iterator;

    public:

        OnlyLastStopsAtVerticesBucketSubstitute(const InputGraphT &inputGraph, const RouteState &routeState,
                                                const Fleet &fleet)
                : inputGraph(inputGraph), routeState(routeState), fleetSize(static_cast<int>(fleet.size())), fleet(fleet),
                  firstLastStopAtVertex(inputGraph.numVertices()), vehiclesOrderedByLastStop() {
        }

        int numLastStopsAtVertex(const int vertex) const {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            const auto range = firstLastStopAtVertex[vertex];
            return range.end - range.start;
        }

        void generateIdleBucketEntries(const Vehicle &veh, stats::UpdatePerformanceStats& stats) {
            Timer timer;
            const int v = inputGraph.edgeHead(
                    routeState.stopLocationsFor(veh.vehicleId)[routeState.numStopsOf(veh.vehicleId) - 1]);
            insertLastStopAt(v, veh.vehicleId);
            stats.lastStopBucketsGenerateEntriesTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void generateNonIdleBucketEntries(const Vehicle &veh, stats::UpdatePerformanceStats& stats) {
            Timer timer;
            const int v = inputGraph.edgeHead(
                    routeState.stopLocationsFor(veh.vehicleId)[routeState.numStopsOf(veh.vehicleId) - 1]);
            insertLastStopAt(v, veh.vehicleId);
            stats.lastStopBucketsGenerateEntriesTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void updateBucketEntries(const Vehicle &, const int, stats::UpdatePerformanceStats&) {
            // no op since last stop of vehicle does not change
        }

        void removeIdleBucketEntries(const Vehicle &veh, const int prevLastStopIdx, stats::UpdatePerformanceStats& stats) {
            Timer timer;
            const int v = inputGraph.edgeHead(routeState.stopLocationsFor(veh.vehicleId)[prevLastStopIdx]);
            removeLastStopAt(v, veh.vehicleId);
            stats.lastStopBucketsDeleteEntriesTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void removeNonIdleBucketEntries(const Vehicle &veh, const int prevLastStopIdx, stats::UpdatePerformanceStats& stats) {
            Timer timer;
            const int v = inputGraph.edgeHead(routeState.stopLocationsFor(veh.vehicleId)[prevLastStopIdx]);
            removeLastStopAt(v, veh.vehicleId);
            stats.lastStopBucketsDeleteEntriesTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        // Batched interface

        void addIdleBucketEntryInsertions(const int vehId) {
            vehiclesToGenerate.local().push_back(vehId);
        }

        void addNonIdleBucketEntryInsertions(const int vehId) {
            vehiclesToGenerate.local().push_back(vehId);
        }

        void addIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            vehiclesToDelete.local().push_back({vehId, stopIndex});
        }

        void addNonIdleBucketEntryDeletions(const int vehId, const int stopIndex) {
            vehiclesToDelete.local().push_back({vehId, stopIndex});
        }

        void addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(const int, const int) {
            // No op as no entries to update
        }


        void addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(const int ) {
            // No op as no entries to update
        }

        int numPendingEntryInsertions() const {
            int sum = 0;
            for (const auto &local: vehiclesToGenerate)
                sum += local.size();
            return sum;
        }

        int numPendingEntryDeletions() const {
            int sum = 0;
            for (const auto &local: vehiclesToDelete)
                sum += local.size();
            return sum;
        }

        bool noPendingEntryInsertionsOrDeletions() const {
            return numPendingEntryInsertions() == 0 && numPendingEntryDeletions() == 0;
        }

        void commitEntryInsertionsAndDeletions(LastStopBucketUpdateStats&) {
            for (const auto &local: vehiclesToGenerate) {
                for (const auto &vehId: local) {
                    const int v = inputGraph.edgeHead(
                            routeState.stopLocationsFor(vehId)[routeState.numStopsOf(vehId) - 1]);
                    insertLastStopAt(v, vehId);
                }
            }

            for (const auto &local: vehiclesToDelete) {
                for (const auto &[vehId, stopIndex]: local) {
                    const int v = inputGraph.edgeHead(routeState.stopLocationsFor(vehId)[stopIndex]);
                    removeLastStopAt(v, vehId);
                }
            }
        }

        bool verifyIdleAndNonIdleBorders() const {
            return true; // No borders in this substitute
        }

        ConstantVectorRange<int> vehiclesWithLastStopAt(const int vertex) const {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            const auto range = firstLastStopAtVertex[vertex];
            return {vehiclesOrderedByLastStop.begin() + range.start,
                    vehiclesOrderedByLastStop.begin() + range.end};
        }

    private:

        // Inserts the last stop of vehicle with given ID at vertex in the data structure.
        void insertLastStopAt(const int vertex, const int vehId) {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            KASSERT(vehId >= 0 && vehId < fleetSize);
            dynamic_ragged2d::insertion(vertex, vehId, firstLastStopAtVertex, vehiclesOrderedByLastStop);
        }

        // Removes the last stop of vehicle with given ID at vertex from the data structure.
        void removeLastStopAt(const int vertex, const int vehId) {
            KASSERT(vertex >= 0 && vertex < inputGraph.numVertices());
            KASSERT(vehId >= 0 && vehId < fleetSize);
            const auto start = firstLastStopAtVertex[vertex].start;
            const auto end = firstLastStopAtVertex[vertex].end;
            KASSERT(end > start);
            for (auto i = start; i < end; ++i)
                if (vehiclesOrderedByLastStop[i] == vehId) {
                    dynamic_ragged2d::removal(vertex, i - start, firstLastStopAtVertex, vehiclesOrderedByLastStop);
                    break;
                }
            KASSERT(firstLastStopAtVertex[vertex].end - firstLastStopAtVertex[vertex].start == end - start - 1);
        }


        const InputGraphT &inputGraph;
        const RouteState &routeState;
        const int fleetSize;
        const Fleet &fleet;
        std::vector<dynamic_ragged2d::ValueBlockPosition> firstLastStopAtVertex;
        std::vector<int> vehiclesOrderedByLastStop;

        using EntryInsertionVecT = parallel::scalable_vector<int>;
        struct Deletion {
            int vehId = INVALID_ID;
            int stopIndex = INVALID_INDEX;
        };
        using EntryDeletionVecT = parallel::scalable_vector<Deletion>;
        tbb::enumerable_thread_specific<EntryInsertionVecT> vehiclesToGenerate;
            tbb::enumerable_thread_specific<EntryDeletionVecT> vehiclesToDelete;

    };
}
