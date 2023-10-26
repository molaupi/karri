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

#include <type_traits>
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Timer.h"
#include "Algorithms/Buckets/LastStopBucketContainer.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT, bool SORTED_BUCKETS>
    class SortedLastStopBucketsEnvironment {

        struct LastStopEntry {
            int vehicleId = INVALID_ID;
            int distOrArrTime = INFTY; // For idle vehicle: dist from last stop to vertex; For non-idle vehicle: arrival time at vertex
        };

        struct CompareEntries {
            bool operator()(const LastStopEntry &e1, const LastStopEntry &e2) const {
                return e1.distOrArrTime < e2.distOrArrTime;
            }
        };

    public:
        static constexpr bool SORTED_BY_DIST = SORTED_BUCKETS;

        using BucketContainer = LastStopBucketContainer<LastStopEntry, CompareEntries, CompareEntries>;

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

        struct GenerateIdleEntry {
            explicit GenerateIdleEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0]);
                bucketContainer.insertIdle(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
        };

        struct GenerateNonIdleEntry {
            explicit GenerateNonIdleEntry(BucketContainer &bucketContainer, int &curVehId,
                                          int &depTimeAtLastStopOfCurVeh, int &verticesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId),
                      depTimeAtLastStopOfCurVeh(depTimeAtLastStopOfCurVeh), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, depTimeAtLastStopOfCurVeh + distToV[0]);
                bucketContainer.insertNonIdle(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &depTimeAtLastStopOfCurVeh;
            int &verticesVisited;
        };

        struct UpdateNewScheduleOfNonIdleVehicle {
            explicit UpdateNewScheduleOfNonIdleVehicle(BucketContainer &bucketContainer, int &curVehId,
                                                       int &depTimeAtLastStopOfCurVeh,
                                                       int &verticesVisited, int &entriesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId),
                      depTimeAtLastStopOfCurVeh(depTimeAtLastStopOfCurVeh),
                      verticesVisited(verticesVisited), entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                ++verticesVisited;
                bool removed = bucketContainer.removeNonIdle(v, curVehId);
                entriesVisited += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
                if (!removed)
                    return true; // Prune if no entry for the vehicle was found at this vertex
                auto newEntry = BucketEntry(curVehId, depTimeAtLastStopOfCurVeh + distToV[0]);
                bucketContainer.insertIdle(v, newEntry);
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &depTimeAtLastStopOfCurVeh;
            int &verticesVisited;
            int &entriesVisited;
        };

        struct UpdateForVehicleHasBecomeIdle {
            explicit UpdateForVehicleHasBecomeIdle(BucketContainer &bucketContainer, int &curVehId,
                                                   int &depTimeAtLastStopOfCurVeh,
                                                   int &verticesVisited, int &entriesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId),
                      depTimeAtLastStopOfCurVeh(depTimeAtLastStopOfCurVeh),
                      verticesVisited(verticesVisited), entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                ++verticesVisited;
                auto oldEntry = BucketEntry(curVehId, depTimeAtLastStopOfCurVeh + distToV[0]);
                bool removed = bucketContainer.removeNonIdle(v, oldEntry);
                entriesVisited += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
                if (!removed)
                    return true; // Prune if no entry for the vehicle was found at this vertex
                auto newEntry = BucketEntry(curVehId, distToV[0]);
                bucketContainer.insertIdle(v, newEntry);
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &depTimeAtLastStopOfCurVeh;
            int &verticesVisited;
            int &entriesVisited;
        };

        struct DeleteIdleEntry {
            explicit DeleteIdleEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited,
                                     int &entriesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited),
                      entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curVehId, distToV[0]);
                bool removed = bucketContainer.removeIdle(v, entry);
                ++verticesVisited;
                entriesVisited += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
                return !removed; // Prune if no entry for the vehicle was found at this vertex
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
            int &entriesVisited;
        };

        struct DeleteNonIdleEntry {
            explicit DeleteNonIdleEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited,
                                        int &entriesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId),
                      verticesVisited(verticesVisited), entriesVisited(entriesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
                // todo: it would be possible to use binary search here if we had the old dep time
                bool removed = bucketContainer.removeNonIdle(v, curVehId);
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

        SortedLastStopBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                         const RouteState &routeState,
                                         karri::stats::UpdatePerformanceStats &stats)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.upwardGraph()),
                  routeState(routeState),
                  bucketContainer(searchGraph.numVertices()),
                  idleEntryGenSearch(
                          chEnv.getForwardSearch(GenerateIdleEntry(bucketContainer, vehicleId, verticesVisitedInSearch),
                                                 StopWhenDistanceExceeded(maxDetourUntilEndOfServiceTime))),
                  nonIdleEntryGenSearch(chEnv.getForwardSearch(
                          GenerateNonIdleEntry(bucketContainer, vehicleId, depTimeOfVehAtLastStop,
                                               verticesVisitedInSearch),
                          StopWhenDistanceExceeded(maxDetourUntilEndOfServiceTime))),
                  updateNewScheduleOfNonIdleVehicleSearch(chEnv.getForwardSearch(
                          UpdateNewScheduleOfNonIdleVehicle(bucketContainer, vehicleId, depTimeOfVehAtLastStop,
                                                        verticesVisitedInSearch, entriesVisitedInSearch))),
                  updateForVehicleHasBecomeIdleSearch(chEnv.getForwardSearch(
                          UpdateForVehicleHasBecomeIdle(bucketContainer, vehicleId, depTimeOfVehAtLastStop,
                                                        verticesVisitedInSearch, entriesVisitedInSearch))),
                  idleEntryDelSearch(chEnv.getForwardSearch(
                          DeleteIdleEntry(bucketContainer, vehicleId, verticesVisitedInSearch,
                                          entriesVisitedInSearch))),
                  nonIdleEntryDelSearch(chEnv.getForwardSearch(
                          DeleteNonIdleEntry(bucketContainer, vehicleId, depTimeOfVehAtLastStop,
                                             verticesVisitedInSearch, entriesVisitedInSearch))),
                  stats(stats) {}


        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }


        void generateBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex == routeState.numStopsOf(veh.vehicleId) - 1);

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);
            const bool isIdle = numStops == 1;
            depTimeOfVehAtLastStop = routeState.schedDepTimesFor(veh.vehicleId)[numStops - 1];

            maxDetourUntilEndOfServiceTime = veh.endOfServiceTime - depTimeOfVehAtLastStop;
            verticesVisitedInSearch = 0;
            if (isIdle) {
                idleEntryGenSearch.run(ch.rank(stopVertex));
            } else {
                nonIdleEntryGenSearch.run(ch.rank(stopVertex));
            }
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsGenerateEntriesTime += time;

//        bucketGenLogger << verticesVisitedInSearch << ',' << time << '\n';
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
        void updateBucketEntries(const Vehicle &veh, const int stopIndex) {

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);
            const bool isIdle = numStops == 1;
            depTimeOfVehAtLastStop = routeState.schedDepTimesFor(veh.vehicleId)[numStops - 1];

            entriesVisitedInSearch = 0;
            maxDetourUntilEndOfServiceTime = INFTY;
            verticesVisitedInSearch = 0;

            if (isIdle) {
                updateForVehicleHasBecomeIdleSearch.run(ch.rank(stopVertex));
            } else {
                updateNewScheduleOfNonIdleVehicleSearch.run(ch.rank(stopVertex));
            }
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsUpdateEntriesTime += time;
        }

        void removeBucketEntries(const Vehicle &veh, const int stopIndex) {
            assert(stopIndex >= 0);
            assert(stopIndex < routeState.numStopsOf(veh.vehicleId));

            Timer timer;
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[stopIndex];
            const auto stopVertex = inputGraph.edgeHead(stopLoc);

            vehicleId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);
            const bool isIdle = numStops == 1;
            depTimeOfVehAtLastStop = routeState.schedDepTimesFor(veh.vehicleId)[numStops - 1];

            entriesVisitedInSearch = 0;
            maxDetourUntilEndOfServiceTime = INFTY;
            verticesVisitedInSearch = 0;
            if (isIdle) {
                idleEntryDelSearch.run(ch.rank(stopVertex));
            } else {
                nonIdleEntryDelSearch.run(ch.rank(stopVertex));
            }
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsDeleteEntriesTime += time;

//        bucketDelLogger << verticesVisitedInSearch << ',' << entriesVisitedInSearch << ',' << time << '\n';
        }

    private:

        using GenerateIdleEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateIdleEntry, StopWhenDistanceExceeded>;
        using GenerateNonIdleEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateNonIdleEntry, StopWhenDistanceExceeded>;
        using UpdateForVehicleHasBecomeIdleSearch = typename CHEnvT::template UpwardSearch<UpdateForVehicleHasBecomeIdle>;
        using UpdateNewScheduleOfNonIdleVehicleSearch = typename CHEnvT::template UpwardSearch<UpdateNewScheduleOfNonIdleVehicle>;
        using DeleteIdleEntriesSearch = typename CHEnvT::template UpwardSearch<DeleteIdleEntry>;
        using DeleteNonIdleEntriesSearch = typename CHEnvT::template UpwardSearch<DeleteNonIdleEntry>;


        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;
        const RouteState &routeState;

        BucketContainer bucketContainer;

        int vehicleId;
        int depTimeOfVehAtLastStop;
        int maxDetourUntilEndOfServiceTime;
        int entriesVisitedInSearch;
        int verticesVisitedInSearch;

        GenerateIdleEntriesSearch idleEntryGenSearch;
        GenerateNonIdleEntriesSearch nonIdleEntryGenSearch;
        UpdateForVehicleHasBecomeIdleSearch updateForVehicleHasBecomeIdleSearch;
        UpdateNewScheduleOfNonIdleVehicleSearch updateNewScheduleOfNonIdleVehicleSearch;
        DeleteIdleEntriesSearch idleEntryDelSearch;
        DeleteNonIdleEntriesSearch nonIdleEntryDelSearch;

        karri::stats::UpdatePerformanceStats &stats;

    };

    struct NoOpLastStopBucketsEnvironment {

        inline void generateBucketEntries(const Vehicle &, const int) {/* no op */}

        inline void removeBucketEntries(const Vehicle &, const int) {/* no op */}
    };
}