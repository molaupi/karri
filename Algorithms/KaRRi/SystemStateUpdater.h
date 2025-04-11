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

#include <tbb/parallel_for.h>
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"
#include "Algorithms/KaRRi/Stats/LastStopBucketUpdateStats.h"
#include "Algorithms/KaRRi/EllipticBCH/OrdinaryStopsAtLocations.h"

namespace karri {

    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
            typename OrdinaryStopsRPHASTSelectionT,
            typename LastStopBucketsEnvT,
            typename VehicleLocatorT,
            typename PathTrackerT,
            typename LoggerT = NullLogger>
    class SystemStateUpdater {

    public:

        SystemStateUpdater(const InputGraphT &inputGraph,
                           const Fleet &fleet,
                           VehicleLocatorT &vehicleLocator,
                           PathTrackerT &pathTracker,
                           RouteState &routeState,
                           OrdinaryStopsAtLocations &ordinaryStopsAtLocations,
                           OrdinaryStopsRPHASTSelectionT& ordinaryStopsRphastSelection,
                           LastStopBucketsEnvT &lastStopBucketsEnv)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  vehicleLocator(vehicleLocator),
                  pathTracker(pathTracker),
                  routeState(routeState),
                  ordinaryStopsAtLocations(ordinaryStopsAtLocations),
                  ordinaryStopsRphastSelection(ordinaryStopsRphastSelection),
                  lastStopBucketsEnv(lastStopBucketsEnv),
                  bestAssignmentsLogger(LogManager<LoggerT>::getLogger("bestassignments.csv",
                                                                       "request_id, "
                                                                       "request_time, "
                                                                       "direct_od_dist, "
                                                                       "dispatching_time, "
                                                                       "vehicle_id, "
                                                                       "pickup_insertion_point, "
                                                                       "dropoff_insertion_point, "
                                                                       "dist_to_pickup, "
                                                                       "dist_from_pickup, "
                                                                       "dist_to_dropoff, "
                                                                       "dist_from_dropoff, "
                                                                       "pickup_id, "
                                                                       "pickup_walking_dist, "
                                                                       "dropoff_id, "
                                                                       "dropoff_walking_dist, "
                                                                       "num_stops, "
                                                                       "veh_dep_time_at_stop_before_pickup, "
                                                                       "veh_dep_time_at_stop_before_dropoff, "
                                                                       "not_using_vehicle, "
                                                                       "cost\n")),
                  overallPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::DispatchingPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::DispatchingPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  initializationPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::InitializationPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::InitializationPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  ellipticBchPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::EllipticBCHPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::EllipticBCHPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  pdDistancesPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PDDistancesPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::PDDistancesPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  ordPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::OrdAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::OrdAssignmentsPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  pbnsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PbnsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::PbnsAssignmentsPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  palsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::PalsAssignmentsPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  dalsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::DalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::DalsAssignmentsPerformanceStats::LOGGER_COLS) +
                                                         "\n")),
                  updatePerfLogger(LogManager<LoggerT>::getLogger(stats::UpdatePerformanceStats::LOGGER_NAME,
                                                                  "request_id, " +
                                                                  std::string(
                                                                          stats::UpdatePerformanceStats::LOGGER_COLS) +
                                                                  "\n")),
                  batchInsertLogger(LogManager<LoggerT>::getLogger("batch_insert_stats.csv", "occurence_time,"
                                                                                             "iteration,"
                                                                                             "num_requests,"
                                                                                             "update_route_state_time,"
                                                                                             "ell_entry_insertions_find_insertions_time,"
                                                                                             "ell_entry_insertions_num_insertions,"
                                                                                             "ell_entry_insertions_perform_insertions_time,"
                                                                                             "last_stop_find_insertions_and_deletions_time,"
                                                                                             "last_stop_num_insertions_and_deletions,"
                                                                                             "last_stop_perform_insertions_and_deletions_time,"
                                                                                             "num_ell_source_entries_after_insertions,"
                                                                                             "num_ell_target_entries_after_insertions,"
                                                                                             "update_leeways_num_entries_scanned_source,"
                                                                                             "update_leeways_num_entries_scanned_target,"
                                                                                             "update_leeways_time,"
                                                                                             "total_time\n")),
                  lastStopBucketUpdateLogger(LogManager<LoggerT>::getLogger("last_stop_bucket_update_stats.csv",
                                                                            LastStopBucketUpdateStats::getLoggerCols() +
                                                                            "\n")) {}


        void insertBestAssignment(const RequestState &requestState, stats::UpdatePerformanceStats &stats) {
            KASSERT(!requestState.isNotUsingVehicleBest());
            Timer timer;

            const auto &asgn = requestState.getBestAssignment();
            KASSERT(asgn.vehicle != nullptr);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = routeState.numStopsOf(vehId);
            const auto depTimeAtLastStopBefore = routeState.schedDepTimesFor(vehId)[numStopsBefore - 1];

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
            // current location before changing the route, and later add the intermediate stop after changing the route.
            bool rerouteVehicle = asgn.pickupStopIdx == 0 && numStopsBefore > 1 &&
                                  routeState.schedDepTimesFor(vehId)[0] < requestState.dispatchingTime;
            VehicleLocation loc;
            if (rerouteVehicle) {
                loc = vehicleLocator.computeCurrentLocation(*asgn.vehicle, requestState.dispatchingTime);
                // If vehicle is currently already at edge that constitutes the next stop, we do not have to reroute
                // and create an intermediate stop (as the intermediate stop would be at the same location).
                rerouteVehicle = routeState.stopLocationsFor(vehId)[1] != loc.location;
            }

            timer.restart();
            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, requestState);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.updateRoutesTime += routeUpdateTime;


            // TODO: Memorize only which stops to generate elliptic bucket entries for, then do searches for finding
            //  specific entry insertions in parallel.
            updateStopsAtLocations(asgn, pickupIndex, dropoffIndex);
            updateLastStopBuckets(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore, stats);

            if (rerouteVehicle) {
                createIntermediateStopAtCurrentLocationForReroute(*asgn.vehicle, requestState.dispatchingTime, loc,
                                                                  stats);
                ++pickupIndex;
                ++dropoffIndex;
            }

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(routeState.stopIdsFor(vehId)[pickupIndex],
                                                          routeState.stopIdsFor(vehId)[dropoffIndex]);
        }


        void initializeForNewBatch() {
            ordinaryStopsRphastSelection.runSelectionPhaseForOrdinaryStops();
        }

        template<typename AssignmentFinderResponsesT, typename StatssT>
        void insertBatchOfBestAssignments(const AssignmentFinderResponsesT &asgnFinderResponses, StatssT &statss,
                                          const int now, const int iteration) {

            KASSERT(std::distance(asgnFinderResponses.begin(), asgnFinderResponses.end()) ==
                    std::distance(statss.begin(), statss.end()));

            // Stops that require elliptic bucket entry changes
            std::vector<int> stopIdsToGenerateSourceEntriesFor;
            std::vector<int> stopIdsToGenerateTargetEntriesFor;

            // Stops that require last stop bucket entry changes
            std::vector<int> stopIdsToRemoveIdleEntriesFor;
            std::vector<int> stopIdsToRemoveNonIdleEntriesFor;
            std::vector<int> vehIdsToGenerateNonIdleEntriesFor;
            std::vector<int> stopIdsToUpdateLastStopEntriesFor;

//            int64_t updateLastStopBucketsTime = 0;
            Timer internalRouteStateUpdateTimer;
//            Timer lastStopBucketsTimer;
            Timer timer;
            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                const auto asgnFinderResponse = *respIt;
                auto &stats = statsIt->updateStats;

                KASSERT(!asgnFinderResponse.isNotUsingVehicleBest());

                const auto &asgn = asgnFinderResponse.getBestAssignment();
                KASSERT(asgn.vehicle != nullptr);

                const auto vehId = asgn.vehicle->vehicleId;
                const auto numStopsBefore = routeState.numStopsOf(vehId);
                const auto depTimeAtLastStopBefore = routeState.schedDepTimesFor(vehId)[numStopsBefore - 1];

                // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
                // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
                // current location before changing the route, and later add the intermediate stop after changing the route.
                bool rerouteVehicle = asgn.pickupStopIdx == 0 && numStopsBefore > 1 &&
                                      routeState.schedDepTimesFor(vehId)[0] < asgnFinderResponse.dispatchingTime;
                VehicleLocation loc;
                if (rerouteVehicle) {
                    loc = vehicleLocator.computeCurrentLocation(*asgn.vehicle, asgnFinderResponse.dispatchingTime);
                    // If vehicle is currently already at edge that constitutes the next stop, we do not have to reroute
                    // and create an intermediate stop (as the intermediate stop would be at the same location).
                    rerouteVehicle = routeState.stopLocationsFor(vehId)[1] != loc.location;
                }

                internalRouteStateUpdateTimer.restart();
                auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, asgnFinderResponse);
                const auto routeUpdateTime = internalRouteStateUpdateTimer.elapsed<std::chrono::nanoseconds>();
                stats.updateRoutesTime += routeUpdateTime;

                updateStopsAtLocations(asgn, pickupIndex, dropoffIndex);

//                lastStopBucketsTimer.restart();
                updateLastStopBuckets(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore, stats,
                                      stopIdsToRemoveIdleEntriesFor,
                                      stopIdsToRemoveNonIdleEntriesFor, vehIdsToGenerateNonIdleEntriesFor,
                                      stopIdsToUpdateLastStopEntriesFor);
//                updateLastStopBucketsTime += lastStopBucketsTimer.elapsed<std::chrono::nanoseconds>();

                if (rerouteVehicle) {
                    createIntermediateStopAtCurrentLocationForReroute(*asgn.vehicle, asgnFinderResponse.dispatchingTime,
                                                                      loc, stats);
                    ++pickupIndex;
                    ++dropoffIndex;
                }

                // Register the inserted pickup and dropoff with the path data
                pathTracker.registerPdEventsForBestAssignment(routeState.stopIdsFor(vehId)[pickupIndex],
                                                              routeState.stopIdsFor(vehId)[dropoffIndex]);
            }
            const auto updateRouteStateTime = timer.elapsed<std::chrono::nanoseconds>();

            LastStopBucketUpdateStats lastStopBucketUpdateStats;

            timer.restart();
            // Find all last stop bucket entry deletions and insertions necessary for assignments
            tbb::parallel_for(0ul, stopIdsToRemoveIdleEntriesFor.size(), [&](const size_t i) {
                const auto &stopId = stopIdsToRemoveIdleEntriesFor[i];
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopIndex = routeState.stopPositionOf(stopId);
                lastStopBucketsEnv.addIdleBucketEntryDeletions(vehId, stopIndex);
            });
            tbb::parallel_for(0ul, stopIdsToRemoveNonIdleEntriesFor.size(), [&](const size_t i) {
                const auto &stopId = stopIdsToRemoveNonIdleEntriesFor[i];
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopIndex = routeState.stopPositionOf(stopId);
                lastStopBucketsEnv.addNonIdleBucketEntryDeletions(vehId, stopIndex);
            });
            tbb::parallel_for(0ul, vehIdsToGenerateNonIdleEntriesFor.size(), [&](const size_t i) {
                const auto &vehId = vehIdsToGenerateNonIdleEntriesFor[i];
                lastStopBucketsEnv.addNonIdleBucketEntryInsertions(vehId);
            });
            tbb::parallel_for(0ul, stopIdsToUpdateLastStopEntriesFor.size(), [&](const size_t i) {
                const auto &stopId = stopIdsToUpdateLastStopEntriesFor[i];
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopIndex = routeState.stopPositionOf(stopId);
                lastStopBucketsEnv.addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(vehId, stopIndex);
            });
            const auto findLastStopBucketEntryInsertionsAndDeletionsTime = timer.elapsed<std::chrono::nanoseconds>();
            const auto numLastStopBucketEntryInsertionsAndDeletions =
                    lastStopBucketsEnv.numPendingEntryInsertions() + lastStopBucketsEnv.numPendingEntryDeletions();
            lastStopBucketUpdateStats.findInsertionsAndDeletionsTime = findLastStopBucketEntryInsertionsAndDeletionsTime;
            lastStopBucketUpdateStats.numInsertions = lastStopBucketsEnv.numPendingEntryInsertions();
            lastStopBucketUpdateStats.numDeletions = lastStopBucketsEnv.numPendingEntryDeletions();

            timer.restart();
            lastStopBucketsEnv.commitEntryInsertionsAndDeletions(
                    lastStopBucketUpdateStats); // Batched update to last stop bucket entries.
            const auto performLastStopBucketEntryInsertionsAndDeletionsTime = timer.elapsed<std::chrono::nanoseconds>();

            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                writeBestAssignmentToLogger(*respIt);
                writePerformanceLogs(*respIt, *statsIt);
            }

            const auto totalTime =
                    updateRouteStateTime + findLastStopBucketEntryInsertionsAndDeletionsTime +
                    performLastStopBucketEntryInsertionsAndDeletionsTime;

            // occurence_time,"
            // "iteration,"
            // "num_requests,"
            // "update_route_state_time,"
            // "ell_entry_insertions_find_insertions_time,"
            // "ell_entry_insertions_num_insertions,"
            // "ell_entry_insertions_perform_insertions_time,"
            // "last_stop_find_insertions_and_deletions_time,"
            // "last_stop_num_insertions_and_deletions,"
            // "last_stop_perform_insertions_and_deletions_time,"
            // "num_ell_source_entries_after_insertions,"
            // "num_ell_target_entries_after_insertions,"
            // "update_leeways_num_entries_scanned_source,"
            // "update_leeways_num_entries_scanned_target,"
            // "update_leeways_time,"
            // "total_time\n")),
            batchInsertLogger << now << ","
                              << iteration << ","
                              << asgnFinderResponses.size() << ","
                              << updateRouteStateTime << ","
                              << 0 << "," // findEllipticBucketEntryInsertionsTime << ","
                              << 0 << "," // numEllipticBucketEntryInsertions << ","
                              << 0 << "," // performEllipticBucketEntryInsertionsTime << ","
                              << findLastStopBucketEntryInsertionsAndDeletionsTime << ","
                              << numLastStopBucketEntryInsertionsAndDeletions << ","
                              << performLastStopBucketEntryInsertionsAndDeletionsTime << ","
                              << 0 << "," // ellipticBucketsEnv.totalNumSourceEntries() << ","
                              << 0 << "," // ellipticBucketsEnv.totalNumTargetEntries() << ","
                              << 0 << "," // totalNumEntriesScannedForSourceUpdates << ","
                              << 0 << "," // totalNumEntriesScannedForTargetUpdates << ","
                              << 0 << "," // updateLeewaysTime << ","
                              << totalTime << "\n";

            lastStopBucketUpdateLogger << lastStopBucketUpdateStats.getLoggerRow() << "\n";
        }

        void commitPendingLastStopBucketEntryInsertionsAndDeletions() {
            LastStopBucketUpdateStats stats;
            lastStopBucketsEnv.commitEntryInsertionsAndDeletions(stats);
        }

        void notifyStopStarted(const Vehicle &veh) {

            // Update buckets and route state
            const auto stopId = routeState.stopIdsFor(veh.vehicleId)[0];
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[0];
            ordinaryStopsAtLocations.removeStopAtLocation(stopId, stopLoc);
            routeState.removeStartOfCurrentLeg(veh.vehicleId);

            // If vehicle has become idle, update last stop bucket entries
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                lastStopBucketsEnv.addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(veh.vehicleId);
            }
        }

        bool noPendingLastStopBucketEntryInsertionsOrDeletions() {
            return lastStopBucketsEnv.noPendingEntryInsertionsOrDeletions();
        }

        void notifyStopCompleted(const Vehicle &veh) {
            pathTracker.logCompletedStop(veh);
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            KASSERT(routeState.numStopsOf(vehId) == 1);

//            stats::UpdatePerformanceStats finalRemovalStatsPlaceholder;
//            lastStopBucketsEnv.removeIdleBucketEntries(veh, 0, finalRemovalStatsPlaceholder);
            lastStopBucketsEnv.addIdleBucketEntryDeletions(veh.vehicleId, 0);

            routeState.removeStartOfCurrentLeg(vehId);
        }


        void writeBestAssignmentToLogger(const RequestState &requestState) {
            bestAssignmentsLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", "
                    << requestState.dispatchingTime << ", ";

            if (requestState.getBestCost() == INFTY) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
                return;
            }

            if (requestState.isNotUsingVehicleBest()) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, true, "
                                      << requestState.getBestCost() << "\n";
                return;
            }

            const auto &bestAsgn = requestState.getBestAssignment();

            const auto &vehId = bestAsgn.vehicle->vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            using time_utils::getVehDepTimeAtStopForRequest;
            const auto &vehDepTimeBeforePickup = getVehDepTimeAtStopForRequest(vehId, bestAsgn.pickupStopIdx,
                                                                               requestState, routeState);
            const auto &vehDepTimeBeforeDropoff = getVehDepTimeAtStopForRequest(vehId, bestAsgn.dropoffStopIdx,
                                                                                requestState, routeState);
            bestAssignmentsLogger
                    << vehId << ", "
                    << bestAsgn.pickupStopIdx << ", "
                    << bestAsgn.dropoffStopIdx << ", "
                    << bestAsgn.distToPickup << ", "
                    << bestAsgn.distFromPickup << ", "
                    << bestAsgn.distToDropoff << ", "
                    << bestAsgn.distFromDropoff << ", "
                    << bestAsgn.pickup.id << ", "
                    << bestAsgn.pickup.walkingDist << ", "
                    << bestAsgn.dropoff.id << ", "
                    << bestAsgn.dropoff.walkingDist << ", "
                    << numStops << ", "
                    << vehDepTimeBeforePickup << ", "
                    << vehDepTimeBeforeDropoff << ", "
                    << "false, "
                    << requestState.getBestCost() << "\n";
        }

        void writePerformanceLogs(const RequestState &requestState, stats::DispatchingPerformanceStats &stats) {
            overallPerfLogger << requestState.originalRequest.requestId << ", "
                              << stats.getLoggerRow() << "\n";
            initializationPerfLogger << requestState.originalRequest.requestId << ", "
                                     << stats.initializationStats.getLoggerRow() << "\n";
            ellipticBchPerfLogger << requestState.originalRequest.requestId << ", "
                                  << stats.ellipticBchStats.getLoggerRow() << "\n";
            pdDistancesPerfLogger << requestState.originalRequest.requestId << ", "
                                  << stats.pdDistancesStats.getLoggerRow() << "\n";
            ordPerfLogger << requestState.originalRequest.requestId << ", "
                          << stats.ordAssignmentsStats.getLoggerRow() << "\n";
            pbnsPerfLogger << requestState.originalRequest.requestId << ", "
                           << stats.pbnsAssignmentsStats.getLoggerRow() << "\n";
            palsPerfLogger << requestState.originalRequest.requestId << ", "
                           << stats.palsAssignmentsStats.getLoggerRow() << "\n";
            dalsPerfLogger << requestState.originalRequest.requestId << ", "
                           << stats.dalsAssignmentsStats.getLoggerRow() << "\n";
            updatePerfLogger << requestState.originalRequest.requestId << ", "
                             << stats.updateStats.getLoggerRow() << "\n";
        }

    private:

        // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
        // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
        // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
        // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
        // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
        void
        createIntermediateStopAtCurrentLocationForReroute(const Vehicle &veh, const int now, const VehicleLocation &loc,
                                                          stats::UpdatePerformanceStats &) {
            LIGHT_KASSERT(loc.depTimeAtHead >= now);
            routeState.createIntermediateStopForReroute(veh.vehicleId, loc.location, now, loc.depTimeAtHead);
            ordinaryStopsAtLocations.addStopAtLocation(routeState.stopIdsFor(veh.vehicleId)[1], loc.location);
        }

        void updateStopsAtLocations(const Assignment &asgn, const int pickupIndex, const int dropoffIndex) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto stopIds = routeState.stopIdsFor(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            if (!pickupAtExistingStop) {
                ordinaryStopsAtLocations.addStopAtLocation(stopIds[pickupIndex], stopLocations[pickupIndex]);
            }
            if (!dropoffAtExistingStop && dropoffIndex < routeState.numStopsOf(vehId) - 1) {
                ordinaryStopsAtLocations.addStopAtLocation(stopIds[dropoffIndex], stopLocations[dropoffIndex]);
            }

            // If last stop changed (dropoff is the new last stop), former last stop becomes ordinary stop
            const auto numStops = routeState.numStopsOf(vehId);
            if (dropoffIndex == numStops - 1 && !dropoffAtExistingStop) {
                const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
                const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
                ordinaryStopsAtLocations.addStopAtLocation(stopIds[formerLastStopIdx],
                                                           stopLocations[formerLastStopIdx]);
            }
        }

        void updateLastStopBuckets(const Assignment &asgn, const int pickupIndex, const int dropoffIndex,
                                   const int depTimeAtLastStopBefore, stats::UpdatePerformanceStats &,
                                   std::vector<int> &stopIdsToRemoveIdleEntriesFor,
                                   std::vector<int> &stopIdsToRemoveNonIdleEntriesFor,
                                   std::vector<int> &vehIdsToGenerateNonIdleEntriesFor,
                                   std::vector<int> &stopIdsToUpdateEntriesFor) {

            const auto vehId = asgn.vehicle->vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            const auto depTimeAtLastStopAfter = routeState.schedDepTimesFor(vehId)[numStops - 1];
            const bool depTimeAtLastChanged = depTimeAtLastStopAfter != depTimeAtLastStopBefore;

            if (dropoffIndex == numStops - 1 && !dropoffAtExistingStop) {
                // If dropoff is the new last stop, remove entries for former last stop, and generate for dropoff
                const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
                const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
                if (formerLastStopIdx == 0) {
                    stopIdsToRemoveIdleEntriesFor.push_back(routeState.stopIdsFor(vehId)[formerLastStopIdx]);
//                    lastStopBucketsEnv.removeIdleBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
                } else {
                    stopIdsToRemoveNonIdleEntriesFor.push_back(routeState.stopIdsFor(vehId)[formerLastStopIdx]);
//                    lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
                }
                vehIdsToGenerateNonIdleEntriesFor.push_back(vehId);
//                lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.vehicle, stats);
            } else if (depTimeAtLastChanged) {
                // If last stop does not change but departure time at last changes, update last stop bucket entries
                // accordingly.
                stopIdsToUpdateEntriesFor.push_back(routeState.stopIdsFor(vehId)[numStops - 1]);
//                lastStopBucketsEnv.updateBucketEntries(*asgn.vehicle, numStops - 1,
//                                                       stats.lastStopBucketsUpdateEntriesTime);
            }
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        VehicleLocatorT &vehicleLocator;
        PathTrackerT &pathTracker;

        // Route state
        RouteState &routeState;

        OrdinaryStopsAtLocations &ordinaryStopsAtLocations;

        // RPHAST state for ordinary stops
        OrdinaryStopsRPHASTSelectionT& ordinaryStopsRphastSelection;

        // Bucket state for last stops
        LastStopBucketsEnvT &lastStopBucketsEnv;


        // Performance Loggers
        LoggerT &bestAssignmentsLogger;
        LoggerT &overallPerfLogger;
        LoggerT &initializationPerfLogger;
        LoggerT &ellipticBchPerfLogger;
        LoggerT &pdDistancesPerfLogger;
        LoggerT &ordPerfLogger;
        LoggerT &pbnsPerfLogger;
        LoggerT &palsPerfLogger;
        LoggerT &dalsPerfLogger;
        LoggerT &updatePerfLogger;

        LoggerT &batchInsertLogger;

        LoggerT &lastStopBucketUpdateLogger;
    };
}