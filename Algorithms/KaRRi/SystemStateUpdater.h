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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"

namespace karri {

    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
            typename EllipticBucketsEnvT,
            typename LastStopBucketsEnvT,
            typename VehicleLocatorT,
            typename PathTrackerT,
            typename LoggerT = NullLogger>
    class SystemStateUpdater {

    public:

        SystemStateUpdater(const InputGraphT &inputGraph,
                           VehicleLocatorT &vehicleLocator,
                           PathTrackerT &pathTracker,
                           RouteState &routeState, EllipticBucketsEnvT &ellipticBucketsEnv,
                           LastStopBucketsEnvT &lastStopBucketsEnv)
                : inputGraph(inputGraph),
                  vehicleLocator(vehicleLocator),
                  pathTracker(pathTracker),
                  routeState(routeState),
                  ellipticBucketsEnv(ellipticBucketsEnv),
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
                                                         std::string(stats::DispatchingPerformanceStats::LOGGER_COLS))),
                  initializationPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::InitializationPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::InitializationPerformanceStats::LOGGER_COLS))),
                  ellipticBchPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::EllipticBCHPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::EllipticBCHPerformanceStats::LOGGER_COLS))),
                  pdDistancesPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PDDistancesPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::PDDistancesPerformanceStats::LOGGER_COLS))),
                  ordPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::OrdAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::OrdAssignmentsPerformanceStats::LOGGER_COLS))),
                  pbnsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PbnsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::PbnsAssignmentsPerformanceStats::LOGGER_COLS))),
                  palsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::PalsAssignmentsPerformanceStats::LOGGER_COLS))),
                  dalsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::DalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::DalsAssignmentsPerformanceStats::LOGGER_COLS))),
                  updatePerfLogger(LogManager<LoggerT>::getLogger(stats::UpdatePerformanceStats::LOGGER_NAME,
                                                                  "request_id, " +
                                                                  std::string(
                                                                          stats::UpdatePerformanceStats::LOGGER_COLS))) {}


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
            updateEllipticBuckets(asgn, pickupIndex, dropoffIndex, stats);
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


        template<typename AssignmentFinderResponsesT, typename StatssT>
        void insertBatchOfBestAssignments(const AssignmentFinderResponsesT &asgnFinderResponses, StatssT &statss,
                                          int64_t &performEllipticBucketEntryInsertionsTime, int64_t &updateLeewaysTime,
                                          int &numEllipticBucketEntryInsertions) {

            KASSERT(std::distance(asgnFinderResponses.begin(), asgnFinderResponses.end()) == std::distance(statss.begin(), statss.end()));

            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                const auto asgnFinderResponse = *respIt;
                auto &stats = statsIt->updateStats;

                KASSERT(!asgnFinderResponse.isNotUsingVehicleBest());
                Timer timer;

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

                timer.restart();
                auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, asgnFinderResponse);
                const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
                stats.updateRoutesTime += routeUpdateTime;


                // TODO: Memorize only which stops to generate elliptic bucket entries for, then do searches for finding
                //  specific entry insertions in parallel.
                updateEllipticBuckets(asgn, pickupIndex, dropoffIndex, stats);
                updateLastStopBuckets(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore, stats);

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


            numEllipticBucketEntryInsertions = numPendingEllipticBucketEntryInsertions();

            Timer timer;
            commitEllipticBucketEntryInsertions(); // Batched update to elliptic buckets for new entries.
            performEllipticBucketEntryInsertionsTime = timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            // Update leeway in elliptic bucket entries of stops in routes of affected vehicles.
            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                updateLeewaysInSourceBucketsForAllStopsOf(
                        respIt->getBestAssignment().vehicle->vehicleId, statsIt->updateStats);
                updateLeewaysInTargetBucketsForAllStopsOf(
                        respIt->getBestAssignment().vehicle->vehicleId, statsIt->updateStats);
            }
            updateLeewaysTime = timer.elapsed<std::chrono::nanoseconds>();

            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                writeBestAssignmentToLogger(*respIt);
                writePerformanceLogs(*respIt, *statsIt);
            }
        }

        void commitEllipticBucketEntryInsertions() {
            ellipticBucketsEnv.commitEntryInsertions();
        }

        int numPendingEllipticBucketEntryInsertions() {
            return ellipticBucketsEnv.numPendingEntryInsertions();
        }

        void commitEllipticBucketEntryDeletions() {
            ellipticBucketsEnv.commitEntryDeletions();
        }

        int numPendingEllipticBucketEntryDeletions() {
            return ellipticBucketsEnv.numPendingEntryDeletions();
        }

        void updateLeewaysInSourceBucketsForAllStopsOf(const int vehId, stats::UpdatePerformanceStats &stats) {
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(vehId, stats);
            }
        }

        void updateLeewaysInTargetBucketsForAllStopsOf(const int vehId, stats::UpdatePerformanceStats &stats) {
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(vehId, stats);
            }
        }

        void notifyStopStarted(const Vehicle &veh) {

            // Update buckets and route state
            ellipticBucketsEnv.addSourceBucketEntryDeletions(veh, 0);
            ellipticBucketsEnv.addTargetBucketEntryDeletions(veh, 1);
            routeState.removeStartOfCurrentLeg(veh.vehicleId);

            // If vehicle has become idle, update last stop bucket entries
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                int64_t updateTimePlaceholder = 0;
                lastStopBucketsEnv.updateBucketEntries(veh, 0, updateTimePlaceholder);
            }
        }


        bool noPendingEllipticBucketEntryInsertionsOrDeletions() {
            return ellipticBucketsEnv.noPendingEntryInsertionsOrDeletions();
        }

        void notifyStopCompleted(const Vehicle &veh) {
            pathTracker.logCompletedStop(veh);
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            KASSERT(routeState.numStopsOf(vehId) == 1);

            stats::UpdatePerformanceStats finalRemovalStatsPlaceholder;
            lastStopBucketsEnv.removeIdleBucketEntries(veh, 0, finalRemovalStatsPlaceholder);

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
                                                          stats::UpdatePerformanceStats &stats) {
            LIGHT_KASSERT(loc.depTimeAtHead >= now);
            routeState.createIntermediateStopForReroute(veh.vehicleId, loc.location, now, loc.depTimeAtHead);
            ellipticBucketsEnv.addSourceBucketEntryInsertions(veh, 1, stats);
//            stopIdsToGenerateSourceEntriesFor.push_back(routeState.stopIdsFor(veh.vehicleId)[1]);
        }

        void updateEllipticBuckets(const Assignment &asgn, const int pickupIndex, const int dropoffIndex,
                                   stats::UpdatePerformanceStats &stats) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStops = routeState.numStopsOf(vehId);
//            const auto stopIds = routeState.stopIdsFor(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
//                stopIdsToGenerateSourceEntriesFor.push_back(stopIds[pickupIndex]);
//                stopIdsToGenerateTargetEntriesFor.push_back(stopIds[pickupIndex]);
                ellipticBucketsEnv.addTargetBucketEntryInsertions(*asgn.vehicle, pickupIndex, stats);
                ellipticBucketsEnv.addSourceBucketEntryInsertions(*asgn.vehicle, pickupIndex, stats);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.addTargetBucketEntryInsertions(*asgn.vehicle, dropoffIndex, stats);
//            stopIdsToGenerateTargetEntriesFor.push_back(stopIds[dropoffIndex]);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (dropoffIndex < numStops - 1) {
                ellipticBucketsEnv.addSourceBucketEntryInsertions(*asgn.vehicle, dropoffIndex, stats);
//                stopIdsToGenerateSourceEntriesFor.push_back(stopIds[dropoffIndex]);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
            const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
            ellipticBucketsEnv.addSourceBucketEntryInsertions(*asgn.vehicle, formerLastStopIdx, stats);
//            stopIdsToGenerateSourceEntriesFor.push_back(stopIds[formerLastStopIdx]);
        }

        void updateLastStopBuckets(const Assignment &asgn, const int pickupIndex, const int dropoffIndex,
                                   const int depTimeAtLastStopBefore, stats::UpdatePerformanceStats &stats) {

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
                    lastStopBucketsEnv.removeIdleBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
                } else {
                    lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
                }
                lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.vehicle, stats);
            } else if (depTimeAtLastChanged) {
                // If last stop does not change but departure time at last changes, update last stop bucket entries
                // accordingly.
                lastStopBucketsEnv.updateBucketEntries(*asgn.vehicle, numStops - 1,
                                                       stats.lastStopBucketsUpdateEntriesTime);
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        PathTrackerT &pathTracker;

        // Route state
        RouteState &routeState;

        // Bucket state
        EllipticBucketsEnvT &ellipticBucketsEnv;
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

    };
}