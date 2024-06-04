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

#include "Algorithms/KaRRi/RouteState/RouteStateData.h"
#include "Algorithms/KaRRi/RouteState/RouteStateUpdater.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"
#include "AssignmentFinderResponse.h"

namespace karri {

    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
            typename EllipticBucketsEnvT,
            typename LastStopBucketsEnvT,
            typename FixedRouteStateUpdaterT,
            typename CurVehLocsT,
            typename PathTrackerT,
            typename LoggerT = NullLogger>
    class SystemStateUpdater {

    public:

        SystemStateUpdater(const InputGraphT &inputGraph, const RequestState &requestState,
                           stats::DispatchingPerformanceStats& dispatchingPerformanceStats,
                           stats::OsmRoadCategoryStats &chosenPdLocRoadCategoryStats,
                           const CurVehLocsT &curVehLocs,
                           PathTrackerT &pathTracker, RouteStateData &varRouteStateData,
                           RouteStateData &fixedRouteStateData, FixedRouteStateUpdaterT &fixedRouteStateUpdater,
                           EllipticBucketsEnvT &varEllipticBucketsEnv,
                           LastStopBucketsEnvT &varLastStopBucketsEnv,
                           EllipticBucketsEnvT &fixedEllipticBucketsEnv,
                           LastStopBucketsEnvT &fixedLastStopBucketsEnv)
                : inputGraph(inputGraph),
                  requestState(requestState),
                  dispatchingPerformanceStats(dispatchingPerformanceStats),
                  chosenPdLocRoadCategoryStats(chosenPdLocRoadCategoryStats),
                  curVehLocs(curVehLocs),
                  pathTracker(pathTracker),
                  varRouteState(varRouteStateData),
                  fixedRouteState(fixedRouteStateData),
                  fixedRouteStateUpdater(fixedRouteStateUpdater),
                  varEllipticBucketsEnv(varEllipticBucketsEnv),
                  varLastStopBucketsEnv(varLastStopBucketsEnv),
                  fixedEllipticBucketsEnv(fixedEllipticBucketsEnv),
                  fixedLastStopBucketsEnv(fixedLastStopBucketsEnv),
                  bestAssignmentsLogger(LogManager<LoggerT>::getLogger("bestassignments.csv",
                                                                       "request_id, "
                                                                       "request_time, "
                                                                       "direct_od_dist, "
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


        void insertBestAssignment(int &pickupStopId, int &dropoffStopId, const BestAsgn& bestAsgn) {
            Timer timer;

            if (bestAsgn.cost() < INFTY && !bestAsgn.asgn().isValid()) {
                // If there is a non-INFTY best cost but no assignment is set, we can assume that direct walking is the
                // best option for this rider.
                pickupStopId = -1;
                dropoffStopId = -1;
                return;
            }

            const auto &asgn = bestAsgn.asgn();
            chosenPdLocRoadCategoryStats.incCountForCat(inputGraph.osmRoadCategory(asgn.pickup->loc));
            chosenPdLocRoadCategoryStats.incCountForCat(inputGraph.osmRoadCategory(asgn.dropoff->loc));
            assert(asgn.vehicle != nullptr);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = varRouteState.numStopsOf(vehId);
            const auto depTimeAtLastStopBefore = varRouteState.schedDepTimesFor(vehId)[numStopsBefore - 1];

            timer.restart();
            auto [pickupIndex, dropoffIndex] = RouteStateUpdater::insertAssignment(varRouteState, asgn,
                                                                                   requestState);
            if (pickupIndex == 0) {
                fixedRouteStateUpdater.updateForNewPickupAtCurrentStop(fixedRouteState,
                                                                       requestState.originalRequest.requestId, vehId);
                if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY)
                    fixedEllipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(vehId);
            }
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            dispatchingPerformanceStats.updateStats.updateRoutesTime += routeUpdateTime;

            updateVarBucketStateForInsertedAssignment(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore);

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting.
            if (asgn.pickupStopIdx == 0 && numStopsBefore > 1 && varRouteState.schedDepTimesFor(vehId)[0] <
                                                                 requestState.originalRequest.requestTime) {
                createIntermediateStopStopAtCurrentLocationForReroute(*asgn.vehicle,
                                                                      requestState.originalRequest.requestTime);
                ++pickupIndex;
                ++dropoffIndex;
            }

            pickupStopId = varRouteState.stopIdsFor(vehId)[pickupIndex];
            dropoffStopId = varRouteState.stopIdsFor(vehId)[dropoffIndex];

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, dropoffStopId);
        }

        void notifyStopStarted(const Vehicle &veh) {

            // Update buckets and route state
            varEllipticBucketsEnv.deleteSourceBucketEntries(veh.vehicleId, 0);
            varEllipticBucketsEnv.deleteTargetBucketEntries(veh.vehicleId, 1);
            fixedEllipticBucketsEnv.deleteSourceBucketEntries(veh.vehicleId, 0);
            StopIdManager::markIdUnused(varRouteState.stopIdsFor(veh.vehicleId)[0]);
            RouteStateUpdater::removeStartOfCurrentLeg(varRouteState, veh.vehicleId);

            // If vehicle has become idle, update last stop bucket entries
            if (varRouteState.numStopsOf(veh.vehicleId) == 1) {
                varLastStopBucketsEnv.updateBucketEntries(veh, 0);
            }

            // Update fixed route
            const int depTimeAtLastFixedStopBefore = fixedRouteState.schedDepTimesFor(
                    veh.vehicleId)[fixedRouteState.numStopsOf(veh.vehicleId) - 1];
            bool wasReachedStopFixed = false;
            const auto indicesOfNewFixedStops = fixedRouteStateUpdater.updateForReachedStop(fixedRouteState,
                                                                                            veh.vehicleId,
                                                                                            wasReachedStopFixed);
            updateFixedBucketStateForReachedStop(veh, indicesOfNewFixedStops, wasReachedStopFixed,
                                                 depTimeAtLastFixedStopBefore);
        }


        void notifyStopCompleted(const Vehicle &veh) {
            pathTracker.logCompletedStop(veh);
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            KASSERT(varRouteState.numStopsOf(vehId) == 1);
            KASSERT(fixedRouteState.numStopsOf(vehId) == 1);

            varLastStopBucketsEnv.removeIdleBucketEntries(veh, 0);
            fixedLastStopBucketsEnv.removeIdleBucketEntries(veh, 0);

            StopIdManager::markIdUnused(varRouteState.stopIdsFor(vehId)[0]);
            RouteStateUpdater::removeStartOfCurrentLeg(varRouteState, vehId);
            RouteStateUpdater::removeStartOfCurrentLeg(fixedRouteState, vehId);
        }


        template<typename AssignmentFindeResponseT>
        void writeBestAssignmentToLogger(const AssignmentFindeResponseT& asgnFinderResponse) {
            bestAssignmentsLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", ";

            if (asgnFinderResponse.getBestAsgnType() == BestAsgnType::NO_ASSIGNMENT) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
                return;
            }

            if (asgnFinderResponse.getBestAsgnType() == BestAsgnType::NOT_USING_VEHICLE) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, true, "
                                      << asgnFinderResponse.getBestCost() << "\n";
                return;
            }


            const auto &asgn = asgnFinderResponse.getBestRegularAsgn().asgn();
            const auto &vehId = asgn.vehicle->vehicleId;
            const auto &numStops = varRouteState.numStopsOf(vehId);
            using time_utils::getVehDepTimeAtStopForRequest;
            const auto &vehDepTimeBeforePickup = getVehDepTimeAtStopForRequest(vehId, asgn.pickupStopIdx,
                                                                               requestState, varRouteState);
            const auto &vehDepTimeBeforeDropoff = getVehDepTimeAtStopForRequest(vehId, asgn.dropoffStopIdx,
                                                                                requestState, varRouteState);
            bestAssignmentsLogger
                    << vehId << ", "
                    << asgn.pickupStopIdx << ", "
                    << asgn.dropoffStopIdx << ", "
                    << asgn.distToPickup << ", "
                    << asgn.distFromPickup << ", "
                    << asgn.distToDropoff << ", "
                    << asgn.distFromDropoff << ", "
                    << asgn.pickup->id << ", "
                    << asgn.pickup->walkingDist << ", "
                    << asgn.dropoff->id << ", "
                    << asgn.dropoff->walkingDist << ", "
                    << numStops << ", "
                    << vehDepTimeBeforePickup << ", "
                    << vehDepTimeBeforeDropoff << ", "
                    << "false, "
                    << asgnFinderResponse.getBestCost() << "\n";
        }

        void writePerformanceLogs() {
            overallPerfLogger << requestState.originalRequest.requestId << ", "
                              << dispatchingPerformanceStats.getLoggerRow() << "\n";
            initializationPerfLogger << requestState.originalRequest.requestId << ", "
                                     << dispatchingPerformanceStats.initializationStats.getLoggerRow() << "\n";
            ellipticBchPerfLogger << requestState.originalRequest.requestId << ", "
                                  << dispatchingPerformanceStats.ellipticBchStats.getLoggerRow() << "\n";
            pdDistancesPerfLogger << requestState.originalRequest.requestId << ", "
                                  << dispatchingPerformanceStats.pdDistancesStats.getLoggerRow() << "\n";
            ordPerfLogger << requestState.originalRequest.requestId << ", "
                          << dispatchingPerformanceStats.ordAssignmentsStats.getLoggerRow() << "\n";
            pbnsPerfLogger << requestState.originalRequest.requestId << ", "
                           << dispatchingPerformanceStats.pbnsAssignmentsStats.getLoggerRow() << "\n";
            palsPerfLogger << requestState.originalRequest.requestId << ", "
                           << dispatchingPerformanceStats.palsAssignmentsStats.getLoggerRow() << "\n";
            dalsPerfLogger << requestState.originalRequest.requestId << ", "
                           << dispatchingPerformanceStats.dalsAssignmentsStats.getLoggerRow() << "\n";
            updatePerfLogger << requestState.originalRequest.requestId << ", "
                             << dispatchingPerformanceStats.updateStats.getLoggerRow() << "\n";
        }

    private:

        // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
        // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
        // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
        // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
        // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
        void createIntermediateStopStopAtCurrentLocationForReroute(const Vehicle &veh, const int now) {
            KASSERT(curVehLocs.knowsCurrentLocationOf(veh.vehicleId));
            auto loc = curVehLocs.getCurrentLocationOf(veh.vehicleId);
            LIGHT_KASSERT(loc.depTimeAtHead >= now);
            RouteStateUpdater::createIntermediateStopForReroute(varRouteState, veh.vehicleId, loc.location, now,
                                                                loc.depTimeAtHead);
            varEllipticBucketsEnv.generateSourceBucketEntries(veh.vehicleId, 1);
        }


        // Updates the bucket state (elliptic buckets, last stop buckets, lastStopsAtVertices structure) given an
        // assignment that has already been inserted into varRouteState as well as the stop index of the pickup and
        // dropoff after the insertion.
        void updateVarBucketStateForInsertedAssignment(const Assignment &asgn,
                                                       const int pickupIndex, const int dropoffIndex,
                                                       const int depTimeAtLastStopBefore) {

            const int vehId = asgn.vehicle->vehicleId;
            const auto numStopsAfter = varRouteState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            std::vector<int> insertedStopsIndices;
            if (!pickupAtExistingStop)
                insertedStopsIndices.push_back(pickupIndex);
            if (!dropoffAtExistingStop)
                insertedStopsIndices.push_back(dropoffIndex);
            generateBucketStateForNewStops(varRouteState, varEllipticBucketsEnv, varLastStopBucketsEnv,
                                           *asgn.vehicle, insertedStopsIndices);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                varEllipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(asgn.vehicle->vehicleId);
                varEllipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(asgn.vehicle->vehicleId);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const auto depTimeAtLastStopAfter = varRouteState.schedDepTimesFor(vehId)[numStopsAfter - 1];
            const bool depTimeAtLastChanged = depTimeAtLastStopAfter != depTimeAtLastStopBefore;

            if ((dropoffAtExistingStop || dropoffIndex < numStopsAfter - 1) && depTimeAtLastChanged) {
                varLastStopBucketsEnv.updateBucketEntries(*asgn.vehicle, numStopsAfter - 1);
            }
        }

        void updateFixedBucketStateForReachedStop(const Vehicle &veh, const std::vector<int> &indicesOfNewStops,
                                                  const bool wasReachedStopFixed, const int depTimeAtLastStopBefore) {
            const auto &vehId = veh.vehicleId;
            if (wasReachedStopFixed) {
                fixedEllipticBucketsEnv.deleteTargetBucketEntries(vehId, 0);
            } else {
                fixedEllipticBucketsEnv.generateSourceBucketEntries(vehId, 0);
            }

            if (fixedRouteState.numStopsOf(veh.vehicleId) == 1) {
                KASSERT(indicesOfNewStops.empty());
                fixedLastStopBucketsEnv.updateBucketEntries(veh, 0);
                return;
            }

            generateBucketStateForNewStops(fixedRouteState, fixedEllipticBucketsEnv, fixedLastStopBucketsEnv, veh,
                                           indicesOfNewStops);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                fixedEllipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(vehId);
                fixedEllipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(vehId);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const auto &numStops = fixedRouteState.numStopsOf(vehId);
            const auto depTimeAtLastStopAfter = fixedRouteState.schedDepTimesFor(vehId)[numStops - 1];
            const bool depTimeAtLastChanged = depTimeAtLastStopAfter != depTimeAtLastStopBefore;

            if ((indicesOfNewStops.empty() || indicesOfNewStops.back() < numStops - 1) && depTimeAtLastChanged) {
                fixedLastStopBucketsEnv.updateBucketEntries(veh, numStops - 1);
            }
        }

        // Expects sorted vector of indices of new stops
        void generateBucketStateForNewStops(RouteStateData &routeState,
                                            EllipticBucketsEnvT &ellipticBucketsEnv,
                                            LastStopBucketsEnvT &lastStopBucketsEnv,
                                            const Vehicle &veh,
                                            const std::vector<int> &indicesOfNewStops) {
            if (indicesOfNewStops.empty())
                return;
            const auto &numStops = routeState.numStopsOf(veh.vehicleId);

            for (const auto &idx: indicesOfNewStops) {
                ellipticBucketsEnv.generateTargetBucketEntries(veh.vehicleId, idx);
                if (idx < numStops - 1)
                    ellipticBucketsEnv.generateSourceBucketEntries(veh.vehicleId, idx);
            }

            // If the last stop has not changed, we do not need to update its bucket entries.
            if (indicesOfNewStops.back() < numStops - 1)
                return;

            // If last stop has changed, the former last stop has the largest index not present in indicesOfNewStops.
            int i = indicesOfNewStops.size() - 1;
            for (; i > 0; --i) {
                if (indicesOfNewStops[i - 1] < indicesOfNewStops[i] - 1)
                    break;
            }
            const int idxOfFormerLastStop = indicesOfNewStops[i] - 1;

            // If last stop has changed, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            KASSERT(idxOfFormerLastStop >= 0 && idxOfFormerLastStop < numStops - 1);
            ellipticBucketsEnv.generateSourceBucketEntries(veh.vehicleId, idxOfFormerLastStop);

            // Remove last stop bucket entries for former last stop and generate them for dropoff
            if (idxOfFormerLastStop == 0) {
                KASSERT(numStops - indicesOfNewStops.size() == 1);
                lastStopBucketsEnv.removeIdleBucketEntries(veh, idxOfFormerLastStop);
            } else {
                KASSERT(numStops - indicesOfNewStops.size() > 1);
                lastStopBucketsEnv.removeNonIdleBucketEntries(veh, idxOfFormerLastStop);
            }
            lastStopBucketsEnv.generateNonIdleBucketEntries(veh);
        }

        const InputGraphT &inputGraph;
        const RequestState &requestState;
        stats::DispatchingPerformanceStats& dispatchingPerformanceStats;
        stats::OsmRoadCategoryStats& chosenPdLocRoadCategoryStats;

        const CurVehLocsT &curVehLocs;
        PathTrackerT &pathTracker;

        // Route state
        RouteStateData &varRouteState;
        RouteStateData &fixedRouteState;
        FixedRouteStateUpdaterT &fixedRouteStateUpdater;

        // Bucket state
        EllipticBucketsEnvT &varEllipticBucketsEnv;
        LastStopBucketsEnvT &varLastStopBucketsEnv;
        EllipticBucketsEnvT &fixedEllipticBucketsEnv;
        LastStopBucketsEnvT &fixedLastStopBucketsEnv;

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