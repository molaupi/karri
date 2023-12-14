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

#include "Algorithms/KaRRi/LastStopSearches/LastStopsAtVertices.h"
#include "PathTracker.h"
#include "Algorithms/KaRRi/RouteState/RouteStateUpdater.h"
#include "Algorithms/KaRRi/RouteState/FixedRouteStateUpdater.h"

namespace karri {

    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
            typename EllipticBucketsUpdaterT,
            typename LastStopBucketsUpdaterT,
            typename CurVehLocsT,
            typename PathTrackerT,
            typename RouteStateUpdaterT,
            typename FixedRouteStateUpdaterT,
            typename CostCalculatorT,
            typename LoggerT = NullLogger>
    class SystemStateUpdater {

    public:

        SystemStateUpdater(const InputGraphT &inputGraph, RequestState<CostCalculatorT> &requestState,
                           const InputConfig &inputConfig, const CurVehLocsT &curVehLocs,
                           PathTrackerT &pathTracker,
                           RouteStateUpdaterT &variableUpdater, RouteStateData &variableData,
                           FixedRouteStateUpdaterT &fixedUpdater, RouteStateData &fixedData,
                           EllipticBucketsUpdaterT &ellipticBucketsEnv,
                           LastStopBucketsUpdaterT &lastStopBucketsEnv,
                           LastStopsAtVertices &lastStopsAtVertices)
                : inputGraph(inputGraph),
                  requestState(requestState),
                  inputConfig(inputConfig),
                  curVehLocs(curVehLocs),
                  pathTracker(pathTracker),
                  variableUpdater(variableUpdater),
                  fixedUpdater(fixedUpdater),
                  variableData(variableData),
                  fixedData(fixedData),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  lastStopBucketsEnv(lastStopBucketsEnv),
                  lastStopsAtVertices(lastStopsAtVertices),
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


        void insertBestAssignment(int &pickupStopId, int &dropoffStopId) {
            Timer timer;

            if (requestState.isNotUsingVehicleBest()) {
                pickupStopId = -1;
                dropoffStopId = -1;
                return;
            }

            auto &asgn = requestState.getBestAssignment();
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.pickup->loc));
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.dropoff->loc));
            assert(asgn.vehicle != nullptr);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = variableData.numStopsOf(vehId);

            timer.restart();
            const auto [pickupIndex, dropoffIndex] = variableUpdater.insert(asgn, requestState);


            updateStopInfos(asgn, pickupIndex, dropoffIndex);

            if (pickupIndex == 0)
                lastMinutePickup(vehId);

            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.updateRoutesTime += routeUpdateTime;

            if (asgn.pickupStopIdx == 0 && numStopsBefore > 1 &&
                    variableData.schedDepTimesFor(vehId)[0] < requestState.originalRequest.requestTime) {
                movePreviousStopToCurrentLocationForReroute(*asgn.vehicle);
            }

            updateBucketState(asgn, pickupIndex, dropoffIndex);

            pickupStopId = variableData.stopIdsFor(vehId)[pickupIndex];
            dropoffStopId = variableData.stopIdsFor(vehId)[dropoffIndex];

            // Register the inserted pickup and dropoff with the path data
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            pathTracker.updateForBestAssignment(pickupIndex, dropoffIndex, numStopsBefore, dropoffAtExistingStop);

        }

        void notifyStopStarted(const Vehicle &veh) {
            pathTracker.logCompletedLeg(veh);

            // Update buckets and route state
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, variableData);
            ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1, variableData);


            auto stopIds = variableData.stopIdsFor(veh.vehicleId);
            const auto newStartStopId = stopIds[1];
            stopInfos[stopIds[0]].isFixed = false;
            variableUpdater.removeStartOfCurrentLeg(veh.vehicleId);


            if (variableData.numStopsOf(veh.vehicleId) > 0 && variableData.stopHasPickups(newStartStopId)) {
                auto &newPickup = stopInfos[newStartStopId];
                updatePickupInfo(stopInfos[newStartStopId]);
                newPickup.insertIndex = 1;
                fixedUpdater.addNewPickup(newPickup, newStartStopId);
                newPickup.isFixed= true;

                for (const auto &tuple: newPickup.pickedupReqAndDropoff) {
                    auto &dropoffStopInfo = stopInfos[tuple.second];
                    calcInsertIndex(dropoffStopInfo, tuple.second, false);
                    dropoffStopInfo.maxArrTimeAtDropoff = maxArrTimeAtDropForRequest[tuple.first];
                    fixedUpdater.addNewDropoff(dropoffStopInfo, tuple.first, tuple.second);
                    dropoffStopInfo.isFixed = true;
                }

                newPickup.pickedupReqAndDropoff.clear();
            }
            fixedUpdater.removeStartOfCurrentLeg(veh.vehicleId);


            assert(fixedData.numStopsOf(veh.vehicleId) <= variableData.numStopsOf(veh.vehicleId));
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            assert(variableData.numStopsOf(vehId) == 1);
            const auto loc = inputGraph.edgeHead(variableData.stopLocationsFor(vehId)[0]);

            auto stopIds = variableData.stopIdsFor(veh.vehicleId);
            stopInfos[stopIds[0]].isFixed = false;

            lastStopsAtVertices.removeLastStopAt(loc, vehId);
            lastStopBucketsEnv.removeBucketEntries(veh, 0, variableData);
            variableUpdater.removeStartOfCurrentLeg(vehId);
            fixedUpdater.removeStartOfCurrentLeg(vehId);
        }


        void writeBestAssignmentToLogger() {
            bestAssignmentsLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", ";

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
            const auto &numStops = variableData.numStopsOf(vehId);
            using time_utils::getVehDepTimeAtStopForRequest;
            const auto &vehDepTimeBeforePickup = getVehDepTimeAtStopForRequest(vehId, bestAsgn.pickupStopIdx,
                                                                               requestState, variableData);
            const auto &vehDepTimeBeforeDropoff = getVehDepTimeAtStopForRequest(vehId, bestAsgn.dropoffStopIdx,
                                                                                requestState, variableData);
            bestAssignmentsLogger
                    << vehId << ", "
                    << bestAsgn.pickupStopIdx << ", "
                    << bestAsgn.dropoffStopIdx << ", "
                    << bestAsgn.distToPickup << ", "
                    << bestAsgn.distFromPickup << ", "
                    << bestAsgn.distToDropoff << ", "
                    << bestAsgn.distFromDropoff << ", "
                    << bestAsgn.pickup->id << ", "
                    << bestAsgn.pickup->walkingDist << ", "
                    << bestAsgn.dropoff->id << ", "
                    << bestAsgn.dropoff->walkingDist << ", "
                    << numStops << ", "
                    << vehDepTimeBeforePickup << ", "
                    << vehDepTimeBeforeDropoff << ", "
                    << "false, "
                    << requestState.getBestCost() << "\n";
        }

        void writePerformanceLogs() {
            overallPerfLogger << requestState.originalRequest.requestId << ", "
                              << requestState.stats().getLoggerRow() << "\n";
            initializationPerfLogger << requestState.originalRequest.requestId << ", "
                                     << requestState.stats().initializationStats.getLoggerRow() << "\n";
            ellipticBchPerfLogger << requestState.originalRequest.requestId << ", "
                                  << requestState.stats().ellipticBchStats.getLoggerRow() << "\n";
            pdDistancesPerfLogger << requestState.originalRequest.requestId << ", "
                                  << requestState.stats().pdDistancesStats.getLoggerRow() << "\n";
            ordPerfLogger << requestState.originalRequest.requestId << ", "
                          << requestState.stats().ordAssignmentsStats.getLoggerRow() << "\n";
            pbnsPerfLogger << requestState.originalRequest.requestId << ", "
                           << requestState.stats().pbnsAssignmentsStats.getLoggerRow() << "\n";
            palsPerfLogger << requestState.originalRequest.requestId << ", "
                           << requestState.stats().palsAssignmentsStats.getLoggerRow() << "\n";
            dalsPerfLogger << requestState.originalRequest.requestId << ", "
                           << requestState.stats().dalsAssignmentsStats.getLoggerRow() << "\n";
            updatePerfLogger << requestState.originalRequest.requestId << ", "
                             << requestState.stats().updateStats.getLoggerRow() << "\n";
        }

    private:

        void lastMinutePickup(const int vehId) {
            auto stopIds = variableData.stopIdsFor(vehId);

            auto &currStopInfo = stopInfos[stopIds[0]];
            updatePickupInfo(currStopInfo);
            currStopInfo.insertIndex = 0;
            fixedUpdater.addNewPickup(currStopInfo, stopIds[0]);

            for (const auto &tuple: currStopInfo.pickedupReqAndDropoff) {
                auto &dropoffStopInfo = stopInfos[tuple.second];
                calcInsertIndex(dropoffStopInfo, tuple.second, true);
                dropoffStopInfo.maxArrTimeAtDropoff = maxArrTimeAtDropForRequest[tuple.first];
                fixedUpdater.addNewDropoff(dropoffStopInfo, tuple.first, tuple.second);
                dropoffStopInfo.isFixed = true;
            }

            currStopInfo.pickedupReqAndDropoff.clear();
        }

        void calcInsertIndex(StopInfo &info, const int stopId, bool lastMinute) {
            int insertIndex = lastMinute ? 0 : 1; // Old start has not been removed yet
            int count = 0;
            auto stopIds = variableData.stopIdsFor(info.vehicleId);
            while (stopId != stopIds[count]) {
                if (stopInfos[stopIds[count]].isFixed) {
                    insertIndex++;
                }
                count++;
            }
            info.insertIndex = insertIndex;
        }

        void updatePickupInfo(StopInfo &info) {
            const int vehId = info.vehicleId;
            auto schedArrTimes = variableData.schedArrTimesFor(vehId);
            auto schedDepTimes = variableData.schedDepTimesFor(vehId);
            auto maxArrTimes = variableData.maxArrTimesFor(vehId);
            info.schedArrTime = schedArrTimes[0];
            info.schedDepTime = schedDepTimes[0];
            info.maxArrTime = maxArrTimes[0];
        }

        // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), move stop 0
        // of the vehicle to its current position to maintain the invariant of the schedule for the first stop,
        // i.e. dist(s_0, s_1) = schedArrTime(s_1) - schedDepTime(s_0).
        // Update the stop location in the routeState and the bucket entries in the elliptic buckets.
        void movePreviousStopToCurrentLocationForReroute(const Vehicle &veh) { //TODO: Done
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, variableData);
            //ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, fixedData);
            assert(curVehLocs.knowsCurrentLocationOf(veh.vehicleId));
            auto loc = curVehLocs.getCurrentLocationOf(veh.vehicleId);
            variableUpdater.updateStartOfCurrentLeg(veh.vehicleId, loc.location, loc.depTimeAtHead);
            fixedUpdater.updateStartOfCurrentLeg(veh.vehicleId, loc.location, loc.depTimeAtHead);
            ellipticBucketsEnv.generateSourceBucketEntries(veh, 0, variableData);
            //ellipticBucketsEnv.generateSourceBucketEntries(veh, 0, fixedData);
        }


        // Updates the bucket state (elliptic buckets, last stop buckets, lastStopsAtVertices structure) given an
        // assignment that has already been inserted into routeState as well as the stop index of the pickup and
        // dropoff after the insertion.
        void updateBucketState(const Assignment &asgn,
                               const int pickupIndex, const int dropoffIndex) {

            generateBucketStateForNewStops(asgn, pickupIndex, dropoffIndex);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsUpdaterT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.vehicle, variableData);
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.vehicle, variableData);
            }
        }

        void generateBucketStateForNewStops(const Assignment &asgn, const int pickupIndex, const int dropoffIndex) {
            const auto vehId = asgn.vehicle->vehicleId;
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, pickupIndex, variableData);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, pickupIndex, variableData);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, dropoffIndex, variableData);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (dropoffIndex < variableData.numStopsOf(vehId) - 1) {
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, dropoffIndex, variableData);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
            const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
            ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, formerLastStopIdx, variableData);

            // Remove last stop bucket entries for former last stop and generate them for dropoff
            lastStopBucketsEnv.removeBucketEntries(*asgn.vehicle, formerLastStopIdx, variableData);
            lastStopBucketsEnv.generateBucketEntries(*asgn.vehicle, dropoffIndex, variableData);

            // Update lastStopAtVertices structure
            Timer timer;
            const auto oldLocHead = inputGraph.edgeHead(variableData.stopLocationsFor(vehId)[formerLastStopIdx]);
            const auto newLocHead = inputGraph.edgeHead(asgn.dropoff->loc);
            lastStopsAtVertices.removeLastStopAt(oldLocHead, vehId);
            lastStopsAtVertices.insertLastStopAt(newLocHead, vehId);
            const auto lastStopsAtVerticesUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.lastStopsAtVerticesUpdateTime += lastStopsAtVerticesUpdateTime;
        }

        void updateStopInfos(const Assignment &assignment, const int pickupIndex, const int dropoffIndex) {
            //Update data for FixedRouteState
            const int reqId = requestState.originalRequest.requestId;
            if (maxArrTimeAtDropForRequest.size() < reqId + 1)
                maxArrTimeAtDropForRequest.resize(reqId + 1, 0);

            maxArrTimeAtDropForRequest[reqId] = requestState.getMaxArrTimeAtDropoff(assignment.pickup->id, assignment.dropoff->id);

            const auto stopIds = variableData.stopIdsFor(assignment.vehicle->vehicleId);

            const auto pickUpStopId = stopIds[pickupIndex];
            const auto dropOffStopId = stopIds[dropoffIndex];

            const int newSize = std::max(pickUpStopId, dropOffStopId) + 1;
            if (stopInfos.size() < newSize)
                stopInfos.resize(newSize);


            auto &pickupInfo = stopInfos[pickUpStopId];
            auto &dropoffInfo = stopInfos[dropOffStopId];

            pickupInfo.location = assignment.pickup->loc;
            pickupInfo.vehicleId = assignment.vehicle->vehicleId;
            pickupInfo.pickedupReqAndDropoff.emplace_back(reqId, dropOffStopId);

            dropoffInfo.location = assignment.dropoff->loc;
            dropoffInfo.vehicleId = assignment.vehicle->vehicleId;
        }

        const InputGraphT &inputGraph;
        RequestState<CostCalculatorT> &requestState;
        const InputConfig &inputConfig;
        const CurVehLocsT &curVehLocs;
        PathTrackerT &pathTracker;

        // Route state
        RouteStateUpdaterT &variableUpdater;
        FixedRouteStateUpdaterT &fixedUpdater;
        RouteStateData &variableData;
        RouteStateData &fixedData;
        std::vector<StopInfo> stopInfos;  //stopInfos[stopId] returns the StopInfo object of stop with id stopId
        std::vector<int> maxArrTimeAtDropForRequest; //maxArrTimeAtDropoff[reqId] returns maxArrTimeAtDropoff of the request

        // Bucket state
        EllipticBucketsUpdaterT &ellipticBucketsEnv;
        LastStopBucketsUpdaterT &lastStopBucketsEnv;
        LastStopsAtVertices &lastStopsAtVertices;

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