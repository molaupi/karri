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
            typename BucketsWrapperT,
            typename LoggerT = NullLogger>
    class SystemStateUpdater {

    public:

        SystemStateUpdater(const InputGraphT &inputGraph,
                           const InputConfig &inputConfig, const CurVehLocsT &curVehLocs,
                           PathTrackerT &pathTracker,
                           RouteStateUpdaterT &variableUpdater, RouteStateData &variableData,
                           FixedRouteStateUpdaterT &fixedUpdater, RouteStateData &fixedData,
                           BucketsWrapperT &fixedBuckets, BucketsWrapperT &variableBuckets,
                           EllipticBucketsUpdaterT &ellipticBucketsEnv,
                           LastStopBucketsUpdaterT &lastStopBucketsEnv,
                           LastStopsAtVertices &variableLastStopsAtVertices,
                           LastStopsAtVertices &fixedLastStopsAtVertices,
                           Fleet &fleet)
                : inputGraph(inputGraph),
                  inputConfig(inputConfig),
                  curVehLocs(curVehLocs),
                  pathTracker(pathTracker),
                  variableUpdater(variableUpdater),
                  fixedUpdater(fixedUpdater),
                  variableData(variableData),
                  fixedData(fixedData),
                  fleet(fleet),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  fixedBuckets(fixedBuckets),
                  variableBuckets(variableBuckets),
                  lastStopBucketsEnv(lastStopBucketsEnv),
                  variableLastStopsAtVertices(variableLastStopsAtVertices),
                  fixedLastStopsAtVertices(fixedLastStopsAtVertices),
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
                                                                  std::string(stats::UpdatePerformanceStats::LOGGER_COLS))) {}






        void notifyStopStarted(const Vehicle &veh) {
            pathTracker.logCompletedLeg(veh);

            // Update buckets and route state
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, variableData, variableBuckets);
            ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1, variableData, variableBuckets);


            auto stopIds = variableData.stopIdsFor(veh.vehicleId);
            const auto newStartStopId = stopIds[1];
            stopInfos[stopIds[0]].isFixed = false;
            variableUpdater.removeStartOfCurrentLeg(veh.vehicleId);


            if (variableData.numStopsOf(veh.vehicleId) > 0 && variableData.stopHasPickups(newStartStopId)) {
                auto &newPickup = stopInfos[newStartStopId];
                updatePickupInfo(stopInfos[newStartStopId]);
                newPickup.insertIndex = 1;
                fixedUpdater.addNewPickup(newPickup, newStartStopId);

                bool pickUpWasAtEnd = newPickup.insertIndex == fixedData.numStopsOf(veh.vehicleId) - 1;
                bool pickUpWasFixed = newPickup.isFixed;

                newPickup.isFixed= true;

                for (const auto &tuple: newPickup.pickedupReqAndDropoff) {
                    auto &dropoffStopInfo = stopInfos[tuple.second];
                    calcInsertIndex(dropoffStopInfo, tuple.second, false);
                    dropoffStopInfo.maxArrTimeAtDropoff = maxArrTimeAtDropForRequest[tuple.first];
                    fixedUpdater.addNewDropoff(dropoffStopInfo, tuple.first, tuple.second);
                    assert(stopInfos[tuple.second].vehicleId == veh.vehicleId);
                    // Updating buckets
                    generateFixedBucketStateForNewPickup(newPickup, pickUpWasFixed);
                    generateFixedBucketStateForNewDropoff(dropoffStopInfo, pickUpWasAtEnd, pickUpWasFixed);
                    pickUpWasAtEnd = false;
                    pickUpWasFixed = true;

                    dropoffStopInfo.isFixed = true;
                }

                newPickup.pickedupReqAndDropoff.clear();
            }
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, fixedData, fixedBuckets);
            ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1, fixedData, fixedBuckets);

            fixedUpdater.removeStartOfCurrentLeg(veh.vehicleId);


            assert(fixedData.numStopsOf(veh.vehicleId) <= variableData.numStopsOf(veh.vehicleId));
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            assert(variableData.numStopsOf(vehId) == 1);
            const auto loc = inputGraph.edgeHead(variableData.stopLocationsFor(vehId)[0]);

            auto stopIds = variableData.stopIdsFor(veh.vehicleId);
            stopInfos[stopIds[0]].isFixed = false;

            variableLastStopsAtVertices.removeLastStopAt(loc, vehId);
            fixedLastStopsAtVertices.removeLastStopAt(loc, vehId);
            lastStopBucketsEnv.removeBucketEntries(veh, 0, variableData);
            lastStopBucketsEnv.removeBucketEntries(veh, 0, fixedData);
            variableUpdater.removeStartOfCurrentLeg(vehId);
            fixedUpdater.removeStartOfCurrentLeg(vehId);
        }

        void insertBestAssignment(int &pickupStopId, int &dropoffStopId, RequestState<CostCalculatorT> &requestState, const bool reoptRun = false) {
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

            pickupStopId = variableData.stopIdsFor(vehId)[pickupIndex];
            dropoffStopId = variableData.stopIdsFor(vehId)[dropoffIndex];

            // Cash changed stopInfo
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            if (reoptRun && !pickupAtExistingStop) {
                newCreatedStopIds.push_back(pickupStopId);
            } else if (reoptRun && std::find(modifiedStopIds.begin(), modifiedStopIds.end(), pickupStopId) == modifiedStopIds.end()
                       && std::find(newCreatedStopIds.begin(), newCreatedStopIds.end(), pickupIndex) == newCreatedStopIds.end()) {
                modifiedStopIds.push_back(pickupStopId);
                unmodifiedStopInfos.push_back(stopInfos[pickupStopId]);
            }
            if (reoptRun && !dropoffAtExistingStop) {
                newCreatedStopIds.push_back(dropoffStopId);
            } else if (reoptRun && std::find(modifiedStopIds.begin(), modifiedStopIds.end(), dropoffStopId) == modifiedStopIds.end()
                       && std::find(newCreatedStopIds.begin(), newCreatedStopIds.end(), dropoffStopId) == newCreatedStopIds.end()) {
                modifiedStopIds.push_back(dropoffStopId);
                unmodifiedStopInfos.push_back(stopInfos[dropoffStopId]);
            }


            updateStopInfos(asgn, pickupIndex, dropoffIndex, requestState);

            if (pickupIndex == 0 && !reoptRun)
                lastMinutePickup(vehId);
            else if (pickupIndex == 0 && reoptRun && std::find(vehiclesWithLastMinutePickup.begin(), vehiclesWithLastMinutePickup.end(), vehId) == vehiclesWithLastMinutePickup.end())
                vehiclesWithLastMinutePickup.push_back(vehId);

            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.updateRoutesTime += routeUpdateTime;

            if (asgn.pickupStopIdx == 0 && numStopsBefore > 1 &&
                variableData.schedDepTimesFor(vehId)[0] < requestState.now()) {
                movePreviousStopToCurrentLocationForReroute(*asgn.vehicle);
            }

            updateBucketState(asgn, pickupIndex, dropoffIndex, requestState);


            pathTracker.updateForBestAssignment(pickupIndex, dropoffIndex, numStopsBefore, dropoffAtExistingStop);

        }

        // Replaces variable route of vehicle with route in the fixed route state
        void exchangeRoutesFor(const Vehicle &veh) {
            const int vehId = veh.vehicleId;
            bool lastStopIdentical = variableData.stopIdsFor(vehId)[variableData.numStopsOf(vehId) - 1] ==
                                     fixedData.stopIdsFor(vehId)[fixedData.numStopsOf(vehId) - 1];
            // Exchange elliptic buckets
            ellipticBucketsEnv.exchangeBucketEntries(veh, variableData, fixedData, variableBuckets);

            // Update last stop vertice
            if (!lastStopIdentical) {
                const auto oldLocHead = inputGraph.edgeHead(variableData.stopLocationsFor(vehId)[variableData.numStopsOf(vehId) - 1]);
                const auto newLocHead = inputGraph.edgeHead(fixedData.stopLocationsFor(vehId)[fixedData.numStopsOf(vehId) - 1]);
                variableLastStopsAtVertices.removeLastStopAt(oldLocHead, vehId);
                variableLastStopsAtVertices.insertLastStopAt(newLocHead, vehId);
            }

            //assert(variableData.schedDepTimesFor(vehId)[0] == fixedData.schedDepTimesFor(vehId)[0]);
            // Update LastStopBuckets and exchange actual routes
            lastStopBucketsEnv.removeBucketEntries(veh, variableData.numStopsOf(vehId) - 1, variableData);
            for (const int stopId: variableUpdater.exchangeRouteFor(vehId, fixedData)) {
                modifiedStopIds.push_back(stopId);
                deletedStopIds.push_back(stopId);
                unmodifiedStopInfos.push_back(stopInfos[stopId]);
                stopInfos[stopId] = StopInfo();
            }

            for (const int stopId: variableData.stopIdsFor(vehId)) {
                assert(stopInfos[stopId].isFixed || stopId == vehId);
                modifiedStopIds.push_back(stopId);
                unmodifiedStopInfos.push_back(stopInfos[stopId]);
                stopInfos[stopId].isFixed = true;
                stopInfos[stopId].pickedupReqAndDropoff.clear();
            }
            lastStopBucketsEnv.generateBucketEntries(veh, variableData.numStopsOf(vehId) - 1, variableData);
        }

        void setChangesOfReoptimizationRun() {
            for (const int vehId: vehiclesWithLastMinutePickup) {
                lastMinutePickup(vehId);
            }
            variableUpdater.invalidateStopIds(deletedStopIds);
            vehiclesWithLastMinutePickup.clear();
            modifiedStopIds.clear();
            unmodifiedStopInfos.clear();
            newCreatedStopIds.clear();
            deletedStopIds.clear();
        }

        void revertChangesOfReoptimizationRun(std::vector<VehicleRouteData> &oldRoutes) {
            assert(oldRoutes.size() >= 1);
            for (int i = 0; i < modifiedStopIds.size(); i++) {
                stopInfos[modifiedStopIds[i]] = unmodifiedStopInfos[i];
            }
            for (const int stopId: newCreatedStopIds) {
                stopInfos[stopId] = StopInfo();
            }

            revertRouteStateFor(oldRoutes[0], false);
            for (int i = 1; i < oldRoutes.size(); i++) {
                revertRouteStateFor(oldRoutes[i], true);
            }

            variableUpdater.invalidateStopIds(newCreatedStopIds);
            vehiclesWithLastMinutePickup.clear();
            modifiedStopIds.clear();
            unmodifiedStopInfos.clear();
            newCreatedStopIds.clear();
            deletedStopIds.clear();
        }


        void writeBestAssignmentToLogger(const RequestState<CostCalculatorT> &requestState) {
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

        void writePerformanceLogs(const RequestState<CostCalculatorT> &requestState) {
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

        // Replaces variable route with the route stored in data
        void revertRouteStateFor(const VehicleRouteData &data, const bool resetVarRoute) {
            assert(!data.stopIds.empty());
            const int numStops = data.stopIds.size();
            const int vehId = data.veh.vehicleId;
            bool lastStopIdentical = variableData.stopIdsFor(vehId)[variableData.numStopsOf(vehId) - 1] == data.stopIds.back();

            // Exchange elliptic buckets
            ellipticBucketsEnv.exchangeBucketEntries(data.veh, variableData, data, variableBuckets);

            // Update last stop vertice
            if (!lastStopIdentical) {
                const auto oldLocHead = inputGraph.edgeHead(variableData.stopLocationsFor(vehId)[variableData.numStopsOf(vehId) - 1]);
                const auto newLocHead = inputGraph.edgeHead(data.stopLocations[numStops - 1]);
                variableLastStopsAtVertices.removeLastStopAt(oldLocHead, vehId);
                variableLastStopsAtVertices.insertLastStopAt(newLocHead, vehId);
            }

            lastStopBucketsEnv.removeBucketEntries(data.veh, variableData.numStopsOf(vehId) - 1, variableData);

            assert(variableData.stopIdsFor(vehId)[0] == data.stopIds[0]);
            if (variableData.stopLocationsFor(vehId)[0] != data.stopLocations[0]) {
                revertReroute(data); // In case the start of the current leg has been changed in the reoptimization run
            }

            if (resetVarRoute) {
                variableUpdater.resetVarRouteFor(data);
            } else {
                variableUpdater.resetFixedRouteFor(data);
            }
            lastStopBucketsEnv.generateBucketEntries(data.veh, variableData.numStopsOf(vehId) - 1, variableData);
        }

        void revertReroute(const VehicleRouteData &data) {
            const Vehicle &veh = data.veh;
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, variableData, variableBuckets);
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, fixedData, fixedBuckets);
            variableData.updateStopLocationFor(veh.vehicleId, 0 , data.stopLocations[0]);
            //ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, variableData, variableBuckets);
            //ellipticBucketsEnv.generateSourceBucketEntries(veh, 0, variableData, variableBuckets);

            if (fixedData.numStopsOf(veh.vehicleId) > 1) {
                fixedUpdater.updateStartOfCurrentLeg(veh.vehicleId, data.stopLocations[0], data.schedDepTimes[0]);
                //ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, fixedData, fixedBuckets);
                //ellipticBucketsEnv.generateSourceBucketEntries(veh, 0, fixedData, fixedBuckets);
                return;
            }
            const auto oldLocHead = inputGraph.edgeHead(fixedData.stopLocationsFor(veh.vehicleId)[0]);
            const auto newLocHead = inputGraph.edgeHead(data.stopLocations[0]);

            lastStopBucketsEnv.removeBucketEntries(veh, 0, fixedData);
            fixedLastStopsAtVertices.removeLastStopAt(oldLocHead, veh.vehicleId);

            fixedUpdater.updateStartOfCurrentLeg(veh.vehicleId, data.stopLocations[0], data.schedDepTimes[0]);

            lastStopBucketsEnv.generateBucketEntries(veh, 0, fixedData);
            fixedLastStopsAtVertices.insertLastStopAt(newLocHead, veh.vehicleId);
        }

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

                generateFixedBucketStateForNewDropoff(dropoffStopInfo, false, true);

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
        void movePreviousStopToCurrentLocationForReroute(const Vehicle &veh) {
            assert(curVehLocs.knowsCurrentLocationOf(veh.vehicleId));
            auto loc = curVehLocs.getCurrentLocationOf(veh.vehicleId);
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, variableData, variableBuckets);
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, fixedData, fixedBuckets);
            variableUpdater.updateStartOfCurrentLeg(veh.vehicleId, loc.location, loc.depTimeAtHead);
            ellipticBucketsEnv.generateSourceBucketEntries(veh, 0, variableData, variableBuckets);

            if (fixedData.numStopsOf(veh.vehicleId) > 1) {
                fixedUpdater.updateStartOfCurrentLeg(veh.vehicleId, loc.location, loc.depTimeAtHead);
                ellipticBucketsEnv.generateSourceBucketEntries(veh, 0, fixedData, fixedBuckets);
                return;
            }
            const auto oldLocHead = inputGraph.edgeHead(fixedData.stopLocationsFor(veh.vehicleId)[0]);
            const auto newLocHead = inputGraph.edgeHead(loc.location);

            lastStopBucketsEnv.removeBucketEntries(veh, 0, fixedData);
            fixedLastStopsAtVertices.removeLastStopAt(oldLocHead, veh.vehicleId);

            fixedUpdater.updateStartOfCurrentLeg(veh.vehicleId, loc.location, loc.depTimeAtHead);

            lastStopBucketsEnv.generateBucketEntries(veh, 0, fixedData);
            fixedLastStopsAtVertices.insertLastStopAt(newLocHead, veh.vehicleId);
        }



        // Updates the bucket state (elliptic buckets, last stop buckets, lastStopsAtVertices structure) given an
        // assignment that has already been inserted into routeState as well as the stop index of the pickup and
        // dropoff after the insertion.
        void updateBucketState(const Assignment &asgn,
                               const int pickupIndex, const int dropoffIndex, RequestState<CostCalculatorT> &requestState) {

            generateBucketStateForNewStops(asgn, pickupIndex, dropoffIndex, requestState);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (BucketsWrapperT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.vehicle, variableData, variableBuckets);
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.vehicle, variableData, variableBuckets);
            }
        }

        void generateFixedBucketStateForNewPickup(const StopInfo &pickup, const bool pickUpWasFixed) {
            const auto vehId = pickup.vehicleId;

            if (!pickUpWasFixed) {
                ellipticBucketsEnv.generateTargetBucketEntries(fleet[vehId], pickup.insertIndex, fixedData, fixedBuckets);
                ellipticBucketsEnv.generateSourceBucketEntries(fleet[vehId], pickup.insertIndex, fixedData, fixedBuckets);
            }
        }

        void generateFixedBucketStateForNewDropoff(const StopInfo &dropoff, const bool pickupWasAtEnd, const bool pickUpWasFixed) {
            const auto vehId = dropoff.vehicleId;
            const bool dropoffAtExistingStop = dropoff.isFixed;

            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.generateTargetBucketEntries(fleet[vehId], dropoff.insertIndex, fixedData, fixedBuckets);

            if (dropoff.insertIndex < fixedData.numStopsOf(vehId) - 1) {
                ellipticBucketsEnv.generateSourceBucketEntries(fleet[vehId], dropoff.insertIndex, fixedData, fixedBuckets);
                return;
            }

            const int formerLastStopIdx = dropoff.insertIndex - (pickupWasAtEnd && !pickUpWasFixed) - 1;
            ellipticBucketsEnv.generateSourceBucketEntries(fleet[vehId], formerLastStopIdx, fixedData, fixedBuckets);

            lastStopBucketsEnv.removeBucketEntries(fleet[vehId], formerLastStopIdx, fixedData);
            lastStopBucketsEnv.generateBucketEntries(fleet[vehId], dropoff.insertIndex, fixedData);

            const auto oldLocHead = inputGraph.edgeHead(fixedData.stopLocationsFor(vehId)[formerLastStopIdx]);
            const auto newLocHead = inputGraph.edgeHead(dropoff.location);


            fixedLastStopsAtVertices.removeLastStopAt(oldLocHead, vehId);
            fixedLastStopsAtVertices.insertLastStopAt(newLocHead, vehId);


            if constexpr (BucketsWrapperT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(fleet[dropoff.vehicleId], fixedData, fixedBuckets);
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(fleet[dropoff.vehicleId], fixedData, fixedBuckets);
            }
        }

        void generateBucketStateForNewStops(const Assignment &asgn, const int pickupIndex, const int dropoffIndex, RequestState<CostCalculatorT> &requestState) {
            const auto vehId = asgn.vehicle->vehicleId;
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, pickupIndex, variableData, variableBuckets);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, pickupIndex, variableData, variableBuckets);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, dropoffIndex, variableData, variableBuckets);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (dropoffIndex < variableData.numStopsOf(vehId) - 1) {
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, dropoffIndex, variableData, variableBuckets);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
            const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
            ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, formerLastStopIdx, variableData, variableBuckets);

            // Remove last stop bucket entries for former last stop and generate them for dropoff
            lastStopBucketsEnv.removeBucketEntries(*asgn.vehicle, formerLastStopIdx, variableData);
            lastStopBucketsEnv.generateBucketEntries(*asgn.vehicle, dropoffIndex, variableData);

            // Update lastStopAtVertices structure
            Timer timer;
            const auto oldLocHead = inputGraph.edgeHead(variableData.stopLocationsFor(vehId)[formerLastStopIdx]);
            const auto newLocHead = inputGraph.edgeHead(asgn.dropoff->loc);
            variableLastStopsAtVertices.removeLastStopAt(oldLocHead, vehId);
            variableLastStopsAtVertices.insertLastStopAt(newLocHead, vehId);
            const auto lastStopsAtVerticesUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.lastStopsAtVerticesUpdateTime += lastStopsAtVerticesUpdateTime;
        }

        void updateStopInfos(const Assignment &assignment, const int pickupIndex, const int dropoffIndex, RequestState<CostCalculatorT> &requestState) {
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

        // Data to make changes revertible
        std::vector<int> deletedStopIds;
        std::vector<int> vehiclesWithLastMinutePickup;
        std::vector<int> newCreatedStopIds;
        std::vector<int> modifiedStopIds; //Stopids that were removed/ modified
        std::vector<StopInfo> unmodifiedStopInfos; // modifiedStopIds[i] corresponds to stopInfo at unmodifiedStopInfos[i]

        const InputGraphT &inputGraph;
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
        Fleet &fleet;

        // Bucket state
        EllipticBucketsUpdaterT &ellipticBucketsEnv;
        BucketsWrapperT &fixedBuckets;
        BucketsWrapperT &variableBuckets;
        LastStopBucketsUpdaterT &lastStopBucketsEnv;
        LastStopsAtVertices &variableLastStopsAtVertices;
        LastStopsAtVertices &fixedLastStopsAtVertices;

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