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

#include <chrono>

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"
#include "Algorithms/KaRRi/PointToPointPathComputer.h"

namespace karri {
    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
        typename EllipticBucketsEnvT,
        typename RepositioningBucketsEnvT,
        typename LastStopBucketsEnvT,
        typename CHEnvT,
        typename CurVehLocsT,
        typename PathTrackerT,
        typename LoggerT = NullLogger>
    class SystemStateUpdater {

        static constexpr bool USE_DETERMINISTIC_REPOSITIONING_TARGETS = true;

    public:
        SystemStateUpdater(const InputGraphT &inputGraph, const Fleet &fleet, RequestState &requestState,
                           const CurVehLocsT &curVehLocs,
                           PathTrackerT &pathTracker,
                           RouteState &routeState, EllipticBucketsEnvT &ellipticBucketsEnv,
                           RepositioningBucketsEnvT &repositioningBucketsEnv,
                           LastStopBucketsEnvT &lastStopBucketsEnv, CHEnvT &chEnv)
            : inputGraph(inputGraph),
        fleet(fleet),
              requestState(requestState),
              curVehLocs(curVehLocs),
              pathTracker(pathTracker),
              routeState(routeState),
              ellipticBucketsEnv(ellipticBucketsEnv),
              repositioningBucketsEnv(repositioningBucketsEnv),
              lastStopBucketsEnv(lastStopBucketsEnv),
              pathComputer(inputGraph, chEnv),
        seenOriginLocations(),
        gen(USE_DETERMINISTIC_REPOSITIONING_TARGETS? 0 : std::random_device{}()),
              bestAssignmentsLogger(LogManager<LoggerT>::getLogger("bestassignments.csv",
                                                                   "request_id,"
                                                                   "request_time,"
                                                                   "direct_od_dist,"
                                                                   "vehicle_id,"
                                                                   "pickup_insertion_point,"
                                                                   "dropoff_insertion_point,"
                                                                   "dist_to_pickup,"
                                                                   "dist_from_pickup,"
                                                                   "dist_to_dropoff,"
                                                                   "dist_from_dropoff,"
                                                                   "pickup_id,"
                                                                   "pickup_walking_dist,"
                                                                   "dropoff_id,"
                                                                   "dropoff_walking_dist,"
                                                                   "num_stops,"
                                                                   "veh_dep_time_at_stop_before_pickup,"
                                                                   "veh_dep_time_at_stop_before_dropoff,"
                                                                   "not_using_vehicle,"
                                                                   "is_vehicle_repositioning,"
                                                                   "accepted,"
                                                                   "cost\n")),
              overallPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::DispatchingPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(stats::DispatchingPerformanceStats::LOGGER_COLS))),
              initializationPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::InitializationPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(
                                                     stats::InitializationPerformanceStats::LOGGER_COLS))),
              ellipticBchPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::EllipticBCHPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(stats::EllipticBCHPerformanceStats::LOGGER_COLS))),
              pdDistancesPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::PDDistancesPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(stats::PDDistancesPerformanceStats::LOGGER_COLS))),
              ordPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::OrdAssignmentsPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(
                                                     stats::OrdAssignmentsPerformanceStats::LOGGER_COLS))),
              pbnsPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::PbnsAssignmentsPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(
                                                     stats::PbnsAssignmentsPerformanceStats::LOGGER_COLS))),
              palsPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::PalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(
                                                     stats::PalsAssignmentsPerformanceStats::LOGGER_COLS))),
              dalsPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::DalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(
                                                     stats::DalsAssignmentsPerformanceStats::LOGGER_COLS))),
              updatePerfLogger(LogManager<LoggerT>::getLogger(stats::UpdatePerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::UpdatePerformanceStats::LOGGER_COLS))) {
        }


        void insertBestAssignment(int &pickupStopId, int &dropoffStopId) {
            if (requestState.isNotUsingVehicleBest()) {
                pickupStopId = -1;
                dropoffStopId = -1;
                return;
            }

            const auto &asgn = requestState.getBestAssignment();
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.pickup.loc));
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.dropoff.loc));
            assert(asgn.vehicle != nullptr);

            seenOriginLocations.push_back(requestState.originalRequest.origin);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = routeState.numStopsOf(vehId);
            const auto depTimeAtLastStopBefore = routeState.schedDepTimesFor(vehId)[numStopsBefore - 1];
            const bool wasIdle = routeState.getIdleVehicles().contains(vehId);

            if (routeState.isRepositioning(vehId)) {
                insertRepositioningAssignment(pickupStopId, dropoffStopId);
                return;
            }

            Timer timer;
            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, requestState);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.updateRoutesTime += routeUpdateTime;

            updateBucketState(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore, wasIdle);

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting.
            if (asgn.pickupStopIdx == 0 && numStopsBefore > 1 && routeState.schedDepTimesFor(vehId)[0] <
                requestState.originalRequest.requestTime) {
                createIntermediateStopStopAtCurrentLocationForReroute(*asgn.vehicle,
                                                                      requestState.originalRequest.requestTime);
                ++pickupIndex;
                ++dropoffIndex;
            }

            pickupStopId = routeState.stopIdsFor(vehId)[pickupIndex];
            dropoffStopId = routeState.stopIdsFor(vehId)[dropoffIndex];

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, dropoffStopId);
        }

        void notifyStartup(const Vehicle &veh) {
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                lastStopBucketsEnv.generateIdleBucketEntries(veh);
                routeState.markAsIdling(veh.vehicleId);
            } else {
                lastStopBucketsEnv.generateNonIdleBucketEntries(veh);
            }
        }

        void notifyStopStarted(const Vehicle &veh) {
            // Update buckets and route state
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0);
            ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1);
            routeState.removeStartOfCurrentLeg(veh.vehicleId);
        }


        void notifyStopCompleted(const Vehicle &veh) {
            pathTracker.logCompletedStop(veh);

            // If vehicle has become idle, update last stop bucket entries
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                lastStopBucketsEnv.updateBucketEntries(veh, 0);
                KASSERT(!routeState.getIdleVehicles().contains(veh.vehicleId));
                routeState.markAsIdling(veh.vehicleId);
            }
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            assert(routeState.numStopsOf(vehId) == 1);

            lastStopBucketsEnv.removeIdleBucketEntries(veh, 0);

            routeState.removeStartOfCurrentLeg(vehId);
            KASSERT(routeState.getIdleVehicles().contains(vehId));
            routeState.markAsNonIdling(vehId);
        }

        // Notify that an idle vehicle should start repositioning. Returns ID of repositioning vehicle.
        int notifyRepositioningEvent(const int now) {

            // Choose a random idle vehicle to reposition.
            const auto& idle = routeState.getIdleVehicles();
            if (idle.size() == 0)
                return INVALID_ID;
            std::uniform_int_distribution<size_t> vehDis(0, idle.size() - 1);
            const int vehId = *(idle.begin() + vehDis(gen));

            // Choose a repositioning target. We pick one of the previously seen origin locations at random
            // (weighted by how many times the location has occurred).
            std::uniform_int_distribution<size_t> targetLocDis(0, seenOriginLocations.size() - 1);
            const int target = seenOriginLocations[targetLocDis(gen)];

            startRepositioning(fleet[vehId], target, now);

            return vehId;
        }

        // Notify that a vehicle finished repositioning and is now idle at its new location.
        // This removes repositioning bucket entries, removes the old stop, and generates idle last-stop buckets.
        void notifyRepositioningFinished(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            KASSERT(routeState.isRepositioning(vehId));

            // Remove repositioning bucket entries for this vehicle
            const auto reposPath = recomputeRepositioningPath(vehId);
            repositioningBucketsEnv.removeRepositioningBucketEntries(veh, reposPath);

            // Mark vehicle as no longer repositioning, update idling location
            routeState.finishedRepositioning(vehId);

            // Now vehicle is idle again; generate idle last-stop bucket entries
            lastStopBucketsEnv.generateIdleBucketEntries(veh);
        }


        void writeBestAssignmentToLogger(const bool requestAccepted) {
            bestAssignmentsLogger
                    << requestState.originalRequest.requestId << ","
                    << requestState.originalRequest.requestTime << ","
                    << requestState.originalReqDirectDist << ",";

            if (requestState.getBestCost() == INFTY) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
                return;
            }

            if (requestState.isNotUsingVehicleBest()) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1," << true << ",-1," << requestAccepted << ","
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
                    << vehId << ","
                    << bestAsgn.pickupStopIdx << ","
                    << bestAsgn.dropoffStopIdx << ","
                    << bestAsgn.distToPickup << ","
                    << bestAsgn.distFromPickup << ","
                    << bestAsgn.distToDropoff << ","
                    << bestAsgn.distFromDropoff << ","
                    << bestAsgn.pickup.id << ","
                    << bestAsgn.pickup.walkingDist << ","
                    << bestAsgn.dropoff.id << ","
                    << bestAsgn.dropoff.walkingDist << ","
                    << numStops << ","
                    << vehDepTimeBeforePickup << ","
                    << vehDepTimeBeforeDropoff << ","
                    << false << ","
                    << routeState.isRepositioning(vehId) << ","
                    << requestAccepted << ","
                    << requestState.getBestCost() << "\n";
        }

        void writePerformanceLogs() {
            overallPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().getLoggerRow() << "\n";
            initializationPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().initializationStats.getLoggerRow() << "\n";
            ellipticBchPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().ellipticBchStats.getLoggerRow() << "\n";
            pdDistancesPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().pdDistancesStats.getLoggerRow() << "\n";
            ordPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().ordAssignmentsStats.getLoggerRow() << "\n";
            pbnsPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().pbnsAssignmentsStats.getLoggerRow() << "\n";
            palsPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().palsAssignmentsStats.getLoggerRow() << "\n";
            dalsPerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().dalsAssignmentsStats.getLoggerRow() << "\n";
            updatePerfLogger << requestState.originalRequest.requestId << ","
                    << requestState.stats().updateStats.getLoggerRow() << "\n";
        }

    private:
        std::vector<int> recomputeRepositioningPath(const int vehId) {
            KASSERT(routeState.isRepositioning(vehId));

            // Compute repositioning path from previous location (stop 0) to reposition target
            const auto fromEdge = routeState.stopLocationsFor(vehId)[0];
            const auto toEdge = routeState.getRepositioningTargetLocation(vehId);

            std::vector<int> pathEdges;
            pathComputer.computePathEdges(fromEdge, toEdge, pathEdges);

            // Convert edge path to vertex path (use edge heads)
            std::vector<int> vertexPath;
            vertexPath.reserve(pathEdges.size());
            for (const auto &e: pathEdges) vertexPath.push_back(inputGraph.edgeHead(e));

            return vertexPath;
        }

        // Start repositioning the given idle vehicle to the targetLocation.
        // This removes last-stop bucket entries for the vehicle and generates repositioning bucket entries for the
        // full path.
        void startRepositioning(const Vehicle &veh, const int targetLocation, const int now) {
            const auto vehId = veh.vehicleId;
            KASSERT(routeState.numStopsOf(vehId) == 1);

            // Compute full path from current idle location (stop 0) to targetLocation
            const auto fromEdge = routeState.stopLocationsFor(vehId)[0];

            std::vector<int> pathEdges;
            const int travelDist = pathComputer.computePathEdges(fromEdge, targetLocation, pathEdges);

            // Convert edge path to vertex path (use edge heads)
            std::vector<int> vertexPath;
            vertexPath.reserve(pathEdges.size());
            for (const auto &e: pathEdges) vertexPath.push_back(inputGraph.edgeHead(e));

            // Append dummy repositioning stop
            const int arrivalTime = now + travelDist;
            routeState.markAsRepositioning(vehId, now, targetLocation, arrivalTime);

            // Remove idle last-stop bucket entries for vehicle
            lastStopBucketsEnv.removeIdleBucketEntries(veh, 0);

            // Generate repositioning bucket entries
            repositioningBucketsEnv.generateRepositioningBucketEntries(veh, vertexPath);
        }

        void insertRepositioningAssignment(int &pickupStopId, int &dropoffStopId) {
            auto asgn = requestState.getBestAssignment();
            const int vehId = asgn.vehicle->vehicleId;
            KASSERT(routeState.isRepositioning(vehId));

            // For a repositioning assignment, we first remove the repositioning bucket entries:
            const auto reposPath = recomputeRepositioningPath(vehId);
            repositioningBucketsEnv.removeRepositioningBucketEntries(*asgn.vehicle, reposPath);

            // We now update the route state to cancel repositioning and insert a new intermediate stop at the
            // vehicle's current location.
            KASSERT(curVehLocs.knowsCurrentLocationOf(vehId));
            const VehicleLocation &vehicleLocation = curVehLocs.getCurrentLocationOf(vehId);
            routeState.cancelRepositioningAndCreateIntermediateStopAtCurrentLocation(
                vehId, vehicleLocation.location, requestState.originalRequest.requestTime,
                vehicleLocation.depTimeAtHead);

            // Due to the intermediate stop, the pickup and dropoff are now inserted after stop 1 in the route.
            // We update the indices in the assignment. The assignment is now the same as a regular PALS assignment
            // after the intermediate stop.
            ++asgn.pickupStopIdx;
            ++asgn.dropoffStopIdx;

            Timer timer;
            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, requestState);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.updateRoutesTime += routeUpdateTime;

            // Generate all elliptic and last stop bucket entries for the route since there were none before.
            const int numStops = routeState.numStopsOf(vehId);
            for (int i = 0; i < numStops - 1; ++i) {
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, i);
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, i + 1);
            }
            lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.vehicle);

            // Set IDs of new stops to be passed back to event simulation.
            pickupStopId = routeState.stopIdsFor(vehId)[pickupIndex];
            dropoffStopId = routeState.stopIdsFor(vehId)[dropoffIndex];

            // Register the inserted pickup and dropoff with the path data.
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, dropoffStopId);
        }

        // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
        // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
        // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
        // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
        // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
        void createIntermediateStopStopAtCurrentLocationForReroute(const Vehicle &veh, const int now) {
            assert(curVehLocs.knowsCurrentLocationOf(veh.vehicleId));
            auto loc = curVehLocs.getCurrentLocationOf(veh.vehicleId);
            LIGHT_KASSERT(loc.depTimeAtHead >= now);
            routeState.createIntermediateStopForReroute(veh.vehicleId, loc.location, now, loc.depTimeAtHead);
            ellipticBucketsEnv.generateSourceBucketEntries(veh, 1);
        }


        // Updates the bucket state (elliptic buckets, last stop buckets, lastStopsAtVertices structure) given an
        // assignment that has already been inserted into routeState as well as the stop index of the pickup and
        // dropoff after the insertion.
        void updateBucketState(const Assignment &asgn,
                               const int pickupIndex, const int dropoffIndex,
                               const int depTimeAtLastStopBefore,
                               const bool wasIdle) {
            generateBucketStateForNewStops(asgn, pickupIndex, dropoffIndex, wasIdle);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.vehicle);
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.vehicle);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const int vehId = asgn.vehicle->vehicleId;
            const auto numStopsAfter = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            const auto depTimeAtLastStopAfter = routeState.schedDepTimesFor(vehId)[numStopsAfter - 1];
            const bool depTimeAtLastChanged = depTimeAtLastStopAfter != depTimeAtLastStopBefore;

            if ((dropoffAtExistingStop || dropoffIndex < numStopsAfter - 1) && depTimeAtLastChanged) {
                lastStopBucketsEnv.updateBucketEntries(*asgn.vehicle, numStopsAfter - 1);
            }
        }

        void generateBucketStateForNewStops(const Assignment &asgn, const int pickupIndex, const int dropoffIndex,
            const bool wasIdle) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, pickupIndex);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, pickupIndex);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, dropoffIndex);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (dropoffIndex < numStops - 1) {
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, dropoffIndex);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
            const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
            ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, formerLastStopIdx);

            // Remove last stop bucket entries for former last stop and generate them for dropoff
            if (wasIdle) {
                KASSERT(formerLastStopIdx == 0);
                lastStopBucketsEnv.removeIdleBucketEntries(*asgn.vehicle, formerLastStopIdx);
            } else {
                lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.vehicle, formerLastStopIdx);
            }
            lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.vehicle);
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        RequestState &requestState;
        const CurVehLocsT &curVehLocs;
        PathTrackerT &pathTracker;

        // Route state
        RouteState &routeState;

        // Bucket state
        EllipticBucketsEnvT &ellipticBucketsEnv;
        RepositioningBucketsEnvT &repositioningBucketsEnv;
        LastStopBucketsEnvT &lastStopBucketsEnv;

        // Path computer
        PointToPointPathComputer<InputGraphT, CHEnvT> pathComputer;

        // Track seen origin locations of requests. Used to pick repositioning target locations.
        std::vector<int> seenOriginLocations;
        std::mt19937 gen;

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
