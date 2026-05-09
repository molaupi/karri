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
#include "Algorithms/KaRRi/BaseObjects/BestAsgnType.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"
#include "Algorithms/KaRRi/Stats/LastStopBucketUpdateStats.h"
#include "Algorithms/KaRRi/RiderModeChoice/TransportMode.h"

namespace karri {
    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
        typename EllipticBucketsEnvT,
        bool AreEllipticBucketsSortedByRemainingLeeway,
        typename LastStopBucketsEnvT,
        typename VehicleLocatorT,
        typename PathTrackerT,
        bool BATCHED_DISPATCHING,
        typename LoggerT = NullLogger,
        typename ComponentPerfLoggerT = NullLogger>
    class SystemStateUpdater {
    public:
        SystemStateUpdater(const InputGraphT &inputGraph,
                           const CH &vehCh,
                           const Fleet &fleet,
                           VehicleLocatorT &vehicleLocator,
                           PathTrackerT &pathTracker,
                           RouteState &routeState,
                           EllipticBucketsEnvT &ellipticBucketsEnv,
                           LastStopBucketsEnvT &lastStopBucketsEnv)
            : inputGraph(inputGraph),
              vehCh(vehCh),
              fleet(fleet),
              chQuery(vehCh),
              vehicleLocator(vehicleLocator),
              pathTracker(pathTracker),
              detourComputer(routeState),
              routeState(routeState),
              ellipticBucketsEnv(ellipticBucketsEnv),
              lastStopBucketsEnv(lastStopBucketsEnv),
              lastStopsWithChangedDepTime(fleet.size()),
              bestAssignmentsOverallLogger(LogManager<LoggerT>::getLogger("bestassignmentsoverall.csv",
                                                                          "request_id,"
                                                                          "request_time,"
                                                                          "direct_od_dist,"
                                                                          "number_of_legs,"
                                                                          "accepted,"
                                                                          "cost\n")),
              bestAssignmentsWithoutUsingVehicleLogger(
                  LogManager<LoggerT>::getLogger("bestassignmentswithoutvehicle.csv",
                                                 "request_id,"
                                                 "request_time,"
                                                 "direct_walking_dist,"
                                                 "accepted,"
                                                 "cost\n")),
              bestAssignmentsWithoutTransferLogger(
                  LogManager<LoggerT>::getLogger("bestassignmentswithouttransfer.csv",
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
                                                 "accepted,"
                                                 "cost\n")),
              bestAssignmentsWithTransferLogger(LogManager<LoggerT>::getLogger("bestassignmentswithtransfer.csv",
                                                                               "request_id,"
                                                                               "request_time,"
                                                                               "direct_od_dist,"
                                                                               "pickup_vehicle_id,"
                                                                               "dropoff_vehicle_id,"
                                                                               "pickup_insertion_point,"
                                                                               "transfer_pveh_insertion_point,"
                                                                               "transfer_dveh_insertion_point,"
                                                                               "dropoff_insertion_point,"
                                                                               "dist_to_pickup,"
                                                                               "dist_from_pickup,"
                                                                               "dist_to_transfer_pveh,"
                                                                               "dist_from_transfer_pveh,"
                                                                               "dist_to_transfer_dveh,"
                                                                               "dist_from_transfer_dveh,"
                                                                               "dist_to_dropoff,"
                                                                               "dist_from_dropoff,"
                                                                               "pickup_type,"
                                                                               "transfer_type_pveh,"
                                                                               "transfer_type_dveh,"
                                                                               "dropoff_type,"
                                                                               "pickup_id,"
                                                                               "pickup_walking_dist,"
                                                                               "dropoff_id,"
                                                                               "dropoff_walking_dist,"
                                                                               "num_stops_pveh,"
                                                                               "num_stops_dveh,"
                                                                               "veh_dep_time_at_stop_before_pickup,"
                                                                               "veh_dep_time_at_stop_before_transfer_pveh,"
                                                                               "veh_dep_time_at_stop_before_transfer_dveh,"
                                                                               "veh_dep_time_at_stop_before_dropoff,"
                                                                               "accepted,"
                                                                               "cost\n")),
              overallPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::DispatchingPerformanceStats::LOGGER_NAME,
                                                 "request_id," +
                                                 std::string(stats::DispatchingPerformanceStats::LOGGER_COLS) +
                                                 "\n")),
              initializationPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::InitializationPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::InitializationPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              ellipticBchPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::EllipticBCHPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::EllipticBCHPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              pdDistancesPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::PDDistancesPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::PDDistancesPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              ordPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::OrdAssignmentsPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::OrdAssignmentsPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              pbnsPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::PbnsAssignmentsPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::PbnsAssignmentsPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              palsPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::PalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::PalsAssignmentsPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              dalsPerfLogger(
                  LogManager<ComponentPerfLoggerT>::getLogger(stats::DalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                              "request_id," +
                                                              std::string(
                                                                  stats::DalsAssignmentsPerformanceStats::LOGGER_COLS) +
                                                              "\n")),
              updatePerfLogger(LogManager<ComponentPerfLoggerT>::getLogger(stats::UpdatePerformanceStats::LOGGER_NAME,
                                                                           "request_id," +
                                                                           std::string(
                                                                               stats::UpdatePerformanceStats::LOGGER_COLS)
                                                                           +
                                                                           "\n")),
              ellipseReconstructionPerfLogger(
                  LogManager<LoggerT>::getLogger(stats::EllipseReconstructionStats::LOGGER_NAME,
                                                 "request_id, " +
                                                 std::string(stats::EllipseReconstructionStats::LOGGER_COLS))),
              ordinaryTransferPerfLogger(LogManager<LoggerT>::getLogger(
                  stats::AssignmentsWithOrdinaryTransferPerformanceStats::LOGGER_NAME,
                  "request_id," +
                  std::string(stats::AssignmentsWithOrdinaryTransferPerformanceStats::LOGGER_COLS))),

              transferALSPVehPerfLogger(LogManager<LoggerT>::getLogger(
                  stats::AssignmentsWithTransferALSPVehPerformanceStats::LOGGER_NAME,
                  "request_id," +
                  std::string(stats::AssignmentsWithTransferALSPVehPerformanceStats::LOGGER_COLS))),

              transferALSDVehPerfLogger(LogManager<LoggerT>::getLogger(
                  stats::AssignmentsWithTransferALSDVehPerformanceStats::LOGGER_NAME,
                  "request_id," +
                  std::string(stats::AssignmentsWithTransferALSDVehPerformanceStats::LOGGER_COLS))),

              assignmentsCostLogger(LogManager<LoggerT>::getLogger(stats::AssignmentCostStats::LOGGER_NAME,
                                                                   "request_id," +
                                                                   std::string(
                                                                       stats::AssignmentCostStats::LOGGER_COLS))),
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
                                                                        "\n")) {
            // Initialize last stop state for initial locations of vehicles
            if constexpr (BATCHED_DISPATCHING) {
                for (const auto &veh: fleet) {
                    lastStopBucketsEnv.addIdleBucketEntryInsertions(veh.vehicleId);
                }
                LastStopBucketUpdateStats stats;
                lastStopBucketsEnv.commitEntryInsertionsAndDeletions(stats);
            } else {
                stats::UpdatePerformanceStats stats{};
                for (const auto &veh: fleet) {
                    lastStopBucketsEnv.generateIdleBucketEntries(veh, stats);
                }
            }
        }

        template<typename AsgnFinderResponseT>
        std::vector<int> getVehiclesAffectedByAssignmentWithoutTransfer(
            const Assignment &asgn, const AsgnFinderResponseT &asgnFinderResponse) {
            detourComputer.computeDetours(asgn, asgnFinderResponse);
            std::vector<int> affectedVehicles(detourComputer.vehiclesSeenInLastOperation.begin(), detourComputer.vehiclesSeenInLastOperation.end());
            detourComputer.computeMaxArrTimes(asgn, asgnFinderResponse);
            affectedVehicles.insert(affectedVehicles.end(), detourComputer.vehiclesSeenInLastOperation.begin(), detourComputer.vehiclesSeenInLastOperation.end());
            return affectedVehicles;
        }

        template<typename AsgnFinderResponseT>
        std::vector<int> getVehiclesAffectedByAssignmentWithTransfer(const AssignmentWithTransfer &asgn,
                                                                     const AsgnFinderResponseT &asgnFinderResponse) {
            detourComputer.computeDetours(asgn, asgnFinderResponse);
            std::vector<int> affectedVehicles(detourComputer.vehiclesSeenInLastOperation.begin(), detourComputer.vehiclesSeenInLastOperation.end());
            detourComputer.computeMaxArrTimes(asgn, asgnFinderResponse);
            affectedVehicles.insert(affectedVehicles.end(), detourComputer.vehiclesSeenInLastOperation.begin(), detourComputer.vehiclesSeenInLastOperation.end());
            return affectedVehicles;
        }

        template<typename AsgnFinderResponseT>
        void insertSingleBestAssignment(const AsgnFinderResponseT &asgnFinderResponse,
                                        Subset &vehiclesWithChangesInRoute,
                                        stats::DispatchingPerformanceStats &stats) {
            lastStopsWithChangedDepTime.clear();
            vehiclesWithChangesInRoute.clear();
            if (asgnFinderResponse.improvementThroughTransfer()) {
                insertSingleBestAssignmentWithTransfer(asgnFinderResponse, vehiclesWithChangesInRoute, stats.updateStats);
            } else {
                insertSingleBestAssignmentWithoutTransfer(asgnFinderResponse, vehiclesWithChangesInRoute, stats.updateStats);
            }

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of any vehicle with changes in its route.
            if constexpr (AreEllipticBucketsSortedByRemainingLeeway) {
                for (const auto &vehIdWithChanges: vehiclesWithChangesInRoute) {
                    ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(fleet[vehIdWithChanges], stats.updateStats);
                    ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(fleet[vehIdWithChanges], stats.updateStats);
                }
            }

            // For all vehicles for which the departure time at their last stop changed, we need to update the last
            // stop buckets to reflect this.
            for (const auto &lastStopIdWithChangedDepTime: lastStopsWithChangedDepTime) {
                const auto vehIdLastStopUpdate = routeState.vehicleIdOf(lastStopIdWithChangedDepTime);
                const int numStopsLastStopUpdate = routeState.numStopsOf(vehIdLastStopUpdate);
                lastStopBucketsEnv.updateBucketEntries(fleet[vehIdLastStopUpdate], numStopsLastStopUpdate - 1, stats.updateStats);
            }

            writePerformanceLogs(asgnFinderResponse, stats);
        }

        template<typename AsgnFinderResponseT>
        void insertSingleBestAssignmentWithoutTransfer(const AsgnFinderResponseT &asgnFinderResponse,
                                                       Subset &vehiclesWithChangesInRoute,
                                                       stats::UpdatePerformanceStats &stats) {
            Timer timer;
            const Assignment &asgn = asgnFinderResponse.getBestAssignmentWithoutTransfer();
            KASSERT(asgn.vehicle != nullptr);


            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = routeState.numStopsOf(vehId);

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
            using namespace time_utils;
            const auto asgnDepTimeAtPickup = getActualDepTimeAtPickup(asgn, asgnFinderResponse, routeState);
            const auto asgnInitialPickupDetour = calcInitialPickupDetour(
                asgn, asgnDepTimeAtPickup, asgnFinderResponse, routeState);
            const auto asgnDropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            const auto asgnArrTimeAtDropoff = getArrTimeAtDropoff(asgnDepTimeAtPickup, asgn, asgnInitialPickupDetour,
                                                                  asgnDropoffAtExistingStop, routeState);
            const auto latestVehDepTimeAtPickup = asgnFinderResponse.getHardConstraintMaxDepTimeAtPickup(
                asgnDepTimeAtPickup);
            const auto asgnTripTime = asgnArrTimeAtDropoff + asgn.dropoff.walkingDist - asgnFinderResponse.
                                      originalRequest.requestTime;
            const auto latestVehArrTimeAtDropoff = asgnFinderResponse.getHardConstraintMaxArrTimeAtDropoff(
                asgn.dropoff, asgnTripTime);

            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, asgnFinderResponse, lastStopsWithChangedDepTime,
                                                                 vehiclesWithChangesInRoute, latestVehDepTimeAtPickup,
                                                                 latestVehArrTimeAtDropoff);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.updateRoutesTime += routeUpdateTime;

            if (rerouteVehicle) {
                // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
                // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
                // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
                // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
                // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
                LIGHT_KASSERT(loc.depTimeAtHead >= asgnFinderResponse.dispatchingTime);
                routeState.createIntermediateStopForReroute(vehId, loc.location, asgnFinderResponse.dispatchingTime,
                                                            loc.depTimeAtHead);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, 1, stats);
                ++pickupIndex;
                ++dropoffIndex;
            }

            // TODO: Memorize only which stops to generate elliptic bucket entries for, then do searches for finding
            //  specific entry insertions in parallel.
            updateEllipticBucketsForSingleAssignment(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx, pickupIndex,
                                                     dropoffIndex, rerouteVehicle,
                                                     stats);
            updateLastStopBucketsForSingleAssignment(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx, pickupIndex,
                                                     dropoffIndex, rerouteVehicle, stats);

            KASSERT(validateRouteDistances(vehId, rerouteVehicle? 1 : 0));

            const int pickupStopId = routeState.stopIdsFor(vehId)[pickupIndex];
            const int dropoffStopId = routeState.stopIdsFor(vehId)[dropoffIndex];

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, dropoffStopId);
        }

        template<typename AsgnFinderResponseT>
        void
        insertSingleBestAssignmentWithTransfer(const AsgnFinderResponseT &asgnFinderResponse,
                                               Subset &vehiclesWithChangesInRoute,
                                               stats::UpdatePerformanceStats &stats) {
            Timer timer;

            const AssignmentWithTransfer &asgn = asgnFinderResponse.getBestAssignmentWithTransfer();
            KASSERT(asgn.pVeh != nullptr);
            KASSERT(asgn.dVeh != nullptr);

            const auto pVehId = asgn.pVeh->vehicleId;
            const auto dVehId = asgn.dVeh->vehicleId;

            const auto numStopsBeforePVeh = routeState.numStopsOf(pVehId);
            const auto numStopsBeforeDVeh = routeState.numStopsOf(dVehId);

            // If the pickup vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
            // current location before changing the route, and later add the intermediate stop after changing the route.
            bool reroutePVeh = asgn.pickupIdx == 0 && numStopsBeforePVeh > 1 &&
                               routeState.schedDepTimesFor(pVehId)[0] < asgnFinderResponse.dispatchingTime;
            VehicleLocation pVehLoc;
            if (reroutePVeh) {
                pVehLoc = vehicleLocator.computeCurrentLocation(*asgn.pVeh, asgnFinderResponse.dispatchingTime);
                // If vehicle is currently already at edge that constitutes the next stop, we do not have to reroute
                // and create an intermediate stop (as the intermediate stop would be at the same location).
                reroutePVeh = routeState.stopLocationsFor(pVehId)[1] != pVehLoc.location;
            }

            // If the vehicle dropoff has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
            // current location before changing the route, and later add the intermediate stop after changing the route.
            bool rerouteDVeh = asgn.transferIdxDVeh == 0 && numStopsBeforeDVeh > 1 &&
                               routeState.schedDepTimesFor(dVehId)[0] < asgnFinderResponse.dispatchingTime;
            VehicleLocation dVehLoc;
            if (rerouteDVeh) {
                dVehLoc = vehicleLocator.computeCurrentLocation(*asgn.dVeh, asgnFinderResponse.dispatchingTime);
                // If vehicle is currently already at edge that constitutes the next stop, we do not have to reroute
                // and create an intermediate stop (as the intermediate stop would be at the same location).
                rerouteDVeh = routeState.stopLocationsFor(dVehId)[1] != dVehLoc.location;
            }

            using namespace time_utils;
            detourComputer.computeDetours(asgn, asgnFinderResponse);
            const int asgnDepTimeAtPickup = getActualDepTimeAtPickup(asgn, asgnFinderResponse, routeState);
            const bool transferAtStopPVeh = isTransferAtExistingStopPVeh(asgn, routeState);
            const int arrTimeAtTransferPoint = computeRiderArrTimeAtTransfer(
                asgn, asgnDepTimeAtPickup, transferAtStopPVeh,
                detourComputer, routeState);
            const int depTimeAtTransferPoint = computeDVehDepTimeAtTransfer(asgn, arrTimeAtTransferPoint,
                                                                            detourComputer, routeState,
                                                                            asgnFinderResponse);
            const int asgnArrTimeAtDropoff = computeArrTimeAtDropoffAfterTransfer(asgn, depTimeAtTransferPoint,
                detourComputer, routeState);
            const auto latestVehDepTimeAtPickup = asgnFinderResponse.getHardConstraintMaxDepTimeAtPickup(
                asgnDepTimeAtPickup);
            const auto asgnTripTime = asgnArrTimeAtDropoff + asgn.dropoff.walkingDist - asgnFinderResponse.
                                      originalRequest.requestTime;
            const auto latestVehArrTimeAtDropoff = asgnFinderResponse.getHardConstraintMaxArrTimeAtDropoff(
                asgn.dropoff, asgnTripTime);

            const bool pickupAtExistingStop = isPickupAtExistingStop(asgn.pickup, pVehId,
                                                                     asgnFinderResponse.dispatchingTime,
                                                                     asgn.pickupIdx,
                                                                     routeState);
            const bool transferAtExistingStopPVeh = isTransferAtExistingStopPVeh(asgn, routeState);
            const bool transferAtExistingStopDVeh = isTransferAtExistingStopDVeh(
                asgn, asgnFinderResponse.dispatchingTime, routeState);
            const bool dropoffAtExistingStopDVeh = isDropoffAtExistingStop(asgn, routeState);
            timer.restart();

            auto [pIdxPVeh, dIdxPVeh, pIdxDVeh, dIdxDVeh] =
                    routeState.insert(asgn, arrTimeAtTransferPoint, asgnFinderResponse,
                                      lastStopsWithChangedDepTime, vehiclesWithChangesInRoute, latestVehDepTimeAtPickup,
                                      latestVehArrTimeAtDropoff);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.updateRoutesTime += routeUpdateTime;

            if (reroutePVeh) {
                // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
                // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
                // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
                // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
                // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
                LIGHT_KASSERT(pVehLoc.depTimeAtHead >= asgnFinderResponse.dispatchingTime);
                routeState.createIntermediateStopForReroute(pVehId, pVehLoc.location,
                                                            asgnFinderResponse.dispatchingTime,
                                                            pVehLoc.depTimeAtHead);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.pVeh, 1, stats);
                ++pIdxPVeh;
                ++dIdxPVeh;
            }

            if (rerouteDVeh) {
                // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
                // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
                // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
                // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
                // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
                LIGHT_KASSERT(dVehLoc.depTimeAtHead >= asgnFinderResponse.dispatchingTime);
                routeState.createIntermediateStopForReroute(dVehId, dVehLoc.location,
                                                            asgnFinderResponse.dispatchingTime,
                                                            dVehLoc.depTimeAtHead);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.dVeh, 1, stats);
                ++pIdxDVeh;
                ++dIdxDVeh;
            }

            // TODO: Memorize only which stops to generate elliptic bucket entries for, then do searches for finding
            //  specific entry insertions in parallel.
            updateEllipticBucketsForSingleAssignment(pVehId, asgn.pickupIdx, asgn.transferIdxPVeh, pIdxPVeh, dIdxPVeh,
                                                     reroutePVeh,
                                                     stats);
            updateLastStopBucketsForSingleAssignment(pVehId, asgn.pickupIdx, asgn.transferIdxPVeh, pIdxPVeh, dIdxPVeh,
                                                     reroutePVeh,
                                                     stats);

            updateEllipticBucketsForSingleAssignment(dVehId, asgn.transferIdxDVeh, asgn.dropoffIdx, pIdxDVeh, dIdxDVeh,
                                                     rerouteDVeh,
                                                     stats);
            updateLastStopBucketsForSingleAssignment(dVehId, asgn.transferIdxDVeh, asgn.dropoffIdx, pIdxDVeh, dIdxDVeh,
                                                     rerouteDVeh,
                                                     stats);

            KASSERT(validateRouteDistances(pVehId, reroutePVeh? 1 : 0), routeState.printRouteOf(pVehId));
            KASSERT(validateRouteDistances(dVehId, rerouteDVeh? 1 : 0), routeState.printRouteOf(dVehId));
            KASSERT(
                routeState.assertRoutePVeh(asgn, reroutePVeh, pickupAtExistingStop,
                    transferAtExistingStopPVeh, asgnDepTimeAtPickup, arrTimeAtTransferPoint));
            KASSERT(
                routeState.assertRouteDVeh(asgn, rerouteDVeh, transferAtExistingStopDVeh,
                    dropoffAtExistingStopDVeh, arrTimeAtTransferPoint,
                    depTimeAtTransferPoint, asgnArrTimeAtDropoff));

            const int pickupStopId = routeState.stopIdsFor(pVehId)[pIdxPVeh];
            const int transferStopIdPVeh = routeState.stopIdsFor(pVehId)[dIdxPVeh];
            const int transferStopIdDVeh = routeState.stopIdsFor(dVehId)[pIdxDVeh];
            const int dropoffStopId = routeState.stopIdsFor(dVehId)[dIdxDVeh];

            KASSERT(routeState.vehicleIdOf(pickupStopId) == pVehId);
            KASSERT(routeState.vehicleIdOf(transferStopIdPVeh) == pVehId);
            KASSERT(routeState.vehicleIdOf(transferStopIdDVeh) == dVehId);
            KASSERT(routeState.vehicleIdOf(dropoffStopId) == dVehId);

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, transferStopIdPVeh);
            pathTracker.registerPdEventsForBestAssignment(transferStopIdDVeh, dropoffStopId);
        }


        template<typename AsgnFinderResponseT>
        void insertBestAssignmentWithoutTransferAsPartOfBatch(const AsgnFinderResponseT &asgnFinderResponse,
                                                              const int now,
                                                              Subset &vehiclesWithChangesInRoute,
                                                              // Stops that require elliptic bucket entry changes
                                                              std::vector<int> &stopIdsToGenerateSourceEntriesFor,
                                                              std::vector<int> &stopIdsToGenerateTargetEntriesFor,
                                                              // Stops that require last stop bucket entry changes
                                                              std::vector<int> &stopIdsToRemoveIdleEntriesFor,
                                                              std::vector<int> &stopIdsToRemoveNonIdleEntriesFor,
                                                              std::vector<int> &vehIdsToGenerateNonIdleEntriesFor,
                                                              stats::UpdatePerformanceStats &stats
        ) {
            Timer internalRouteStateUpdateTimer;
            const Assignment &asgn = asgnFinderResponse.getBestAssignmentWithoutTransfer();
            KASSERT(asgn.vehicle != nullptr);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = routeState.numStopsOf(vehId);

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
            // current location before changing the route, and later add the intermediate stop after changing the route.
            bool rerouteVehicle = asgn.pickupStopIdx == 0 && numStopsBefore > 1 &&
                                  routeState.schedDepTimesFor(vehId)[0] < asgnFinderResponse.dispatchingTime;
            VehicleLocation loc;
            if (rerouteVehicle) {
                loc = vehicleLocator.computeCurrentLocation(*asgn.vehicle, asgnFinderResponse.dispatchingTime);
            }

            internalRouteStateUpdateTimer.restart();

            using namespace time_utils;
            using namespace time_utils;
            const auto asgnDepTimeAtPickup = getActualDepTimeAtPickup(asgn, asgnFinderResponse, routeState);
            const auto asgnInitialPickupDetour = calcInitialPickupDetour(
                asgn, asgnDepTimeAtPickup, asgnFinderResponse, routeState);
            const auto asgnDropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            const auto asgnArrTimeAtDropoff = getArrTimeAtDropoff(asgnDepTimeAtPickup, asgn, asgnInitialPickupDetour,
                                                                  asgnDropoffAtExistingStop, routeState);
            const auto latestVehDepTimeAtPickup = asgnFinderResponse.getHardConstraintMaxDepTimeAtPickup(
                asgnDepTimeAtPickup);
            const auto asgnTripTime = asgnArrTimeAtDropoff + asgn.dropoff.walkingDist - asgnFinderResponse.
                                      originalRequest.requestTime;
            const auto latestVehArrTimeAtDropoff = asgnFinderResponse.getHardConstraintMaxArrTimeAtDropoff(
                asgn.dropoff, asgnTripTime);

            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, asgnFinderResponse, lastStopsWithChangedDepTime,
                                                                 vehiclesWithChangesInRoute, latestVehDepTimeAtPickup,
                                                                 latestVehArrTimeAtDropoff, loc);
            const auto routeUpdateTime = internalRouteStateUpdateTimer.elapsed<std::chrono::nanoseconds>();
            stats.updateRoutesTime += routeUpdateTime;

            markStopsWithNewEllipticEntries(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                            pickupIndex, dropoffIndex, rerouteVehicle, stats,
                                            stopIdsToGenerateSourceEntriesFor,
                                            stopIdsToGenerateTargetEntriesFor);

            markStopsAndVehiclesWithLastStopBucketUpdates(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                                          pickupIndex, dropoffIndex, rerouteVehicle, stats,
                                                          stopIdsToRemoveIdleEntriesFor,
                                                          stopIdsToRemoveNonIdleEntriesFor,
                                                          vehIdsToGenerateNonIdleEntriesFor);

            if (rerouteVehicle && loc.location != routeState.stopLocationsFor(vehId)[1]) {
                // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
                // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
                // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
                // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
                // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
                LIGHT_KASSERT(loc.depTimeAtHead >= now);
                routeState.createIntermediateStopForReroute(vehId, loc.location, now, loc.depTimeAtHead);
                stopIdsToGenerateSourceEntriesFor.push_back(routeState.stopIdsFor(vehId)[1]);
                ++pickupIndex;
                ++dropoffIndex;
            }

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(routeState.stopIdsFor(vehId)[pickupIndex],
                                                          routeState.stopIdsFor(vehId)[dropoffIndex]);
        }

        template<typename AsgnFinderResponseT>
        void
        insertBestAssignmentWithTransferAsPartOfBatch(const AsgnFinderResponseT &asgnFinderResponse,
                                                      const int now,
                                                      Subset &vehiclesWithChangesInRoute,
                                                      // Stops that require elliptic bucket entry changes
                                                      std::vector<int> &stopIdsToGenerateSourceEntriesFor,
                                                      std::vector<int> &stopIdsToGenerateTargetEntriesFor,
                                                      // Stops that require last stop bucket entry changes
                                                      std::vector<int> &stopIdsToRemoveIdleEntriesFor,
                                                      std::vector<int> &stopIdsToRemoveNonIdleEntriesFor,
                                                      std::vector<int> &vehIdsToGenerateNonIdleEntriesFor,
                                                      stats::UpdatePerformanceStats &stats) {
            Timer timer;

            const AssignmentWithTransfer &asgn = asgnFinderResponse.getBestAssignmentWithTransfer();
            KASSERT(asgn.pVeh != nullptr);
            KASSERT(asgn.dVeh != nullptr);

            const auto pVehId = asgn.pVeh->vehicleId;
            const auto dVehId = asgn.dVeh->vehicleId;

            const auto numStopsBeforePVeh = routeState.numStopsOf(pVehId);
            const auto numStopsBeforeDVeh = routeState.numStopsOf(dVehId);

            // If the pickup vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
            // current location before changing the route, and later add the intermediate stop after changing the route.
            bool reroutePVeh = asgn.pickupIdx == 0 && numStopsBeforePVeh > 1 &&
                               routeState.schedDepTimesFor(pVehId)[0] < asgnFinderResponse.dispatchingTime;
            VehicleLocation pVehLoc;
            if (reroutePVeh) {
                pVehLoc = vehicleLocator.computeCurrentLocation(*asgn.pVeh, asgnFinderResponse.dispatchingTime);
                // If vehicle is currently already at edge that constitutes the next stop, we do not have to reroute
                // and create an intermediate stop (as the intermediate stop would be at the same location).
                reroutePVeh = routeState.stopLocationsFor(pVehId)[1] != pVehLoc.location;
            }

            // If the vehicle dropoff has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting. We have to compute the vehicle's
            // current location before changing the route, and later add the intermediate stop after changing the route.
            bool rerouteDVeh = asgn.transferIdxDVeh == 0 && numStopsBeforeDVeh > 1 &&
                               routeState.schedDepTimesFor(dVehId)[0] < asgnFinderResponse.dispatchingTime;
            VehicleLocation dVehLoc;
            if (rerouteDVeh) {
                dVehLoc = vehicleLocator.computeCurrentLocation(*asgn.dVeh, asgnFinderResponse.dispatchingTime);
                // If vehicle is currently already at edge that constitutes the next stop, we do not have to reroute
                // and create an intermediate stop (as the intermediate stop would be at the same location).
                rerouteDVeh = routeState.stopLocationsFor(dVehId)[1] != dVehLoc.location;
            }

            using namespace time_utils;
            detourComputer.computeDetours(asgn, asgnFinderResponse);
            const int asgnDepTimeAtPickup = getActualDepTimeAtPickup(asgn, asgnFinderResponse, routeState);
            const bool transferAtStopPVeh = isTransferAtExistingStopPVeh(asgn, routeState);
            const int arrTimeAtTransferPoint = computeRiderArrTimeAtTransfer(
                asgn, asgnDepTimeAtPickup, transferAtStopPVeh,
                detourComputer, routeState);
            const int depTimeAtTransferPoint = computeDVehDepTimeAtTransfer(asgn, arrTimeAtTransferPoint,
                                                                            detourComputer, routeState,
                                                                            asgnFinderResponse);
            const int asgnArrTimeAtDropoff = computeArrTimeAtDropoffAfterTransfer(asgn, depTimeAtTransferPoint,
                detourComputer, routeState);
            const auto latestVehDepTimeAtPickup = asgnFinderResponse.getHardConstraintMaxDepTimeAtPickup(
                asgnDepTimeAtPickup);
            const auto asgnTripTime = asgnArrTimeAtDropoff + asgn.dropoff.walkingDist - asgnFinderResponse.
                                      originalRequest.requestTime;
            const auto latestVehArrTimeAtDropoff = asgnFinderResponse.getHardConstraintMaxArrTimeAtDropoff(
                asgn.dropoff, asgnTripTime);

            const bool pickupAtExistingStop = isPickupAtExistingStop(asgn.pickup, pVehId, now, asgn.pickupIdx,
                                                                     routeState);
            const bool transferAtExistingStopPVeh = isTransferAtExistingStopPVeh(asgn, routeState);
            const bool transferAtExistingStopDVeh = isTransferAtExistingStopDVeh(asgn, now, routeState);
            const bool dropoffAtExistingStopDVeh = isDropoffAtExistingStop(asgn, routeState);
            timer.restart();

            auto [pIdxPVeh, dIdxPVeh, pIdxDVeh, dIdxDVeh] =
                    routeState.insert(asgn, arrTimeAtTransferPoint, asgnFinderResponse,
                                      lastStopsWithChangedDepTime, vehiclesWithChangesInRoute, latestVehDepTimeAtPickup,
                                      latestVehArrTimeAtDropoff);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.updateRoutesTime += routeUpdateTime;

            if (reroutePVeh) {
                // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
                // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
                // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
                // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
                // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
                LIGHT_KASSERT(pVehLoc.depTimeAtHead >= asgnFinderResponse.dispatchingTime);
                routeState.createIntermediateStopForReroute(pVehId, pVehLoc.location,
                                                            asgnFinderResponse.dispatchingTime,
                                                            pVehLoc.depTimeAtHead);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.pVeh, 1, stats);
                ++pIdxPVeh;
                ++dIdxPVeh;
            }

            if (rerouteDVeh) {
                // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
                // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
                // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
                // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
                // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
                LIGHT_KASSERT(dVehLoc.depTimeAtHead >= asgnFinderResponse.dispatchingTime);
                routeState.createIntermediateStopForReroute(dVehId, dVehLoc.location,
                                                            asgnFinderResponse.dispatchingTime,
                                                            dVehLoc.depTimeAtHead);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.dVeh, 1, stats);
                ++pIdxDVeh;
                ++dIdxDVeh;
            }

            // TODO: Memorize only which stops to generate elliptic bucket entries for, then do searches for finding
            //  specific entry insertions in parallel.
            markStopsWithNewEllipticEntries(pVehId, asgn.pickupIdx, asgn.transferIdxPVeh, pIdxPVeh, dIdxPVeh,
                                            reroutePVeh, stats,
                                            stopIdsToGenerateSourceEntriesFor,
                                            stopIdsToGenerateTargetEntriesFor);
            markStopsAndVehiclesWithLastStopBucketUpdates(pVehId, asgn.pickupIdx, asgn.transferIdxPVeh, pIdxPVeh,
                                                          dIdxPVeh,
                                                          reroutePVeh, stats,
                                                          stopIdsToRemoveIdleEntriesFor,
                                                          stopIdsToRemoveNonIdleEntriesFor,
                                                          vehIdsToGenerateNonIdleEntriesFor);

            markStopsWithNewEllipticEntries(dVehId, asgn.transferIdxDVeh, asgn.dropoffIdx, pIdxDVeh, dIdxDVeh,
                                            rerouteDVeh, stats,
                                            stopIdsToGenerateSourceEntriesFor,
                                            stopIdsToGenerateTargetEntriesFor);
            markStopsAndVehiclesWithLastStopBucketUpdates(dVehId, asgn.transferIdxDVeh, asgn.dropoffIdx, pIdxDVeh,
                                                          dIdxDVeh,
                                                          rerouteDVeh, stats,
                                                          stopIdsToRemoveIdleEntriesFor,
                                                          stopIdsToRemoveNonIdleEntriesFor,
                                                          vehIdsToGenerateNonIdleEntriesFor);

            KASSERT(validateRouteDistances(pVehId, reroutePVeh? 1 : 0), routeState.printRouteOf(pVehId));
            KASSERT(validateRouteDistances(dVehId, rerouteDVeh? 1 : 0), routeState.printRouteOf(dVehId));
            KASSERT(
                routeState.assertRoutePVeh(asgn, reroutePVeh, pickupAtExistingStop,
                    transferAtExistingStopPVeh, asgnDepTimeAtPickup, arrTimeAtTransferPoint));
            KASSERT(
                routeState.assertRouteDVeh(asgn, rerouteDVeh, transferAtExistingStopDVeh,
                    dropoffAtExistingStopDVeh, arrTimeAtTransferPoint,
                    depTimeAtTransferPoint, asgnArrTimeAtDropoff));

            const int pickupStopId = routeState.stopIdsFor(pVehId)[pIdxPVeh];
            const int transferStopIdPVeh = routeState.stopIdsFor(pVehId)[dIdxPVeh];
            const int transferStopIdDVeh = routeState.stopIdsFor(dVehId)[pIdxDVeh];
            const int dropoffStopId = routeState.stopIdsFor(dVehId)[dIdxDVeh];

            KASSERT(routeState.vehicleIdOf(pickupStopId) == pVehId);
            KASSERT(routeState.vehicleIdOf(transferStopIdPVeh) == pVehId);
            KASSERT(routeState.vehicleIdOf(transferStopIdDVeh) == dVehId);
            KASSERT(routeState.vehicleIdOf(dropoffStopId) == dVehId);

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, transferStopIdPVeh);
            pathTracker.registerPdEventsForBestAssignment(transferStopIdDVeh, dropoffStopId);
        }


        template<typename AssignmentFinderResponsesT, typename StatssT>
        void insertBatchOfBestAssignments(const AssignmentFinderResponsesT &asgnFinderResponses, StatssT &statss,
                                          const int now, const int iteration,
                                          Subset &vehiclesWithChangesInRoute) requires (BATCHED_DISPATCHING) {
            KASSERT(std::distance(asgnFinderResponses.begin(), asgnFinderResponses.end()) ==
                std::distance(statss.begin(), statss.end()));

            // Stops that require elliptic bucket entry changes
            std::vector<int> stopIdsToGenerateSourceEntriesFor;
            std::vector<int> stopIdsToGenerateTargetEntriesFor;

            // Stops that require last stop bucket entry changes
            std::vector<int> stopIdsToRemoveIdleEntriesFor;
            std::vector<int> stopIdsToRemoveNonIdleEntriesFor;
            std::vector<int> vehIdsToGenerateNonIdleEntriesFor;

            lastStopsWithChangedDepTime.clear();

            Timer timer;
            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                const auto asgnFinderResponse = *respIt;
                auto &stats = statsIt->updateStats;

                if (asgnFinderResponse.improvementThroughTransfer()) {
                    insertBestAssignmentWithTransferAsPartOfBatch(asgnFinderResponse, now,
                                                                  vehiclesWithChangesInRoute,
                                                                  stopIdsToGenerateSourceEntriesFor,
                                                                  stopIdsToGenerateTargetEntriesFor,
                                                                  stopIdsToRemoveIdleEntriesFor,
                                                                  stopIdsToRemoveNonIdleEntriesFor,
                                                                  vehIdsToGenerateNonIdleEntriesFor, stats);
                } else {
                    insertBestAssignmentWithoutTransferAsPartOfBatch(asgnFinderResponse,
                                                                     now,
                                                                     vehiclesWithChangesInRoute,
                                                                     stopIdsToGenerateSourceEntriesFor,
                                                                     stopIdsToGenerateTargetEntriesFor,
                                                                     stopIdsToRemoveIdleEntriesFor,
                                                                     stopIdsToRemoveNonIdleEntriesFor,
                                                                     vehIdsToGenerateNonIdleEntriesFor, stats);
                }
            }
            const auto updateRouteStateTime = timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            // Find all elliptic bucket entry insertions necessary for assignments.
            tbb::parallel_for(0ul, stopIdsToGenerateSourceEntriesFor.size(), [&](const size_t i) {
                const auto &stopId = stopIdsToGenerateSourceEntriesFor[i];
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopIndex = routeState.stopPositionOf(stopId);
                ellipticBucketsEnv.addSourceBucketEntryInsertions(vehId, stopIndex);
            });
            tbb::parallel_for(0ul, stopIdsToGenerateTargetEntriesFor.size(), [&](const size_t i) {
                const auto &stopId = stopIdsToGenerateTargetEntriesFor[i];
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopIndex = routeState.stopPositionOf(stopId);
                ellipticBucketsEnv.addTargetBucketEntryInsertions(vehId, stopIndex);
            });
            const auto findEllipticBucketEntryInsertionsTime = timer.elapsed<std::chrono::nanoseconds>();
            const auto numEllipticBucketEntryInsertions = ellipticBucketsEnv.numPendingEntryInsertions();

            timer.restart();
            ellipticBucketsEnv.commitEntryInsertions(); // Batched update to elliptic buckets for new entries.
            const auto performEllipticBucketEntryInsertionsTime = timer.elapsed<std::chrono::nanoseconds>();

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

            // Remove all vehicles that have new last stop from lastStopsWithChangedDepTime as new last stop does not
            // need update
            for (const auto &stopId : stopIdsToRemoveIdleEntriesFor)
                lastStopsWithChangedDepTime.remove(stopId);
            for (const auto &stopId : stopIdsToRemoveNonIdleEntriesFor)
                lastStopsWithChangedDepTime.remove(stopId);
            for (const auto &vehId : vehIdsToGenerateNonIdleEntriesFor) {
                const int stopId = routeState.stopIdsFor(vehId)[routeState.numStopsOf(vehId) - 1];
                lastStopsWithChangedDepTime.remove(stopId);
            }
            tbb::parallel_for(0, lastStopsWithChangedDepTime.size(), [&](const int i) {
                const auto &stopId = *(lastStopsWithChangedDepTime.begin() + i);
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopIndex = routeState.stopPositionOf(stopId);
                KASSERT(stopIndex == routeState.numStopsOf(vehId) - 1, "Only last stops should be in lastStopsWithChangedDepTime");
                lastStopBucketsEnv.addBucketEntryInsertionsAndDeletionsForUpdatedSchedule(vehId, stopIndex);
            });
            const auto findLastStopBucketEntryInsertionsAndDeletionsTime = timer.elapsed<std::chrono::nanoseconds>();
            const auto numLastStopBucketEntryInsertionsAndDeletions =
                    lastStopBucketsEnv.numPendingEntryInsertions() + lastStopBucketsEnv.numPendingEntryDeletions();
            lastStopBucketUpdateStats.findInsertionsAndDeletionsTime =
                    findLastStopBucketEntryInsertionsAndDeletionsTime;
            lastStopBucketUpdateStats.numInsertions = lastStopBucketsEnv.numPendingEntryInsertions();
            lastStopBucketUpdateStats.numDeletions = lastStopBucketsEnv.numPendingEntryDeletions();

            timer.restart();
            lastStopBucketsEnv.commitEntryInsertionsAndDeletions(
                lastStopBucketUpdateStats); // Batched update to last stop bucket entries.
            const auto performLastStopBucketEntryInsertionsAndDeletionsTime = timer.elapsed<std::chrono::nanoseconds>();


            timer.restart();
            // Update leeway in elliptic bucket entries of stops in routes of affected vehicles.
            int64_t totalNumEntriesScannedForSourceUpdates = 0;
            int64_t totalNumEntriesScannedForTargetUpdates = 0;
            int64_t totelNumVerticesVisitedForSourceUpdates = 0;
            int64_t totalNumVerticesVisitedForTargetUpdates = 0;

            if constexpr (AreEllipticBucketsSortedByRemainingLeeway) {
                ellipticBucketsEnv.addSourceBucketLeewayUpdates(vehiclesWithChangesInRoute,
                                                                totalNumEntriesScannedForSourceUpdates,
                                                                totelNumVerticesVisitedForSourceUpdates);
                ellipticBucketsEnv.addTargetBucketLeewayUpdates(vehiclesWithChangesInRoute,
                                                                totalNumEntriesScannedForTargetUpdates,
                                                                totalNumVerticesVisitedForTargetUpdates);
                ellipticBucketsEnv.commitSourceBucketLeewayUpdatesParallel(vehiclesWithChangesInRoute);
                ellipticBucketsEnv.commitTargetBucketLeewayUpdatesParallel(vehiclesWithChangesInRoute);
            }
            totalNumEntriesScannedForSourceUpdates *= 2; // entries are scanned again for updates
            totalNumEntriesScannedForTargetUpdates *= 2; // entries are scanned again for updates
            const auto updateLeewaysTime = timer.elapsed<std::chrono::nanoseconds>();

            for (auto respIt = asgnFinderResponses.begin(), statsIt = statss.begin();
                 respIt != asgnFinderResponses.end(); ++respIt, ++statsIt) {
                writePerformanceLogs(*respIt, *statsIt);
            }

            const auto totalTime =
                    updateRouteStateTime + findLastStopBucketEntryInsertionsAndDeletionsTime +
                    performLastStopBucketEntryInsertionsAndDeletionsTime + findEllipticBucketEntryInsertionsTime +
                    performEllipticBucketEntryInsertionsTime + updateLeewaysTime;

            batchInsertLogger << now << ","
                    << iteration << ","
                    << asgnFinderResponses.size() << ","
                    << updateRouteStateTime << ","
                    << findEllipticBucketEntryInsertionsTime << ","
                    << numEllipticBucketEntryInsertions << ","
                    << performEllipticBucketEntryInsertionsTime << ","
                    << findLastStopBucketEntryInsertionsAndDeletionsTime << ","
                    << numLastStopBucketEntryInsertionsAndDeletions << ","
                    << performLastStopBucketEntryInsertionsAndDeletionsTime << ","
                    << 0 << "," // ellipticBucketsEnv.totalNumSourceEntries() << ","
                    << 0 << "," // ellipticBucketsEnv.totalNumTargetEntries() << ","
                    << totalNumEntriesScannedForSourceUpdates << ","
                    << totalNumEntriesScannedForTargetUpdates << ","
                    << updateLeewaysTime << ","
                    << totalTime << "\n";

            lastStopBucketUpdateLogger << lastStopBucketUpdateStats.getLoggerRow() << "\n";
        }

        void commitPendingEllipticBucketEntryDeletions() requires BATCHED_DISPATCHING {
            ellipticBucketsEnv.commitEntryDeletions();
        }

        void commitPendingLastStopBucketEntryInsertionsAndDeletions() requires BATCHED_DISPATCHING {
            LastStopBucketUpdateStats stats;
            lastStopBucketsEnv.commitEntryInsertionsAndDeletions(stats);
        }

        int numPendingEllipticBucketEntryDeletions() requires BATCHED_DISPATCHING {
            return ellipticBucketsEnv.numPendingEntryDeletions();
        }

        bool noPendingEllipticBucketEntryInsertionsOrDeletions() requires BATCHED_DISPATCHING {
            return ellipticBucketsEnv.noPendingEntryInsertionsOrDeletions();
        }


        bool noPendingLastStopBucketEntryInsertionsOrDeletions() requires BATCHED_DISPATCHING {
            return lastStopBucketsEnv.noPendingEntryInsertionsOrDeletions();
        }

        void notifyStopStarted(const Vehicle &veh) {
            // Update buckets and route state
            if constexpr (BATCHED_DISPATCHING) {
                // Memorize elliptic bucket entry deletions for collective update before next batch.
                ellipticBucketsEnv.addSourceBucketEntryDeletions(veh, 0);
                ellipticBucketsEnv.addTargetBucketEntryDeletions(veh, 1);
            } else {
                stats::UpdatePerformanceStats statsPlaceholder{};
                ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0, statsPlaceholder);
                ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1, statsPlaceholder);
            }

            // Forward dependencies of reached stop are fulfilled. Remove them from route state.
            routeState.removeForwardDependencies(veh.vehicleId, 1);

            // Remove old previous stop.
            routeState.removeStartOfCurrentLeg(veh.vehicleId);

            // If vehicle has become idle, update last stop bucket entries
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                if constexpr (BATCHED_DISPATCHING) {
                    // Memorize last stop bucket entry insertions for collective update before next batch.
                    lastStopBucketsEnv.addBucketEntryInsertionsAndDeletionsForVehicleHasBecomeIdle(veh.vehicleId);
                } else {
                    stats::UpdatePerformanceStats statsPlaceholder{};
                    lastStopBucketsEnv.updateBucketEntries(veh, 0, statsPlaceholder);
                }
            }
        }

        void notifyStopCompleted(const Vehicle &veh) {

            // Backward dependencies of completed stop cannot affect it anymore. Remove them from route state.
            routeState.removeBackwardDependencies(veh.vehicleId, 0);

            pathTracker.logCompletedStop(veh);
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            KASSERT(routeState.numStopsOf(vehId) == 1);

            if constexpr (BATCHED_DISPATCHING) {
                // Memorize last stop bucket entry deletion for collective update before next batch.
                lastStopBucketsEnv.addIdleBucketEntryDeletions(veh.vehicleId, 0);
            } else {
                stats::UpdatePerformanceStats finalRemovalStatsPlaceholder{};
                lastStopBucketsEnv.removeIdleBucketEntries(veh, 0, finalRemovalStatsPlaceholder);
            }

            routeState.removeStartOfCurrentLeg(vehId);
        }

        template<typename AsgnFinderResponseT>
        void writeBestAssignmentToLogger(const mode_choice::TransportMode &mode,
                                         const AsgnFinderResponseT &asgnFinderResponse,
                                         const stats::AssignmentCostStats &costStats) {
            // Set up the best assignment for logging
            int numLegs = -1;
            switch (asgnFinderResponse.getBestAsgnType()) {
                case ONE_LEG:
                    numLegs = 1;
                    break;
                case TWO_LEGS:
                    numLegs = 2;
                    break;
                default:
                    numLegs = 0;
            };
            bestAssignmentsOverallLogger
                    << asgnFinderResponse.originalRequest.requestId << ", "
                    << asgnFinderResponse.originalRequest.requestTime << ", "
                    << asgnFinderResponse.originalReqDirectDist << ", "
                    << numLegs << ", "
                    << (mode == mode_choice::TransportMode::Taxi) << ", "
                    << asgnFinderResponse.getBestCost() << "\n";

            bestAssignmentsWithoutUsingVehicleLogger
                    << asgnFinderResponse.originalRequest.requestId << ", "
                    << asgnFinderResponse.originalRequest.requestTime << ", ";

            bestAssignmentsWithoutTransferLogger
                    << asgnFinderResponse.originalRequest.requestId << ", "
                    << asgnFinderResponse.originalRequest.requestTime << ", "
                    << asgnFinderResponse.originalReqDirectDist << ", ";

            bestAssignmentsWithTransferLogger
                    << asgnFinderResponse.originalRequest.requestId << ", "
                    << asgnFinderResponse.originalRequest.requestTime << ", "
                    << asgnFinderResponse.originalReqDirectDist << ", ";

            // Log the cost as well
            assignmentsCostLogger << asgnFinderResponse.originalRequest.requestId << ", "
                    << costStats.getLoggerRow() << "\n";

            // const int costWithoutVehicle = requestState.getBestCostWithoutUsingVehicle();
            const int costWOT = asgnFinderResponse.getCostObjectWithoutTransfer().total;
            const int costWT = asgnFinderResponse.getCostObjectWithTransfer().total;

            // if (costWithoutVehicle >= INFTY) {
            //     bestAssignmentsWithoutUsingVehicleLogger << "-1,-1,inf\n";
            // }

            if (costWOT >= INFTY) {
                bestAssignmentsWithoutTransferLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
            }

            if (costWT >= INFTY) {
                bestAssignmentsWithTransferLogger
                        << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
            }

            using time_utils::getVehDepTimeAtStopForRequest;
            // if (costWithoutVehicle < INFTY) {
            //     bestAssignmentsWithoutUsingVehicleLogger << requestState.getNotUsingVehicleDist() << ", "
            //                                              << requestState.getBestCostWithoutUsingVehicle() << "\n";
            // }

            if (costWOT < INFTY) {
                const auto &bestAsgn = asgnFinderResponse.getBestAssignmentWithoutTransfer();

                const auto &vehId = bestAsgn.vehicle->vehicleId;
                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &vehDepTimeBeforePickup = getVehDepTimeAtStopForRequest(vehId, bestAsgn.pickupStopIdx,
                    asgnFinderResponse, routeState);
                const auto &vehDepTimeBeforeDropoff = getVehDepTimeAtStopForRequest(vehId, bestAsgn.dropoffStopIdx,
                    asgnFinderResponse, routeState);
                bestAssignmentsWithoutTransferLogger
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
                        << (mode == mode_choice::TransportMode::Taxi) << ", "
                        << asgnFinderResponse.getBestCostWithoutTransfer() << "\n";
            }

            if (costWT < INFTY) {
                const auto &bestAsgnWT = asgnFinderResponse.getBestAssignmentWithTransfer();
                const int pVehId = bestAsgnWT.pVeh->vehicleId;
                const int dVehId = bestAsgnWT.dVeh->vehicleId;
                const int pickupIdx = bestAsgnWT.pickupIdx;
                const int transferIdxPVeh = bestAsgnWT.transferIdxPVeh;
                const int transferIdxDVeh = bestAsgnWT.transferIdxDVeh;
                const int dropoffIdx = bestAsgnWT.dropoffIdx;

                const auto &numStopsPVeh = routeState.numStopsOf(pVehId);
                const auto &numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto &vehDepTimeBeforePickupWT = getVehDepTimeAtStopForRequest(pVehId, pickupIdx,
                    asgnFinderResponse, routeState);
                const auto &vehDepTimeBeforeTransferPVeh = getVehDepTimeAtStopForRequest(pVehId, transferIdxPVeh,
                    asgnFinderResponse, routeState);
                const auto &vehDepTimeBeforeTransferDVeh = getVehDepTimeAtStopForRequest(dVehId, transferIdxDVeh,
                    asgnFinderResponse, routeState);
                const auto &vehDepTimeBeforeDropoffWT = getVehDepTimeAtStopForRequest(dVehId, dropoffIdx,
                    asgnFinderResponse, routeState);

                const std::array<std::string, 4> types = {"NOT_SET", "BNS", "ORD", "ALS"};

                const auto pickupType = types[bestAsgnWT.pickupType];
                const auto transferTypePVeh = types[bestAsgnWT.transferTypePVeh];
                const auto transferTypeDVeh = types[bestAsgnWT.transferTypeDVeh];
                const auto dropoffType = types[bestAsgnWT.dropoffType];


                bestAssignmentsWithTransferLogger
                        << pVehId << ", "
                        << dVehId << ", "
                        << pickupIdx << ", "
                        << transferIdxPVeh << ", "
                        << transferIdxDVeh << ", "
                        << dropoffIdx << ", "
                        << bestAsgnWT.distToPickup << ", "
                        << bestAsgnWT.distFromPickup << ", "
                        << bestAsgnWT.distToTransferPVeh << ", "
                        << bestAsgnWT.distFromTransferPVeh << ", "
                        << bestAsgnWT.distToTransferDVeh << ", "
                        << bestAsgnWT.distFromTransferDVeh << ", "
                        << bestAsgnWT.distToDropoff << ", "
                        << bestAsgnWT.distFromDropoff << ", "
                        << pickupType << ", "
                        << transferTypePVeh << ", "
                        << transferTypeDVeh << ", "
                        << dropoffType << ", "
                        << bestAsgnWT.pickup.id << ", "
                        << bestAsgnWT.pickup.walkingDist << ", "
                        << bestAsgnWT.dropoff.id << ", "
                        << bestAsgnWT.dropoff.walkingDist << ", "
                        << numStopsPVeh << ", "
                        << numStopsDVeh << ", "
                        << vehDepTimeBeforePickupWT << ", "
                        << vehDepTimeBeforeTransferPVeh << ", "
                        << vehDepTimeBeforeTransferDVeh << ", "
                        << vehDepTimeBeforeDropoffWT << ", "
                        << (mode == mode_choice::TransportMode::Taxi) << ", "
                        << asgnFinderResponse.getBestCostWithTransfer() << "\n";
            }
        }


        void writePerformanceLogs(const RequestState &requestState, stats::DispatchingPerformanceStats &stats) {
            overallPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.getLoggerRow() << "\n";
            initializationPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.initializationStats.getLoggerRow() << "\n";
            ellipticBchPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.ellipticBchStats.getLoggerRow() << "\n";
            pdDistancesPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.pdDistancesStats.getLoggerRow() << "\n";
            ordPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.ordAssignmentsStats.getLoggerRow() << "\n";
            pbnsPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.pbnsAssignmentsStats.getLoggerRow() << "\n";
            palsPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.palsAssignmentsStats.getLoggerRow() << "\n";
            dalsPerfLogger << requestState.originalRequest.requestId << ","
                    << stats.dalsAssignmentsStats.getLoggerRow() << "\n";
            updatePerfLogger << requestState.originalRequest.requestId << ","
                    << stats.updateStats.getLoggerRow() << "\n";
            ellipseReconstructionPerfLogger << requestState.originalRequest.requestId << ", "
                    << stats.ellipseReconstructionStats.getLoggerRow() << "\n";
            ordinaryTransferPerfLogger << requestState.originalRequest.requestId << ", "
                    << stats.ordinaryTransferStats.getLoggerRow() << "\n";
            transferALSPVehPerfLogger << requestState.originalRequest.requestId << ", "
                    << stats.transferALSPVehStats.getLoggerRow() << "\n";
            transferALSDVehPerfLogger << requestState.originalRequest.requestId << ", "
                    << stats.transferALSDVehStats.getLoggerRow() << "\n";
        }

    private:
        void updateEllipticBucketsForSingleAssignment(const int vehId,
                                                      const int preAsgnPickupIndex,
                                                      const int preAsgnDropoffIndex,
                                                      const int postAsgnPickupIndex, const int postAsgnDropoffIndex,
                                                      const bool insertedIntermediateStopForReroute,
                                                      stats::UpdatePerformanceStats &stats) {
            const auto numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = postAsgnPickupIndex == preAsgnPickupIndex +
                                              insertedIntermediateStopForReroute;
            const bool dropoffAtExistingStop = postAsgnDropoffIndex == preAsgnDropoffIndex +
                                               insertedIntermediateStopForReroute
                                               + !pickupAtExistingStop;

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (!pickupAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(fleet[vehId], postAsgnPickupIndex, stats);
                ellipticBucketsEnv.generateSourceBucketEntries(fleet[vehId], postAsgnPickupIndex, stats);
            }

            // If no new stop was inserted for the dropoff, we do not need to generate any new entries for it.
            if (!dropoffAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(fleet[vehId], postAsgnDropoffIndex, stats);

                if (postAsgnDropoffIndex < numStops - 1) {
                    // If dropoff is not the new last stop, we generate elliptic source buckets for it.
                    ellipticBucketsEnv.generateSourceBucketEntries(fleet[vehId], postAsgnDropoffIndex, stats);
                } else {
                    // If dropoff is the new last stop, the former last stop becomes a regular stop:
                    // Generate elliptic source bucket entries for former last stop
                    const auto pickupAtEnd = postAsgnPickupIndex + 1 == postAsgnDropoffIndex && !pickupAtExistingStop;
                    const int formerLastStopIdx = postAsgnDropoffIndex - pickupAtEnd - 1;
                    ellipticBucketsEnv.generateSourceBucketEntries(fleet[vehId], formerLastStopIdx, stats);
                }
            }
        }

        void markStopsWithNewEllipticEntries(const int vehId, const int preAsgnPickupIndex,
                                             const int preAsgnDropoffIndex,
                                             const int postAsgnPickupIndex, const int postAsgnDropoffIndex,
                                             const bool insertedIntermediateStopForReroute,
                                             stats::UpdatePerformanceStats &,
                                             std::vector<int> &stopIdsToGenerateSourceEntriesFor,
                                             std::vector<int> &stopIdsToGenerateTargetEntriesFor) {
            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopIds = routeState.stopIdsFor(vehId);
            const bool pickupAtExistingStop = postAsgnPickupIndex == preAsgnPickupIndex +
                                              insertedIntermediateStopForReroute;
            const bool dropoffAtExistingStop = postAsgnDropoffIndex == preAsgnDropoffIndex +
                                               insertedIntermediateStopForReroute
                                               + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                stopIdsToGenerateSourceEntriesFor.push_back(stopIds[postAsgnPickupIndex]);
                stopIdsToGenerateTargetEntriesFor.push_back(stopIds[postAsgnPickupIndex]);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            stopIdsToGenerateTargetEntriesFor.push_back(stopIds[postAsgnDropoffIndex]);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (postAsgnDropoffIndex < numStops - 1) {
                stopIdsToGenerateSourceEntriesFor.push_back(stopIds[postAsgnDropoffIndex]);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = postAsgnPickupIndex + 1 == postAsgnDropoffIndex && postAsgnPickupIndex >
                                     preAsgnPickupIndex;
            const int formerLastStopIdx = postAsgnDropoffIndex - pickupAtEnd - 1;
            stopIdsToGenerateSourceEntriesFor.push_back(stopIds[formerLastStopIdx]);
        }

        void updateLastStopBucketsForSingleAssignment(const int vehId,
                                                      const int preAsgnPickupIndex, const int preAsgnDropoffIndex,
                                                      const int postAsgnPickupIndex, const int postAsgnDropoffIndex,
                                                      const bool insertedIntermediateStopForReroute,
                                                      stats::UpdatePerformanceStats &stats) {
            const auto &numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = postAsgnPickupIndex == preAsgnPickupIndex +
                                              insertedIntermediateStopForReroute;
            const bool dropoffAtExistingStop = postAsgnDropoffIndex == preAsgnDropoffIndex +
                                               insertedIntermediateStopForReroute
                                               + !pickupAtExistingStop;

            if (postAsgnDropoffIndex == numStops - 1 && !dropoffAtExistingStop) {
                // If dropoff is the new last stop, remove entries for former last stop, and generate for dropoff
                const auto pickupAtEnd = postAsgnPickupIndex + 1 == postAsgnDropoffIndex && postAsgnPickupIndex >
                                         preAsgnPickupIndex;
                const int formerLastStopIdx = postAsgnDropoffIndex - pickupAtEnd - 1;
                if (formerLastStopIdx == 0) {
                    lastStopBucketsEnv.removeIdleBucketEntries(fleet[vehId], formerLastStopIdx, stats);
                } else {
                    lastStopBucketsEnv.removeNonIdleBucketEntries(fleet[vehId], formerLastStopIdx, stats);
                }
                lastStopBucketsEnv.generateNonIdleBucketEntries(fleet[vehId], stats);
            }
        }

        void markStopsAndVehiclesWithLastStopBucketUpdates(const int vehId,
                                                           const int preAsgnPickupIndex,
                                                           const int preAsgnDropoffIndex,
                                                           const int postAsgnPickupIndex,
                                                           const int postAsgnDropoffIndex,
                                                           const bool insertedIntermediateStopForReroute,
                                                           stats::UpdatePerformanceStats &,
                                                           std::vector<int> &stopIdsToRemoveIdleEntriesFor,
                                                           std::vector<int> &stopIdsToRemoveNonIdleEntriesFor,
                                                           std::vector<int> &vehIdsToGenerateNonIdleEntriesFor) {
            const auto &numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = postAsgnPickupIndex == preAsgnPickupIndex +
                                              insertedIntermediateStopForReroute;
            const bool dropoffAtExistingStop = postAsgnDropoffIndex == preAsgnDropoffIndex +
                                               insertedIntermediateStopForReroute
                                               + !pickupAtExistingStop;

            if (postAsgnDropoffIndex == numStops - 1 && !dropoffAtExistingStop) {
                // If dropoff is the new last stop, remove entries for former last stop, and generate for dropoff
                const auto pickupAtEnd = postAsgnPickupIndex + 1 == postAsgnDropoffIndex && postAsgnPickupIndex >
                                         preAsgnPickupIndex;
                const int formerLastStopIdx = postAsgnDropoffIndex - pickupAtEnd - 1;
                if (formerLastStopIdx == 0) {
                    stopIdsToRemoveIdleEntriesFor.push_back(routeState.stopIdsFor(vehId)[formerLastStopIdx]);
                } else {
                    stopIdsToRemoveNonIdleEntriesFor.push_back(routeState.stopIdsFor(vehId)[formerLastStopIdx]);
                }
                vehIdsToGenerateNonIdleEntriesFor.push_back(vehId);
            }
            // else if (depTimeAtLastChanged) {
            //     // If last stop does not change but departure time at last changes, update last stop bucket entries
            //     // accordingly.
            //     stopIdsToUpdateEntriesFor.push_back(routeState.stopIdsFor(vehId)[numStops - 1]);
            // }
        }

        bool validateRouteDistances(const int vehId, const int startIdx) {
            const auto numStops = routeState.numStopsOf(vehId);
            if (numStops <= 1)
                return true;

            const auto schedArrTimes = routeState.schedArrTimesFor(vehId);
            const auto schedDepTimes = routeState.schedDepTimesFor(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);

            for (auto i = startIdx; i < numStops - 1; ++i) {
                if (routeState.isIntermediateStopForReroute(vehId, i + 1))
                    continue; // Intermediate stops have temporary arrival times, so we skip distance checks for them.
                const auto distSchedule = schedArrTimes[i + 1] - schedDepTimes[i];
                if (stopLocations[i] == stopLocations[i + 1]) {
                    if (distSchedule != 0) {
                        KASSERT(false, routeState.printRouteOf(vehId));
                        return false;
                    }
                    continue;
                }

                const auto source = inputGraph.edgeHead(stopLocations[i]);
                const auto target = inputGraph.edgeTail(stopLocations[i + 1]);
                const auto offset = inputGraph.travelTime(stopLocations[i + 1]);
                chQuery.run(vehCh.rank(source), vehCh.rank(target));
                const auto distRecomputed = chQuery.getDistance() + offset;

                if (distSchedule != distRecomputed) {
                    KASSERT(false, routeState.printRouteOf(vehId));
                    return false;
                }
            }

            return true;
        }

        const InputGraphT &inputGraph;
        const CH &vehCh;
        const Fleet &fleet;
        CHQuery<BasicLabelSet<0, ParentInfo::NO_PARENT_INFO> > chQuery;
        VehicleLocatorT &vehicleLocator;
        PathTrackerT &pathTracker;
        time_utils::DetourComputer detourComputer;

        // Route state
        RouteState &routeState;

        // Bucket state
        EllipticBucketsEnvT &ellipticBucketsEnv;
        LastStopBucketsEnvT &lastStopBucketsEnv;


        Subset lastStopsWithChangedDepTime;

        // Performance Loggers
        LoggerT &bestAssignmentsOverallLogger;
        LoggerT &bestAssignmentsWithoutUsingVehicleLogger;
        LoggerT &bestAssignmentsWithoutTransferLogger;
        LoggerT &bestAssignmentsWithTransferLogger;
        LoggerT &overallPerfLogger;
        ComponentPerfLoggerT &initializationPerfLogger;
        ComponentPerfLoggerT &ellipticBchPerfLogger;
        ComponentPerfLoggerT &pdDistancesPerfLogger;
        ComponentPerfLoggerT &ordPerfLogger;
        ComponentPerfLoggerT &pbnsPerfLogger;
        ComponentPerfLoggerT &palsPerfLogger;
        ComponentPerfLoggerT &dalsPerfLogger;
        ComponentPerfLoggerT &updatePerfLogger;

        LoggerT &ellipseReconstructionPerfLogger;
        LoggerT &ordinaryTransferPerfLogger;
        LoggerT &transferALSPVehPerfLogger;
        LoggerT &transferALSDVehPerfLogger;

        LoggerT &assignmentsCostLogger;

        LoggerT &batchInsertLogger;

        LoggerT &lastStopBucketUpdateLogger;
    };
}
