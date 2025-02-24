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
#include "Tools/Timer.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleLocation.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {


    template<typename InputGraphT, typename VehicleLocatorT, typename CHEnvT, typename LabelSetT>
    class CurVehLocToPickupSearches {

    private:


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        static constexpr VehicleLocation INVALID_LOC = {INVALID_EDGE, -1};

        static constexpr int unknownDist = INFTY + 1;


        struct StopWhenMaxDistExceeded {

            explicit StopWhenMaxDistExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return !anySet(~(maxDist < distToV));
            }


        private:
            const int &maxDist;
        };

        struct WriteVehicleDistLabel {
            explicit WriteVehicleDistLabel(CurVehLocToPickupSearches &searches) : searches(searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                searches.distFromCurVehLocation[v] = distToV[0];
                return false;
            }

        private:

            CurVehLocToPickupSearches &searches;
        };

        struct ScanLabelAndUpdateDistances {
            explicit ScanLabelAndUpdateDistances(CurVehLocToPickupSearches &searches) : searches(
                    searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                const auto &distFromVehLocToV = searches.distFromCurVehLocation[v];
                if (distFromVehLocToV >= INFTY)
                    return false;

                DistanceLabel dists = distToV + DistanceLabel(distFromVehLocToV);
                dists.setIf(DistanceLabel(INFTY), ~(distToV < INFTY));
                searches.tentativeDistances.min(dists);
                searches.maxTentativeDist = std::min(searches.maxTentativeDist,
                                                     searches.tentativeDistances.horizontalMax());
                return false;
            }

        private:

            CurVehLocToPickupSearches &searches;
        };

        using WriteVehLabelsSearch = typename CHEnvT::template UpwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>;
        using FindDistancesSearch = typename CHEnvT::template UpwardSearch<ScanLabelAndUpdateDistances, StopWhenMaxDistExceeded, LabelSetT>;


    public:

        CurVehLocToPickupSearches(const InputGraphT &graph,
                                  VehicleLocatorT &locator,
                                  const CHEnvT &chEnv,
                                  const RouteState &routeState,
                                  const int fleetSize)
                : inputGraph(graph),
                  vehicleLocator(locator),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  fleetSize(fleetSize),
                  distances(),
                  currentVehicleLocations(fleetSize, INVALID_LOC),
                  prevNumPickups(0),
                  vehiclesWithKnownLocation(),
                  writeVehLabelsSearch(
                          chEnv.template getForwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>(
                                  WriteVehicleDistLabel(*this), StopWhenMaxDistExceeded(curLeeway))),
                  findDistancesSearch(
                          chEnv.template getReverseSearch<ScanLabelAndUpdateDistances, StopWhenMaxDistExceeded, LabelSetT>(
                                  ScanLabelAndUpdateDistances(*this),
                                  StopWhenMaxDistExceeded(maxTentativeDist))),
                  maxTentativeDist(INFTY),
                  curPickupIds(),
                  currentTime(-1),
                  waitingQueue(),
                  distFromCurVehLocation(chEnv.getCH().upwardGraph().numVertices(), INFTY) {}

        void initialize(const int now, const PDLocs& pdLocs) {
            currentTime = now;

            clearDistances();
            waitingQueue.clear();

            curNumPickups = pdLocs.numPickups();
            const int numDistances = curNumPickups * fleetSize;
            if (numDistances > distances.size()) {
                const int diff = numDistances - distances.size();
                distances.insert(distances.end(), diff, unknownDist);
            }


            totalLocatingVehiclesTimeForRequest = 0;
            totalVehicleToPickupSearchTimeForRequest = 0;
            totalNumCHSearchesRunForRequest = 0;
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < curNumPickups);
            const int idx = vehId * curNumPickups + pickupId;
            return distances[idx] != unknownDist;
        }

        int getDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < curNumPickups);
            const int idx = vehId * curNumPickups + pickupId;
            return distances[idx];
        }

        bool knowsCurrentLocationOf(const int vehId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            return currentVehicleLocations[vehId] != INVALID_LOC;
        }

        const VehicleLocation &getCurrentLocationOf(const int vehId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            return currentVehicleLocations[vehId];
        }

        // Register pickups for which we want to know the distance from the current location of a vehicle to this pickup.
        // All pickups registered until the next call to computeExactDistancesVia() will be processed with the same vehicle.
        void addPickupForProcessing(const int pickupId, const int distFromPrevStopToPickup) {
            assert(pickupId >= 0);
            assert(pickupId < curNumPickups);
            waitingQueue.push_back({pickupId, distFromPrevStopToPickup});
        }

        // Computes the exact distances via a given vehicle to all pickups added using addPickupForProcessing() (since the
        // last call to this function or initialize()). Skips pickups for which the distance via the given vehicle is
        // already known.
        void computeExactDistancesVia(const Vehicle &vehicle, const PDLocs& pdLocs) {

            assert(routeState.numStopsOf(vehicle.vehicleId) > 1);
            curLeeway = routeState.leewayOfLegStartingAt(routeState.stopIdsFor(vehicle.vehicleId)[0]);
            if (waitingQueue.empty()) return;

            if (!knowsCurrentLocationOf(vehicle.vehicleId)) {
                currentVehicleLocations[vehicle.vehicleId] = locateVehicle(vehicle);
                vehiclesWithKnownLocation.push_back(vehicle.vehicleId);
            }
            const auto &vehLocation = currentVehicleLocations[vehicle.vehicleId];
            assert(vehLocation != INVALID_LOC);

            if (vehLocation.location == routeState.stopLocationsFor(vehicle.vehicleId)[0]) {
                fillDistancesForVehicleAtPrevStop(vehicle, pdLocs);
                waitingQueue.clear();
                prevNumPickups = curNumPickups;
                return;
            }

            const auto distToCurLoc = vehLocation.depTimeAtHead - routeState.schedDepTimesFor(vehicle.vehicleId)[0];

            int numChSearchesRun = 0;
            Timer timer;
            std::array<int, K> targets;
            std::array<int, K> targetOffsets;

            unsigned int i = 0;
            bool builtLabelsForVeh = false;

            for (auto it = waitingQueue.begin(); it != waitingQueue.end();) {
                const auto pickupId = it->first;

                if (!knowsDistance(vehicle.vehicleId, pickupId)) {
                    const auto pickupLocation = pdLocs.pickups[pickupId].loc;
                    if (vehLocation.location == pickupLocation) {
                        const int idx = vehicle.vehicleId * curNumPickups + pickupId;
                        distances[idx] = distToCurLoc;
                    } else {
                        targets[i] = ch.rank(inputGraph.edgeTail(pickupLocation));
                        targetOffsets[i] = inputGraph.travelTime(pickupLocation);
                        curPickupIds[i] = pickupId;
                        ++i;
                    }
                }

                ++it;
                if (i == K || (it == waitingQueue.end() && i > 0)) {
                    // If there were any pairs left but fewer than K, fill the sources and targets with duplicates of the first pair
                    int endOfBatch = i;
                    for (; i < K; ++i) {
                        targets[i] = targets[0];
                        targetOffsets[i] = targetOffsets[0];
                        curPickupIds[i] = curPickupIds[0];
                    }

                    maxTentativeDist = curLeeway;
                    tentativeDistances = DistanceLabel(INFTY);
                    // Build distance labels for the vehicle
                    if (!builtLabelsForVeh) {
                        distFromCurVehLocation.clear();
                        const auto source = ch.rank(inputGraph.edgeHead(vehLocation.location));
                        writeVehLabelsSearch.runWithOffset(source, distToCurLoc);
                        builtLabelsForVeh = true;
                    }

                    // Run search from pickups against the vehicle distance labels
                    findDistancesSearch.runWithOffset(targets, targetOffsets);
                    ++numChSearchesRun;

                    // Set found distances.
                    for (int j = 0; j < endOfBatch; ++j) {
                        const int idx = vehicle.vehicleId * curNumPickups + curPickupIds[j];
                        distances[idx] = tentativeDistances[j];
                    }

                    i = 0;
                }
            }

            waitingQueue.clear();
            prevNumPickups = curNumPickups;

            totalVehicleToPickupSearchTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
            totalNumCHSearchesRunForRequest += numChSearchesRun;
        }

        int64_t getTotalLocatingVehiclesTimeForRequest() const {
            return totalLocatingVehiclesTimeForRequest;
        }

        int64_t getTotalVehicleToPickupSearchTimeForRequest() const {
            return totalVehicleToPickupSearchTimeForRequest;
        }

        int64_t getTotalNumCHSearchesRunForRequest() const {
            return totalNumCHSearchesRunForRequest;
        }

    private:

        void clearDistances() {

            // Clear the distances for every vehicle for which we computed the current location:
            for (const auto &vehId: vehiclesWithKnownLocation) {
                const int start = vehId * prevNumPickups;
                const int end = start + prevNumPickups;
                std::fill(distances.begin() + start, distances.begin() + end, unknownDist);
                currentVehicleLocations[vehId] = INVALID_LOC;
            }
            assert(std::all_of(distances.begin(), distances.end(), [&](const auto &d) { return d == unknownDist; }));
            assert(std::all_of(currentVehicleLocations.begin(), currentVehicleLocations.end(),
                               [&](const auto &l) { return l == INVALID_LOC; }));
            vehiclesWithKnownLocation.clear();
        }

        VehicleLocation locateVehicle(const Vehicle &vehicle) {
            Timer timer;
            const auto curLoc = vehicleLocator.computeCurrentLocation(vehicle, currentTime);
            totalLocatingVehiclesTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
            return curLoc;
        }

        void fillDistancesForVehicleAtPrevStop(const Vehicle &vehicle, const PDLocs& pdLocs) {
            const auto &stopLocations = routeState.stopLocationsFor(vehicle.vehicleId);
            for (const auto &[pickupId, distFromPrevStopToPickup]: waitingQueue) {
                if (stopLocations[0] != pdLocs.pickups[pickupId].loc) {
                    const int idx = vehicle.vehicleId * curNumPickups + pickupId;
                    distances[idx] = distFromPrevStopToPickup;
                } else {
                    const int idx = vehicle.vehicleId * curNumPickups + pickupId;
                    distances[idx] = 0;
                }
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        const CH &ch;
        const RouteState &routeState;
        const int fleetSize;


        std::vector<int> distances;
        std::vector<VehicleLocation> currentVehicleLocations;
        int prevNumPickups;
        std::vector<int> vehiclesWithKnownLocation;

        WriteVehLabelsSearch writeVehLabelsSearch;
        FindDistancesSearch findDistancesSearch;
        DistanceLabel tentativeDistances;
        int maxTentativeDist;
        std::array<unsigned int, K> curPickupIds;
        int curLeeway;

        int currentTime;
        int curNumPickups;

        int64_t totalLocatingVehiclesTimeForRequest;
        int64_t totalVehicleToPickupSearchTimeForRequest;
        int64_t totalNumCHSearchesRunForRequest;

        // Entry in waiting queue is pickupId + dist from previous stop of vehicle to pickup.
        std::vector<std::pair<unsigned int, int>> waitingQueue;

        TimestampedVector<int> distFromCurVehLocation;
    };

}