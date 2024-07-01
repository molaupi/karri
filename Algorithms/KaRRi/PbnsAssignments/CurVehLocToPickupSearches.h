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




        struct StopIfUpperBoundCostExceeded {

            explicit StopIfUpperBoundCostExceeded(const int &upperBoundCost)
                    : upperBoundCost(upperBoundCost) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &costToV, const DistLabelContainerT & /*distLabels*/) {
                return allSet(costToV > upperBoundCost);
            }

        private:
            const int &upperBoundCost;
        };

        // Bool overload of allSet which is usually defined for LabelMasks of LabelSets
        static bool localAllSet(const bool &b) {
            return b;
        }

        static bool localAllSet(const typename LabelSetT::LabelMask& mask) {
            return allSet(mask);
        }

        template<typename TravelTimesContainer>
        struct PruneIfTravelTimeExceedsLeeway {

            explicit PruneIfTravelTimeExceedsLeeway(const int &leeway, TravelTimesContainer &travelTimes)
                    : leeway(leeway), travelTimes(travelTimes) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &, const DistLabelContainerT & /*distLabels*/) {
                return localAllSet(travelTimes[v] > leeway);
            }

        private:
            const int &leeway;
            TravelTimesContainer &travelTimes;
        };

        using PruneSingleTravelTime = PruneIfTravelTimeExceedsLeeway<TimestampedVector < int>>;
        using PruneKTravelTimes = PruneIfTravelTimeExceedsLeeway<StampedDistanceLabelContainer<DistanceLabel>>;

        struct ScanLabelAndUpdateDistances {
            explicit ScanLabelAndUpdateDistances(CurVehLocToPickupSearches &searches) : searches(
                    searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &costFromV, const DistLabelContT &) {
                const auto &travelTimesFromV = searches.travelTimesToPickups[v];
                const auto &costFromVehLocToV = searches.writeVehLabelsSearch.getDistance(v);
                const auto &travelTimeFromVehLocToV = searches.travelTimeFromCurVehLocation[v];
                if (costFromVehLocToV >= INFTY)
                    return false;

                DistanceLabel costsFromVehLocToPickup = DistanceLabel(costFromVehLocToV) + costFromV;
                DistanceLabel travelTimesFromVehLocToPickup = DistanceLabel(travelTimeFromVehLocToV) + travelTimesFromV;
                costsFromVehLocToPickup.setIf(DistanceLabel(INFTY), ~(costFromV < INFTY));
                travelTimesFromVehLocToPickup.setIf(DistanceLabel(INFTY), ~(costFromV < INFTY));
                const auto smaller = costsFromVehLocToPickup < searches.tentativeCosts;
                searches.tentativeCosts.setIf(costsFromVehLocToPickup, smaller);
                searches.tentativeTravelTimes.setIf(travelTimesFromVehLocToPickup, smaller);
                searches.maxTentativeCost = std::min(searches.maxTentativeCost,
                                                     searches.tentativeCosts.horizontalMax());
                return false;
            }

        private:

            CurVehLocToPickupSearches &searches;
        };

        struct WriteLabelsUpdateTravelTimeCallback {

            WriteLabelsUpdateTravelTimeCallback(const typename CH::SearchGraph &searchGraph,
                                                TimestampedVector<int> &travelTimes) : searchGraph(searchGraph),
                                                                                       travelTimes(travelTimes) {}

            template<typename LabelMaskT, typename DistanceLabelContainerT>
            void operator()(const int v, const int w, const int e, const LabelMaskT &improved,
                            const DistanceLabelContainerT &) {
                if (improved[0]) {
                    travelTimes[w] = travelTimes[v] + searchGraph.travelTime(e);
                }
            }

            const CH::SearchGraph &searchGraph;
            TimestampedVector<int> &travelTimes;
        };

        struct FindDistancesUpdateTravelTimeCallback {

            FindDistancesUpdateTravelTimeCallback(const typename CH::SearchGraph &searchGraph,
                                                  StampedDistanceLabelContainer<DistanceLabel> &travelTimes)
                    : searchGraph(searchGraph), travelTimes(travelTimes) {}

            template<typename LabelMaskT, typename DistanceLabelContainerT>
            void operator()(const int v, const int w, const int e, const LabelMaskT &improved,
                            const DistanceLabelContainerT &) {
                travelTimes[w].setIf(travelTimes[v] + searchGraph.travelTime(e), improved);
            }

            const CH::SearchGraph &searchGraph;
            StampedDistanceLabelContainer<DistanceLabel> &travelTimes;
        };

        using WriteVehLabelsSearch = typename CHEnvT::template UpwardSearch<
                PruneSingleTravelTime, StopIfUpperBoundCostExceeded, WriteLabelsUpdateTravelTimeCallback>;
        using FindDistancesSearch = typename CHEnvT::template UpwardSearch<
                dij::CompoundCriterion<PruneKTravelTimes, ScanLabelAndUpdateDistances>,
                StopIfUpperBoundCostExceeded, FindDistancesUpdateTravelTimeCallback, LabelSetT>;


    public:

        CurVehLocToPickupSearches(const InputGraphT &graph,
                                  VehicleLocatorT &locator,
                                  const CHEnvT &chEnv,
                                  const RouteState &routeState,
                                  RequestState &requestState,
                                  const int fleetSize)
                : inputGraph(graph),
                  vehicleLocator(locator),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  requestState(requestState),
                  fleetSize(fleetSize),
                  costs(),
                  travelTimes(),
                  currentVehicleLocations(fleetSize, INVALID_LOC),
                  prevNumPickups(0),
                  vehiclesWithKnownLocation(),
                  writeVehLabelsSearch(
                          chEnv.getForwardSearch(
                                  PruneSingleTravelTime(curLeeway, travelTimeFromCurVehLocation),
                                  StopIfUpperBoundCostExceeded(requestState.getBestCost()),
                                  WriteLabelsUpdateTravelTimeCallback(chEnv.getCH().upwardGraph(),
                                                                      travelTimeFromCurVehLocation))),
                  findDistancesSearch(
                          chEnv.template getReverseSearch<dij::CompoundCriterion<PruneKTravelTimes, ScanLabelAndUpdateDistances>,
                                  StopIfUpperBoundCostExceeded, FindDistancesUpdateTravelTimeCallback, LabelSetT>(
                                  dij::CompoundCriterion(PruneKTravelTimes(curLeeway, travelTimesToPickups),
                                                         ScanLabelAndUpdateDistances(*this)),
                                  StopIfUpperBoundCostExceeded(requestState.getBestCost()),
                                  FindDistancesUpdateTravelTimeCallback(chEnv.getCH().downwardGraph(),
                                                                        travelTimesToPickups))),
                  travelTimesToPickups(ch.downwardGraph().numVertices()),
                  maxTentativeCost(INFTY),
                  curPickupIds(),
                  currentTime(-1),
                  waitingQueue(),
                  travelTimeFromCurVehLocation(chEnv.getCH().upwardGraph().numVertices(), INFTY) {}

        void initialize(const int now) {
            currentTime = now;

            clearDistances();
            waitingQueue.clear();


            totalLocatingVehiclesTimeForRequest = 0;
            totalVehicleToPickupSearchTimeForRequest = 0;
            totalNumCHSearchesRunForRequest = 0;
        }

        bool knowsCost(const int vehId, const unsigned int pickupId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            KASSERT(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return costs[idx] != unknownDist;
        }

        int getCost(const int vehId, const unsigned int pickupId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            KASSERT(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return costs[idx];
        }

        int getTravelTime(const int vehId, const unsigned int pickupId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            KASSERT(pickupId < requestState.numPickups());
            const int idx = vehId * requestState.numPickups() + pickupId;
            return travelTimes[idx];
        }

        bool knowsCurrentLocationOf(const int vehId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            return currentVehicleLocations[vehId] != INVALID_LOC;
        }

        const VehicleLocation &getCurrentLocationOf(const int vehId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            return currentVehicleLocations[vehId];
        }

        // Register pickups for which we want to know the distance from the current location of a vehicle to this pickup.
        // All pickups registered until the next call to computeExactDistancesVia() will be processed with the same vehicle.
        void addPickupForProcessing(const int pickupId, const int distFromPrevStopToPickup,
                                    const int travelTimeFromPrevStopToPickup) {
            KASSERT(pickupId >= 0);
            KASSERT(pickupId < requestState.pickups.size());
            waitingQueue.push_back({pickupId, distFromPrevStopToPickup, travelTimeFromPrevStopToPickup});
        }

        // Computes the exact distances via a given vehicle to all pickups added using addPickupForProcessing() (since the
        // last call to this function or initialize()). Skips pickups for which the distance via the given vehicle is
        // already known.
        void computeExactDistancesVia(const Vehicle &vehicle) {

            KASSERT(routeState.numStopsOf(vehicle.vehicleId) > 1);
            curLeeway = routeState.leewayOfLegStartingAt(routeState.stopIdsFor(vehicle.vehicleId)[0]);
            if (waitingQueue.empty()) return;

            if (!knowsCurrentLocationOf(vehicle.vehicleId)) {
                currentVehicleLocations[vehicle.vehicleId] = locateVehicle(vehicle);
                vehiclesWithKnownLocation.push_back(vehicle.vehicleId);
            }
            const auto &vehLocation = currentVehicleLocations[vehicle.vehicleId];
            KASSERT(vehLocation != INVALID_LOC);

            if (vehLocation.location == routeState.stopLocationsFor(vehicle.vehicleId)[0]) {
                fillDistancesForVehicleAtPrevStop(vehicle);
                waitingQueue.clear();
                prevNumPickups = requestState.numPickups();
                return;
            }

            const auto costToCurLoc = vehLocation.costFromPrevStopToHead;
            const auto travelTimeToCurLoc = vehLocation.travelTimeFromPrevStopToHead;

            int numChSearchesRun = 0;
            Timer timer;
            std::array<int, K> targets;
            std::array<int, K> targetCostOffsets;

            unsigned int i = 0;
            bool builtLabelsForVeh = false;

            travelTimesToPickups.init();
            for (auto it = waitingQueue.begin(); it != waitingQueue.end();) {
                const auto pickupId = std::get<0>(*it);

                if (!knowsCost(vehicle.vehicleId, pickupId)) {
                    const auto pickupLocation = requestState.pickups[pickupId].loc;
                    if (vehLocation.location == pickupLocation) {
                        const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                        costs[idx] = costToCurLoc;
                        travelTimes[idx] = travelTimeToCurLoc;
                    } else {
                        targets[i] = ch.rank(inputGraph.edgeTail(pickupLocation));
                        targetCostOffsets[i] = inputGraph.traversalCost(pickupLocation);
                        travelTimesToPickups[targets[i]][i] = inputGraph.travelTime(pickupLocation);
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
                        targetCostOffsets[i] = targetCostOffsets[0];
                        travelTimesToPickups[targets[0]][i] = static_cast<int>(travelTimesToPickups[targets[0]][0]);
                        curPickupIds[i] = curPickupIds[0];
                    }

                    tentativeCosts = DistanceLabel(INFTY);
                    tentativeTravelTimes = DistanceLabel(INFTY);
                    // Build distance labels for the vehicle
                    if (!builtLabelsForVeh) {
                        const auto source = ch.rank(inputGraph.edgeHead(vehLocation.location));
                        travelTimeFromCurVehLocation.clear();
                        travelTimeFromCurVehLocation[source] = travelTimeToCurLoc;
                        writeVehLabelsSearch.runWithOffset(source, costToCurLoc);
                        builtLabelsForVeh = true;
                    }

                    // Run search from pickups against the vehicle distance labels
                    findDistancesSearch.runWithOffset(targets, targetCostOffsets);
                    travelTimesToPickups.init();
                    ++numChSearchesRun;

                    // Set found distances.
                    for (int j = 0; j < endOfBatch; ++j) {
                        const int idx = vehicle.vehicleId * requestState.numPickups() + curPickupIds[j];
                        costs[idx] = tentativeCosts[j];
                        travelTimes[idx] = tentativeTravelTimes[j];
                    }

                    i = 0;
                }
            }

            waitingQueue.clear();
            prevNumPickups = requestState.numPickups();

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
                std::fill(costs.begin() + start, costs.begin() + end, unknownDist);
                std::fill(travelTimes.begin() + start, travelTimes.begin() + end, unknownDist);
                currentVehicleLocations[vehId] = INVALID_LOC;
            }
            KASSERT(std::all_of(currentVehicleLocations.begin(), currentVehicleLocations.end(),
                                [&](const auto &l) { return l == INVALID_LOC; }));
            vehiclesWithKnownLocation.clear();

            const int numDistances = requestState.numPickups() * fleetSize;
            if (numDistances > costs.size()) {
                const int diff = numDistances - costs.size();
                costs.insert(costs.end(), diff, unknownDist);
                travelTimes.insert(travelTimes.end(), diff, unknownDist);
            }
        }

        VehicleLocation locateVehicle(const Vehicle &vehicle) {
            Timer timer;
            const auto curLoc = vehicleLocator.computeCurrentLocation(vehicle, currentTime);
            totalLocatingVehiclesTimeForRequest += timer.elapsed<std::chrono::nanoseconds>();
            return curLoc;
        }

        void fillDistancesForVehicleAtPrevStop(const Vehicle &vehicle) {
            const auto &stopLocations = routeState.stopLocationsFor(vehicle.vehicleId);
            for (const auto &[pickupId, costFromPrevStopToPickup, travelTimeFromPrevStopToPickup]: waitingQueue) {
                if (stopLocations[0] != requestState.pickups[pickupId].loc) {
                    const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                    costs[idx] = costFromPrevStopToPickup;
                    travelTimes[idx] = travelTimeFromPrevStopToPickup;
                } else {
                    const int idx = vehicle.vehicleId * requestState.numPickups() + pickupId;
                    costs[idx] = 0;
                    travelTimes[idx] = 0;
                }
            }
        }

        const InputGraphT &inputGraph;
        VehicleLocatorT &vehicleLocator;
        const CH &ch;
        const RouteState &routeState;
        RequestState &requestState;
        const int fleetSize;

        // Result of searches:
        // costs[i * requestState.numPickups() + j] is the SP distance according to traversal costs from the current
        // location of vehicle i to pickup j.
        // travelTimes[i * requestState.numPickups() + j] is the total travel time on the SP according to traversal costs.
        std::vector<int> costs;
        std::vector<int> travelTimes;

        std::vector<VehicleLocation> currentVehicleLocations;
        int prevNumPickups;
        std::vector<int> vehiclesWithKnownLocation;

        WriteVehLabelsSearch writeVehLabelsSearch;
        FindDistancesSearch findDistancesSearch;
        StampedDistanceLabelContainer<DistanceLabel> travelTimesToPickups;
        DistanceLabel tentativeCosts; // Tentative SP distances according to traversal cost
        DistanceLabel tentativeTravelTimes; // Tentative travel times on tentative SPs according to traversal cost
        int maxTentativeCost;
        std::array<unsigned int, K> curPickupIds;
        int curLeeway;

        int currentTime;

        int64_t totalLocatingVehiclesTimeForRequest;
        int64_t totalVehicleToPickupSearchTimeForRequest;
        int64_t totalNumCHSearchesRunForRequest;

        // Entry in waiting queue is pickupId + dist from previous stop of vehicle to pickup.
        std::vector<std::tuple<unsigned int, int, int>> waitingQueue;

        // travelTimeFromCurVehLocation[v] is the total travel time on the shortest path according to traversal cost
        // found by writeVehLabelsSearch. (i.e. not necessarily SP according to travel time).
        TimestampedVector<int> travelTimeFromCurVehLocation;
    };

}