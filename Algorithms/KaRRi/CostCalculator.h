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

#include <algorithm>
#include <cassert>
#include <cmath>

#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Tools/Constants.h"
#include "Tools/Workarounds.h"
#include "Algorithms/KaRRi/AssignmentCostFunctions/TimeIsMoneyCostFunction.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

// A facility for computing the cost of an assignment of a request into a vehicle's route.
    template<typename CostFunctionT>
    class CostCalculatorTemplate {

        using F = CostFunctionT;

    public:

        using CostFunction = CostFunctionT;

        explicit CostCalculatorTemplate() {}


        static int calc(const Assignment &asgn, const RequestState &reqState, const RouteStateData& routeState) {
            return calcBase<true>(asgn, reqState, routeState);
        }


        static int calcWithoutHardConstraints(const Assignment &asgn, const RequestState &reqState, const RouteStateData& routeState) {
            return calcBase<false>(asgn, reqState, routeState);
        }

        // Calculates the objective value for a given assignment.
        template<bool checkHardConstraints>
        static int calcBase(const Assignment &asgn, const RequestState &reqState, const RouteStateData& routeState) {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup && asgn.dropoff);
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return INFTY;

            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY)
                return INFTY;
            const int vehId = asgn.vehicle->vehicleId;
            const auto numStops = routeState.numStopsOf(asgn.vehicle->vehicleId);
            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, reqState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, reqState, routeState);

            int addedTripTime = calcAddedTripTimeInInterval(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                                            initialPickupDetour, routeState);

            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            const auto initialDropoffDetour = calcInitialDropoffDetour(asgn, dropoffAtExistingStop, routeState);
            const auto detourRightAfterDropoff = calcDetourRightAfterDropoff(asgn, initialPickupDetour,
                                                                             initialDropoffDetour, routeState);
            const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(asgn.vehicle->vehicleId,
                                                                                        asgn.dropoffStopIdx,
                                                                                        numStops - 1,
                                                                                        detourRightAfterDropoff,
                                                                                        routeState);

            if (checkHardConstraints && isAnyHardConstraintViolated(asgn, reqState, initialPickupDetour,
                                                                    detourRightAfterDropoff, residualDetourAtEnd,
                                                                    dropoffAtExistingStop, routeState))
                return INFTY;

            addedTripTime += calcAddedTripTimeAffectedByPickupAndDropoff(asgn, detourRightAfterDropoff, routeState);

            return calcCost(asgn, reqState, routeState, initialPickupDetour, residualDetourAtEnd,
                            actualDepTimeAtPickup, dropoffAtExistingStop, addedTripTime);
        }

        // Calculate the cost for a passenger moving to their destination independently without using a vehicle.

        static int calcCostForNotUsingVehicle(const int walkingDist, const int travelTimeOfDestEdge,
                                              const RequestState &reqState) {
            assert(walkingDist >= travelTimeOfDestEdge);

            if (walkingDist >= INFTY)
                return INFTY;

            int destSideDist;
            if (walkingDist <= InputConfig::getInstance().pickupRadius + InputConfig::getInstance().dropoffRadius) {
                // Wait time violation only necessary if travel time of destination edge is already larger than dropoff
                // radius (since we always have to traverse destination edge from tail to head).
                destSideDist = std::max(InputConfig::getInstance().dropoffRadius, travelTimeOfDestEdge);
            } else {
                // Optimally split walking distance in origin side and destination side to minimize wait time violations:
                const int halfOfVioDist = std::floor(
                        0.5 * (walkingDist -
                               (InputConfig::getInstance().pickupRadius + InputConfig::getInstance().dropoffRadius)));
                destSideDist = std::max(InputConfig::getInstance().dropoffRadius + halfOfVioDist, travelTimeOfDestEdge);
            }

            const auto originSideDist = walkingDist - destSideDist;
            const auto walkingCost = F::calcWalkingCost(originSideDist, InputConfig::getInstance().pickupRadius) +
                                     F::calcWalkingCost(destSideDist, InputConfig::getInstance().dropoffRadius);
            const auto tripCost = F::calcTripCost(originSideDist + destSideDist, reqState);
            // no costs for detour, wait violation or change in trip time of other passengers
            return walkingCost + tripCost;
        }

        // For lower bound, pass the assignment consisting of the pickup with the smallest distance from the
        // previous stop, and the dropoff with the smallest distance to the next stop. Uses a lower bound on every
        // PD-distance.

        static int calcCostLowerBoundForOrdinaryPairedAssignment(const Assignment &asgn, const RequestState &reqState, const RouteStateData& routeState) {
            using namespace time_utils;
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return INFTY;
            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY)
                return INFTY;

            assert(asgn.pickupStopIdx == asgn.dropoffStopIdx);
            const auto stopIdx = asgn.pickupStopIdx;
            const auto vehId = asgn.vehicle->vehicleId;

            const int minDetour = asgn.distToPickup + asgn.distToDropoff + asgn.distFromDropoff -
                                  calcLengthOfLegStartingAt(stopIdx, vehId, routeState);
            if (doesDropoffDetourViolateHardConstraints(*asgn.vehicle, reqState, stopIdx, minDetour, routeState))
                return INFTY;

            const int tripTimeLowerBound = asgn.distToDropoff;
            const auto tripCostLowerBound = F::calcTripCost(tripTimeLowerBound, reqState);

            return F::calcVehicleCost(minDetour) + tripCostLowerBound;

        }


        static int calcCostLowerBoundForPairedAssignmentBeforeNextStop(const Vehicle &veh, const PDLoc &pickup,
                                                                const int distToPickup,
                                                                const int minDistToDropoff,
                                                                const int distFromPickupForDetourLowerBound,
                                                                const RequestState &reqState, const RouteStateData& routeState) {
            using namespace time_utils;

            const int vehId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);

            Assignment asgn(&veh, &pickup);
            asgn.distToPickup = distToPickup;
            asgn.distToDropoff = minDistToDropoff;

            const int minActualDepTimeAtPickup = getActualDepTimeAtPickup(vehId, 0, distToPickup, pickup, reqState,
                                                                          routeState);

            const auto initialPickupDetour = calcInitialPickupDetour(asgn, minActualDepTimeAtPickup, reqState,
                                                                     routeState);
            const auto minInitialDropoffDetour = std::max(minDistToDropoff, distFromPickupForDetourLowerBound) -
                                                 calcLengthOfLegStartingAt(0, vehId, routeState);
            auto minDetourRightAfterDropoff =
                    calcDetourRightAfterDropoff(vehId, 0, 0, initialPickupDetour, minInitialDropoffDetour, routeState);
            minDetourRightAfterDropoff = std::max(minDetourRightAfterDropoff, 0);
            const auto residualDetourAtEnd = calcResidualTotalDetour(vehId, 0, 0, numStops - 1, initialPickupDetour,
                                                                     minDetourRightAfterDropoff, routeState);
            if (isAnyHardConstraintViolated(asgn, reqState, initialPickupDetour, minDetourRightAfterDropoff,
                                            residualDetourAtEnd, false, routeState))
                return INFTY;


            const int minTripTime = minActualDepTimeAtPickup - reqState.originalRequest.requestTime + minDistToDropoff;
            const int walkingCost = F::calcWalkingCost(pickup.walkingDist, InputConfig::getInstance().pickupRadius);
            const int minWaitViolationCost = F::calcWaitViolationCost(minActualDepTimeAtPickup, reqState);
            const int minTripCost = F::calcTripCost(minTripTime, reqState);

            const int minAddedTripCostOfOthers = F::calcChangeInTripCostsOfExistingPassengers(
                    calcAddedTripTimeInInterval(vehId, 0, numStops - 1, minDetourRightAfterDropoff, routeState));

            return F::calcVehicleCost(residualDetourAtEnd) + walkingCost + minWaitViolationCost + minTripCost +
                   minAddedTripCostOfOthers;
        }



        static int calcCostLowerBoundForPickupAfterLastStopIndependentOfVehicle(const int distToPickup,
                                                                         const int minDistToDropoff,
                                                                         const RequestState &reqState) {
            if (distToPickup >= INFTY)
                return INFTY;

            const int minDetour = distToPickup + minDistToDropoff + InputConfig::getInstance().stopTime;

            const int minActualDepTimeAtPickup = reqState.originalRequest.requestTime + distToPickup;
            const int minTripTime = minActualDepTimeAtPickup - reqState.originalRequest.requestTime + minDistToDropoff;
            const int minWaitViolationCost = F::calcWaitViolationCost(minActualDepTimeAtPickup, reqState);
            const int minTripCost = F::calcTripCost(minTripTime, reqState);

            // Pickup after last stop so no added trip costs for existing passengers.

            return F::calcVehicleCost(minDetour) + minWaitViolationCost + minTripCost;
        }

        template<typename DistanceLabel>
        static DistanceLabel
        calcCostLowerBoundForKPickupsAfterLastStopIndependentOfVehicle(const DistanceLabel &distsToPickup,
                                                                       const DistanceLabel &minDistsToDropoff,
                                                                       const RequestState &reqState) {
            const DistanceLabel minDetour = distsToPickup + minDistsToDropoff + InputConfig::getInstance().stopTime;


            const DistanceLabel minActualDepTimeAtPickup = reqState.originalRequest.requestTime + distsToPickup;
            const DistanceLabel minTripTime = distsToPickup + minDistsToDropoff;
            const DistanceLabel minWaitViolationCost = F::calcKWaitViolationCosts(minActualDepTimeAtPickup, reqState);
            const DistanceLabel minTripCost = F::calcKTripCosts(minTripTime, reqState);
            // Pickup after last stop so no added trip costs for existing passengers.

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDetour) + minWaitViolationCost + minTripCost;

            // Calculations with INFTY don't work like mathematical infinity, so make infinite costs caused by infinite
            // distances explicit
            costLowerBound.setIf(DistanceLabel(INFTY), ~(distsToPickup < INFTY));

            return costLowerBound;
        }

        template<typename LabelSet>
        static typename LabelSet::DistanceLabel
        calcLowerBoundCostForKPairedAssignmentsAfterLastStop(
                const typename LabelSet::DistanceLabel &detourTillDepAtPickup,
                const typename LabelSet::DistanceLabel &tripTimeTillDepAtPickup,
                const typename LabelSet::DistanceLabel &directDist,
                const typename LabelSet::DistanceLabel &pickupWalkingDists,
                const RequestState &reqState) {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;
            assert(directDist.horizontalMin() >= 0 && directDist.horizontalMax() < INFTY);
            assert(pickupWalkingDists.horizontalMin() >= 0 && pickupWalkingDists.horizontalMax() < INFTY);


            // Calculations with INFTY don't work like mathematical infinity, so set cost to INFTY later.
            const LabelMask inftyMask = ~((detourTillDepAtPickup < INFTY) & (tripTimeTillDepAtPickup < INFTY));
//            const DistanceLabel adaptedVehTimeTillDepAtPickup = select(vehTimeInftyMask, 0, vehTimeTillDepAtPickup);

            const DistanceLabel detourCost = F::calcKVehicleCosts(detourTillDepAtPickup + directDist + InputConfig::getInstance().stopTime);
            const DistanceLabel tripCost = F::calcKTripCosts(tripTimeTillDepAtPickup + directDist, reqState);
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists,
                                                                   InputConfig::getInstance().pickupRadius);
            const DistanceLabel waitViolationCost = F::calcKWaitViolationCosts(
                    reqState.originalRequest.requestTime + tripTimeTillDepAtPickup, reqState);
            // Pickup after last stop so no added trip costs for existing passengers.
            DistanceLabel minCost = detourCost + tripCost + walkingCost + waitViolationCost;

            // Set cost to INFTY where input times were invalid
            minCost.setIf(DistanceLabel(INFTY), inftyMask);

            return minCost;
        }

        template<typename LabelSet>
        static typename LabelSet::DistanceLabel
        calcUpperBoundCostForKPairedAssignmentsAfterLastStop(const Vehicle &veh,
                                                             const typename LabelSet::DistanceLabel &distancesToPickups,
                                                             const typename LabelSet::DistanceLabel &psgArrTimesAtPickups,
                                                             const typename LabelSet::DistanceLabel &distancesToDest,
                                                             const typename LabelSet::DistanceLabel &pickupWalkingDists,
                                                             const RequestState &reqState, const RouteStateData& routeState) {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;
            using namespace time_utils;
            assert(psgArrTimesAtPickups.horizontalMin() >= 0 && psgArrTimesAtPickups.horizontalMax() < INFTY);
            assert(distancesToDest.horizontalMin() >= 0 && distancesToDest.horizontalMax() < INFTY);
            assert(pickupWalkingDists.horizontalMin() >= 0 && pickupWalkingDists.horizontalMax() < INFTY);

            const int &vehId = veh.vehicleId;

            // Calculations with INFTY don't work like mathematical infinity, so use 0 as placeholder value and set cost
            // to INFTY later.
            const LabelMask distToPickupInftyMask = ~(distancesToPickups < INFTY);
            const DistanceLabel adaptedDistToPickup = select(distToPickupInftyMask, 0, distancesToPickups);

            const auto &stopIdx = routeState.numStopsOf(vehId) - 1;
            const int vehDepTimeAtLastStop = getVehDepTimeAtStopForRequest(vehId, stopIdx, reqState, routeState);


            auto depTimesAtPickups = DistanceLabel(vehDepTimeAtLastStop) + adaptedDistToPickup + InputConfig::getInstance().stopTime;
            depTimesAtPickups.max(psgArrTimesAtPickups);

            const DistanceLabel vehTimeTillDepAtPickup = depTimesAtPickups - DistanceLabel(vehDepTimeAtLastStop);
            const DistanceLabel detourCost = F::calcKVehicleCosts(vehTimeTillDepAtPickup + distancesToDest + InputConfig::getInstance().stopTime);

            const DistanceLabel psgTimeTillDepAtPickup = depTimesAtPickups - reqState.originalRequest.requestTime;
            const DistanceLabel tripCost = F::calcKTripCosts(psgTimeTillDepAtPickup + distancesToDest, reqState);
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists,
                                                                   InputConfig::getInstance().pickupRadius);
            const DistanceLabel waitViolationCost = F::calcKWaitViolationCosts(depTimesAtPickups, reqState);

            DistanceLabel cost = detourCost + tripCost + walkingCost + waitViolationCost;

            // Set cost to INFTY where dist was INFTY
            cost.setIf(DistanceLabel(INFTY), distToPickupInftyMask);

            // Check if service time hard constraint is violated for any pairs. Set cost to INFTY if so.
            const LabelMask violatesServiceTime = DistanceLabel(veh.endOfServiceTime) <
                                                  (DistanceLabel(vehDepTimeAtLastStop + 2 * InputConfig::getInstance().stopTime) +
                                                   distancesToPickups +
                                                   distancesToDest);
            cost.setIf(DistanceLabel(INFTY), violatesServiceTime);


            return cost;
        }



        static int calcCostLowerBoundForPickupAfterLastStop(const Vehicle &veh,
                                                     const PDLoc &pickup,
                                                     const int distToPickup,
                                                     const int minDistToDropoff,
                                                     const RequestState &reqState, const RouteStateData& routeState) {
            using namespace time_utils;
            if (distToPickup >= INFTY || minDistToDropoff >= INFTY)
                return INFTY;

            const auto vehId = veh.vehicleId;

            const int numStops = routeState.numStopsOf(vehId);
            const int actualDepTimeAtPickup = getActualDepTimeAtPickup(vehId, numStops - 1, distToPickup, pickup,
                                                                       reqState, routeState);
            const int vehDepTimeAtPrevStop = std::max(routeState.schedDepTimesFor(vehId)[numStops - 1],
                                                      reqState.originalRequest.requestTime);
            const int detourUntilDepAtPickup = actualDepTimeAtPickup - vehDepTimeAtPrevStop;
            assert(!((bool) (detourUntilDepAtPickup < 0)));
            const int minDetour = detourUntilDepAtPickup + minDistToDropoff;

            if (time_utils::isServiceTimeConstraintViolated(veh, reqState, minDetour, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(pickup.walkingDist, InputConfig::getInstance().pickupRadius);
            const int waitViolationCost = F::calcWaitViolationCost(actualDepTimeAtPickup, reqState);
            const int waitTimeIncludingWalking = actualDepTimeAtPickup - reqState.originalRequest.requestTime;
            assert(waitTimeIncludingWalking >= 0);
            const int minTripTime = waitTimeIncludingWalking + reqState.minDirectPDDist;
            const int minTripCost = F::calcTripCost(minTripTime, reqState);

            return F::calcVehicleCost(minDetour) + walkingCost + waitViolationCost + minTripCost;
        }


        static int calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(const int distToDropoff,
                                                                                 const PDLoc &dropoff,
                                                                                 const RequestState &reqState) {
            return calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(dropoff.walkingDist,
                                                                                        distToDropoff,
                                                                                        InputConfig::getInstance().stopTime,
                                                                                        reqState);
        }


        static int calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(const int dropoffWalkingDist,
                                                                                 const int distToDropoff,
                                                                                 const int minTripTimeToLastStop,
                                                                                 const RequestState &reqState) {
            assert(distToDropoff < INFTY);
            if (distToDropoff >= INFTY)
                return INFTY;


            const int minDetour = distToDropoff + InputConfig::getInstance().stopTime;

            const int walkingCost = F::calcWalkingCost(dropoffWalkingDist, InputConfig::getInstance().dropoffRadius);
            const int minTripTime = minTripTimeToLastStop + distToDropoff + dropoffWalkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, reqState);

            // Independent of vehicle so we cannot know here which existing passengers may be affected
            // const int minAddedTripCostOfOthers = 0;

            return F::calcVehicleCost(minDetour) + walkingCost + minTripCost;
        }

        template<typename LabelSet>
        static typename LabelSet::DistanceLabel
        calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinDistToDropoff(
                const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                const typename LabelSet::DistanceLabel &distToDropoff,
                const typename LabelSet::DistanceLabel &minTripTimeToLastStop,
                const RequestState &reqState) {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, set cost to INFTY later.
            const LabelMask inftyMask = ~(distToDropoff < INFTY);

            const DistanceLabel minDropoffDetours = distToDropoff + InputConfig::getInstance().stopTime;
            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists,
                                                                    InputConfig::getInstance().dropoffRadius);
            DistanceLabel minTripTimes =
                    minTripTimeToLastStop + distToDropoff + dropoffWalkingDists;
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, reqState);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;
            costLowerBound.setIf(DistanceLabel(INFTY), inftyMask);

            return costLowerBound;
        }


        static int calcVehicleIndependentCostLowerBoundForDALSWithKnownMinArrTime(const int dropoffWalkingDist,
                                                                           const int minDistToDropoff,
                                                                           const int minArrTimeAtDropoff,
                                                                           const RequestState &reqState) {
            assert(minArrTimeAtDropoff >= reqState.originalRequest.requestTime);
            assert(minDistToDropoff < INFTY);
            if (minDistToDropoff >= INFTY)
                return INFTY;

            const int minDetour = minDistToDropoff + InputConfig::getInstance().stopTime;
            const int walkingCost = F::calcWalkingCost(dropoffWalkingDist, InputConfig::getInstance().dropoffRadius);
            const int minTripTime = minArrTimeAtDropoff - reqState.originalRequest.requestTime + dropoffWalkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, reqState);

            return F::calcVehicleCost(minDetour) + walkingCost + minTripCost;
        }

        template<typename LabelSet>
        static typename LabelSet::DistanceLabel
        calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinArrTime(
                const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                const typename LabelSet::DistanceLabel &minDistToDropoff,
                const typename LabelSet::DistanceLabel &minArrTimeAtDropoff,
                const RequestState &reqState) {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, set cost to INFTY later.
            const LabelMask inftyMask = ~((minDistToDropoff < INFTY) & (minArrTimeAtDropoff < INFTY));

            const DistanceLabel minDropoffDetours = minDistToDropoff + InputConfig::getInstance().stopTime;
            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists,
                                                                    InputConfig::getInstance().dropoffRadius);
            DistanceLabel minTripTimes =
                    minArrTimeAtDropoff + dropoffWalkingDists - DistanceLabel(reqState.originalRequest.requestTime);
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, reqState);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;
            costLowerBound.setIf(DistanceLabel(INFTY), inftyMask);

            return costLowerBound;
        }

        template<typename LabelSet>
        static typename LabelSet::DistanceLabel
        calcKVehicleDependentCostLowerBoundsForDALSWithKnownDistToDropoff(
                const int vehId,
                const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                const typename LabelSet::DistanceLabel &distToDropoff,
                const typename LabelSet::DistanceLabel &minTripTimeToLastStop,
                const RequestState &reqState, const RouteStateData& routeState) {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, set cost to INFTY later.
            const LabelMask inftyMask = ~(distToDropoff < INFTY);

            const DistanceLabel minDropoffDetours = distToDropoff + InputConfig::getInstance().stopTime;
            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists,
                                                                    InputConfig::getInstance().dropoffRadius);
            const auto depTimeAtLastStop = routeState.schedDepTimesFor(vehId)[routeState.numStopsOf(vehId) - 1];
            DistanceLabel minTripTimes =
                    minTripTimeToLastStop + DistanceLabel(depTimeAtLastStop - reqState.originalRequest.requestTime) +
                    distToDropoff + dropoffWalkingDists;
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, reqState);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;
            costLowerBound.setIf(DistanceLabel(INFTY), inftyMask);

            return costLowerBound;
        }


        static int calcCostForPairedAssignmentAfterLastStop(const int vehTimeTillDepAtPickup,
                                                     const int psgTimeTillDepAtPickup,
                                                     const int directDist,
                                                     const int pickupWalkingDist,
                                                     const int dropoffWalkingDist,
                                                     const RequestState &reqState) {
            const int detourCost = F::calcVehicleCost(vehTimeTillDepAtPickup + directDist + InputConfig::getInstance().stopTime);
            const int tripCost = F::calcTripCost(psgTimeTillDepAtPickup + directDist + dropoffWalkingDist, reqState);
            const int walkingCost = F::calcWalkingCost(pickupWalkingDist, InputConfig::getInstance().pickupRadius) +
                                    F::calcWalkingCost(dropoffWalkingDist, InputConfig::getInstance().dropoffRadius);
            const int waitViolationCost = F::calcWaitViolationCost(
                    reqState.originalRequest.requestTime + psgTimeTillDepAtPickup, reqState);
            // Pickup after last stop so no added trip costs for existing passengers.
            return detourCost + tripCost + walkingCost + waitViolationCost;
        }

        // Calculates the cost for the pickup side of an assignment that can be known without knowing which
        // dropoff is used or where the dropoff will be inserted.

        static int calcMinKnownPickupSideCost(const Vehicle &veh, const int pickupIndex, const int initialPickupDetour,
                                       const int walkingDist, const int depTimeAtPickup,
                                       const RequestState &reqState, const RouteStateData& routeState) {
            using namespace time_utils;

            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto residualDetourAtEnd = calcResidualPickupDetour(veh.vehicleId, pickupIndex, numStops - 1,
                                                                      initialPickupDetour, routeState);
            if (isServiceTimeConstraintViolated(veh, reqState, residualDetourAtEnd, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(walkingDist, InputConfig::getInstance().pickupRadius);
            const int addedTripTimeOfOthers = calcAddedTripTimeInInterval(veh.vehicleId, pickupIndex, numStops - 1,
                                                                          initialPickupDetour, routeState);

            const int changeInTripTimeCosts = F::calcChangeInTripCostsOfExistingPassengers(
                    addedTripTimeOfOthers);
            const int waitViolation = F::calcWaitViolationCost(depTimeAtPickup, reqState);
            const int minTripTime = (depTimeAtPickup - reqState.originalRequest.requestTime) + reqState.minDirectPDDist;
            const int minTripCost = F::calcTripCost(minTripTime, reqState);
            return F::calcVehicleCost(initialPickupDetour) + changeInTripTimeCosts
                   + minTripCost
                   + walkingCost + waitViolation;
        }

        // Calculates the cost for the dropoff side of an assignment that can be known without knowing which
        // pickup is used or where the pickup will be inserted.

        static int calcMinKnownDropoffSideCost(const Vehicle &veh, const int dropoffIndex,
                                        const int initialDropoffDetour, const int walkingDist,
                                        const RequestState &reqState, const RouteStateData& routeState) {
            using namespace time_utils;

            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto residualDetourAtEnd = calcResidualTotalDetour(veh.vehicleId, dropoffIndex, dropoffIndex,
                                                                     numStops - 1, 0, initialDropoffDetour, routeState);

            if (isServiceTimeConstraintViolated(veh, reqState, residualDetourAtEnd, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(walkingDist, InputConfig::getInstance().dropoffRadius);
            const int minAddedTripTimeOfOthers = calcAddedTripTimeInInterval(veh.vehicleId, dropoffIndex, numStops - 1,
                                                                             initialDropoffDetour, routeState);
            const int minChangeInTripTimeCosts = F::calcChangeInTripCostsOfExistingPassengers(minAddedTripTimeOfOthers);

            const int minTripTime = reqState.minDirectPDDist + walkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, reqState);

            return F::calcVehicleCost(initialDropoffDetour) + walkingCost + minChangeInTripTimeCosts + minTripCost;
        }


        static inline bool
        isDropoffCostPromisingForAfterLastStop(const PDLoc &dropoff, const RequestState &reqState) {
            const auto walkMinCost =
                    F::calcTripCost(dropoff.walkingDist, reqState) +
                    F::calcWalkingCost(dropoff.walkingDist, InputConfig::getInstance().dropoffRadius);
            const auto vehMinCost =
                    F::calcVehicleCost(dropoff.vehDistToCenter) + F::calcTripCost(dropoff.vehDistToCenter, reqState);
            return walkMinCost <= vehMinCost;
        }


    private:


        static int calcCost(const Assignment &asgn,
                     const RequestState &reqState, const RouteStateData& routeState, const int initialPickupDetour,
                     const int residualDetourAtEnd, const int depTimeAtPickup,
                     const bool dropoffAtExistingStop, const int addedTripTimeForExistingPassengers) {
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return INFTY;

            using namespace time_utils;
            const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, asgn, initialPickupDetour,
                                                              dropoffAtExistingStop, routeState);
            const int tripTime = arrTimeAtDropoff - reqState.originalRequest.requestTime + asgn.dropoff->walkingDist;

            const auto walkingCost =
                    F::calcWalkingCost(asgn.pickup->walkingDist, InputConfig::getInstance().pickupRadius) +
                    F::calcWalkingCost(asgn.dropoff->walkingDist, InputConfig::getInstance().dropoffRadius);
            const auto tripCost = F::calcTripCost(tripTime, reqState);
            const auto waitTimeViolationCost = F::calcWaitViolationCost(depTimeAtPickup, reqState);
            const auto changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(
                    addedTripTimeForExistingPassengers);
            const auto vehCost = F::calcVehicleCost(residualDetourAtEnd);

            return vehCost + walkingCost + tripCost + waitTimeViolationCost + changeInTripCostsOfOthers;

        }
    };

    static constexpr int PSG_COST_SCALE = KARRI_PSG_COST_SCALE; // CMake compile time parameter
    static constexpr int VEH_COST_SCALE = KARRI_VEH_COST_SCALE; // CMake compile time parameter
    static constexpr int WALKING_COST_SCALE = KARRI_WALKING_COST_SCALE; // CMake compile time parameter
    static constexpr int WAIT_PENALTY_SCALE = KARRI_WAIT_PENALTY_SCALE; // CMake compile time parameter
    static constexpr int TRIP_PENALTY_SCALE = KARRI_TRIP_PENALTY_SCALE; // CMake compile time parameter
    using Calc = CostCalculatorTemplate<TimeIsMoneyCostFunction<PSG_COST_SCALE, WALKING_COST_SCALE, VEH_COST_SCALE, WAIT_PENALTY_SCALE, TRIP_PENALTY_SCALE>>;
}