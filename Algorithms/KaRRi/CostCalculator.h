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
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Tools/Constants.h"
#include "Tools/Workarounds.h"
#include "Algorithms/KaRRi/AssignmentCostFunctions/TimeIsMoneyCostFunction.h"

namespace karri {

// A facility for computing the cost of an assignment of a request into a vehicle's route.
    template<typename CostFunctionT, typename RouteStateT>
    class CostCalculatorTemplate {

        using F = CostFunctionT;

    public:

        using CostFunction = CostFunctionT;

        explicit CostCalculatorTemplate(const RouteStateT &routeState,
                                        const InputConfig &inputConfig)
                : routeState(routeState),
                  inputConfig(inputConfig),
                  stopTime(inputConfig.stopTime) {}

        template<typename RequestContext>
        int calc(const Assignment &asgn, const RequestContext &context) const {
            return calcBase<true>(asgn, context);
        }

        template<typename RequestContext>
        int calcWithoutHardConstraints(const Assignment &asgn, const RequestContext &context) const {
            return calcBase<false>(asgn, context);
        }

        // Calculates the objective value for a given assignment.
        template<bool checkHardConstraints, typename RequestContext>
        int calcBase(const Assignment &asgn, const RequestContext &context) const {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup && asgn.dropoff);
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return INFTY;

            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY)
                return INFTY;
            const int vehId = asgn.vehicle->vehicleId;
            const auto numStops = routeState.numStopsOf(asgn.vehicle->vehicleId);
            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, context, routeState, inputConfig);
            const auto initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, context, routeState);

            int addedTripTime = calcAddedTripTimeInInterval(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                                            initialPickupDetour, routeState);

            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            const auto initialDropoffDetour = calcInitialDropoffDetour(asgn, dropoffAtExistingStop, routeState,
                                                                       inputConfig);
            const auto detourRightAfterDropoff = calcDetourRightAfterDropoff(asgn, initialPickupDetour,
                                                                             initialDropoffDetour, routeState);
            const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(asgn.vehicle->vehicleId,
                                                                                        asgn.dropoffStopIdx,
                                                                                        numStops - 1,
                                                                                        detourRightAfterDropoff,
                                                                                        routeState);

            if (checkHardConstraints && isAnyHardConstraintViolated(asgn, context, initialPickupDetour,
                                                                    detourRightAfterDropoff, residualDetourAtEnd,
                                                                    dropoffAtExistingStop, routeState))
                return INFTY;

            addedTripTime += calcAddedTripTimeAffectedByPickupAndDropoff(asgn, detourRightAfterDropoff, routeState);

            return calcCost(asgn, context, initialPickupDetour, residualDetourAtEnd,
                            actualDepTimeAtPickup, dropoffAtExistingStop, addedTripTime);
        }

        // Calculate the cost for a passenger moving to their destination independently without using a vehicle.
        template<typename RequestContext>
        static int calcCostForNotUsingVehicle(const int walkingDist, const int travelTimeOfDestEdge,
                                              const RequestContext &context, const InputConfig &inputConfig) {
            assert(walkingDist >= travelTimeOfDestEdge);

            if (walkingDist >= INFTY)
                return INFTY;

            int destSideDist;
            if (walkingDist <= inputConfig.pickupRadius + inputConfig.dropoffRadius) {
                // Wait time violation only necessary if travel time of destination edge is already larger than dropoff
                // radius (since we always have to traverse destination edge from tail to head).
                destSideDist = std::max(inputConfig.dropoffRadius, travelTimeOfDestEdge);
            } else {
                // Optimally split walking distance in origin side and destination side to minimize wait time violations:
                const int halfOfVioDist = std::floor(
                        0.5 * (walkingDist - (inputConfig.pickupRadius + inputConfig.dropoffRadius)));
                destSideDist = std::max(inputConfig.dropoffRadius + halfOfVioDist, travelTimeOfDestEdge);
            }

            const auto originSideDist = walkingDist - destSideDist;
            const auto walkingCost = F::calcWalkingCost(originSideDist, inputConfig.pickupRadius) +
                                     F::calcWalkingCost(destSideDist, inputConfig.dropoffRadius);
            const auto tripCost = F::calcTripCost(originSideDist + destSideDist, context);
            // no costs for detour, wait violation or change in trip time of other passengers
            return walkingCost + tripCost;
        }

        // For lower bound, pass the assignment consisting of the pickup with the smallest distance from the
        // previous stop, and the dropoff with the smallest distance to the next stop. Uses a lower bound on every
        // PD-distance.
        template<typename RequestContext>
        int calcCostLowerBoundForOrdinaryPairedAssignment(const Assignment &asgn, const RequestContext &context) const {
            using namespace time_utils;
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return INFTY;
            assert(asgn.pickupStopIdx == asgn.dropoffStopIdx);
            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY)
                return INFTY;

            const PDLoc &pickupWithMinDistToPrev = *asgn.pickup;

            const auto vehId = asgn.vehicle->vehicleId;
            const bool pickupAtExistingStop = isPickupAtExistingStop(pickupWithMinDistToPrev, vehId, context.now(),
                                                                     asgn.pickupStopIdx, routeState);
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            const auto initialPickupDetour = calcOnlyDrivingTimeInInitialPickupDetour(asgn, pickupAtExistingStop,
                                                                                      routeState);
            const auto initialDropoffDetour = calcInitialDropoffDetour(asgn, dropoffAtExistingStop, routeState,
                                                                       inputConfig);
            const auto detourRightAfterDropoff = calcDetourRightAfterDropoff(asgn, initialPickupDetour,
                                                                             initialDropoffDetour,
                                                                             routeState);

            const auto totalVehicleDetour = calcResidualTotalDetour(asgn, routeState.numStopsOf(vehId) - 1,
                                                                    initialPickupDetour, detourRightAfterDropoff,
                                                                    routeState);
            if (isAnyHardConstraintViolated(asgn, context, initialPickupDetour, detourRightAfterDropoff,
                                            totalVehicleDetour,
                                            dropoffAtExistingStop, routeState))
                return INFTY;

            const int tripTimeLowerBound = context.originalReqDirectDist;

            const auto tripCostLowerBound = F::calcTripCost(tripTimeLowerBound, context);
            const auto &numStops = routeState.numStopsOf(vehId);
            const auto minAddedTripCostOfOthers =
                    F::calcChangeInTripCostsOfExistingPassengers(
                            calcAddedTripTimeInInterval(vehId, asgn.dropoffStopIdx, numStops - 1,
                                                        detourRightAfterDropoff, routeState));

            return F::calcVehicleCost(totalVehicleDetour) + tripCostLowerBound + minAddedTripCostOfOthers;
        }

        template<typename RequestContext>
        int calcCostLowerBoundForPairedAssignmentBeforeNextStop(const Vehicle &veh, const PDLoc &pickup,
                                                                const int distToPickup,
                                                                const int minDistToDropoff,
                                                                const int distFromPickupForDetourLowerBound,
                                                                const RequestContext &context) const {
            using namespace time_utils;

            const int vehId = veh.vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);

            Assignment asgn(&veh, &pickup);
            asgn.distToPickup = distToPickup;
            asgn.distToDropoff = minDistToDropoff;

            const int minActualDepTimeAtPickup = getActualDepTimeAtPickup(vehId, 0, distToPickup, pickup, context,
                                                                          routeState, inputConfig);

            const auto initialPickupDetour = calcInitialPickupDetour(asgn, minActualDepTimeAtPickup, context,
                                                                     routeState);
            const auto minInitialDropoffDetour = std::max(minDistToDropoff, distFromPickupForDetourLowerBound) -
                                                 calcLengthOfLegStartingAt(0, vehId, routeState);
            auto minDetourRightAfterDropoff =
                    calcDetourRightAfterDropoff(vehId, 0, 0, initialPickupDetour, minInitialDropoffDetour, routeState);
            minDetourRightAfterDropoff = std::max(minDetourRightAfterDropoff, 0);
            const auto residualDetourAtEnd = calcResidualTotalDetour(vehId, 0, 0, numStops - 1, initialPickupDetour,
                                                                     minDetourRightAfterDropoff, routeState);
            if (isAnyHardConstraintViolated(asgn, context, initialPickupDetour, minDetourRightAfterDropoff,
                                            residualDetourAtEnd, false, routeState))
                return INFTY;


            const int minTripTime = minActualDepTimeAtPickup - context.originalRequest.requestTime + minDistToDropoff;
            const int walkingCost = F::calcWalkingCost(pickup.walkingDist, inputConfig.pickupRadius);
            const int minWaitViolationCost = F::calcWaitViolationCost(minActualDepTimeAtPickup, context);
            const int minTripCost = F::calcTripCost(minTripTime, context);

            const int minAddedTripCostOfOthers = F::calcChangeInTripCostsOfExistingPassengers(
                    calcAddedTripTimeInInterval(vehId, 0, numStops - 1, minDetourRightAfterDropoff, routeState));

            return F::calcVehicleCost(residualDetourAtEnd) + walkingCost + minWaitViolationCost + minTripCost +
                   minAddedTripCostOfOthers;
        }


        template<typename RequestContext>
        int calcCostLowerBoundForPickupAfterLastStopIndependentOfVehicle(const int distToPickup,
                                                                         const int minDistToDropoff,
                                                                         const RequestContext &context) const {
            if (distToPickup >= INFTY)
                return INFTY;

            const int minDetour = distToPickup + minDistToDropoff + stopTime;

            const int minActualDepTimeAtPickup = context.originalRequest.requestTime + distToPickup;
            const int minTripTime = minActualDepTimeAtPickup - context.originalRequest.requestTime + minDistToDropoff;
            const int minWaitViolationCost = F::calcWaitViolationCost(minActualDepTimeAtPickup, context);
            const int minTripCost = F::calcTripCost(minTripTime, context);

            // Pickup after last stop so no added trip costs for existing passengers.

            return F::calcVehicleCost(minDetour) + minWaitViolationCost + minTripCost;
        }

        template<typename DistanceLabel, typename RequestContext>
        DistanceLabel
        calcCostLowerBoundForKPickupsAfterLastStopIndependentOfVehicle(const DistanceLabel &distsToPickup,
                                                                       const DistanceLabel &minDistsToDropoff,
                                                                       const RequestContext &context) const {
            const DistanceLabel minDetour = distsToPickup + minDistsToDropoff + stopTime;


            const DistanceLabel minActualDepTimeAtPickup = context.originalRequest.requestTime + distsToPickup;
            const DistanceLabel minTripTime = distsToPickup + minDistsToDropoff;
            const DistanceLabel minWaitViolationCost = F::calcKWaitViolationCosts(minActualDepTimeAtPickup, context);
            const DistanceLabel minTripCost = F::calcKTripCosts(minTripTime, context);
            // Pickup after last stop so no added trip costs for existing passengers.

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDetour) + minWaitViolationCost + minTripCost;

            // Calculations with INFTY don't work like mathematical infinity, so make infinite costs caused by infinite
            // distances explicit
            costLowerBound.setIf(DistanceLabel(INFTY), ~(distsToPickup < INFTY));

            return costLowerBound;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcLowerBoundCostForKPairedAssignmentsAfterLastStop(
                const typename LabelSet::DistanceLabel &distToPickup,
                const typename LabelSet::DistanceLabel &directDist,
                const typename LabelSet::DistanceLabel &pickupWalkingDists,
                const RequestContext &context) const {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;
            assert(directDist.horizontalMin() >= 0 && directDist.horizontalMax() < INFTY);
            assert(pickupWalkingDists.horizontalMin() >= 0 && pickupWalkingDists.horizontalMax() < INFTY);


            // Calculations with INFTY don't work like mathematical infinity, so use 0 as placeholder value and set cost
            // to INFTY later.
            const LabelMask distToPickupInftyMask = ~(distToPickup < INFTY);
            const DistanceLabel adaptedDistToPickup = select(distToPickupInftyMask, 0, distToPickup);

            const DistanceLabel detourCost = F::calcKVehicleCosts(adaptedDistToPickup + directDist + 2 * stopTime);
            DistanceLabel minPsgTimeTillDepAtPickup = adaptedDistToPickup + stopTime;
            minPsgTimeTillDepAtPickup.max(pickupWalkingDists);
            const DistanceLabel tripCost = F::calcKTripCosts(minPsgTimeTillDepAtPickup + directDist, context);
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists, inputConfig.pickupRadius);
            const DistanceLabel waitViolationCost = F::calcKWaitViolationCosts(
                    context.originalRequest.requestTime + minPsgTimeTillDepAtPickup, context);
            // Pickup after last stop so no added trip costs for existing passengers.
            DistanceLabel minCost = detourCost + tripCost + walkingCost + waitViolationCost;

            // Set cost to INFTY where dist was INFTY
            minCost.setIf(DistanceLabel(INFTY), distToPickupInftyMask);

            return minCost;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcUpperBoundCostForKPairedAssignmentsAfterLastStop(const Vehicle &veh,
                                                             const typename LabelSet::DistanceLabel &distancesToPickups,
                                                             const typename LabelSet::DistanceLabel &psgArrTimesAtPickups,
                                                             const typename LabelSet::DistanceLabel &distancesToDest,
                                                             const typename LabelSet::DistanceLabel &pickupWalkingDists,
                                                             const RequestContext &context) const {
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
            const int vehDepTimeAtLastStop = getVehDepTimeAtStopForRequest(vehId, stopIdx, context, routeState);


            auto depTimesAtPickups = DistanceLabel(vehDepTimeAtLastStop) + adaptedDistToPickup + stopTime;
            depTimesAtPickups.max(psgArrTimesAtPickups);

            const DistanceLabel vehTimeTillDepAtPickup = depTimesAtPickups - DistanceLabel(vehDepTimeAtLastStop);
            const DistanceLabel detourCost = F::calcKVehicleCosts(vehTimeTillDepAtPickup + distancesToDest + stopTime);

            const DistanceLabel psgTimeTillDepAtPickup = depTimesAtPickups - context.originalRequest.requestTime;
            const DistanceLabel tripCost = F::calcKTripCosts(psgTimeTillDepAtPickup + distancesToDest, context);
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists, inputConfig.pickupRadius);
            const DistanceLabel waitViolationCost = F::calcKWaitViolationCosts(depTimesAtPickups, context);

            DistanceLabel cost = detourCost + tripCost + walkingCost + waitViolationCost;

            // Set cost to INFTY where dist was INFTY
            cost.setIf(DistanceLabel(INFTY), distToPickupInftyMask);

            // Check if service time hard constraint is violated for any pairs. Set cost to INFTY if so.
            const LabelMask violatesServiceTime = DistanceLabel(veh.endOfServiceTime) <
                                                  (DistanceLabel(vehDepTimeAtLastStop + 2 * stopTime) +
                                                   distancesToPickups +
                                                   distancesToDest);
            cost.setIf(DistanceLabel(INFTY), violatesServiceTime);


            return cost;
        }


        template<typename RequestContext>
        int calcCostLowerBoundForPickupAfterLastStop(const Vehicle &veh,
                                                     const PDLoc &pickup,
                                                     const int distToPickup,
                                                     const int minDistToDropoff,
                                                     const RequestContext &context) const {
            using namespace time_utils;
            if (distToPickup >= INFTY || minDistToDropoff >= INFTY)
                return INFTY;

            const auto vehId = veh.vehicleId;

            const int numStops = routeState.numStopsOf(vehId);
            const int actualDepTimeAtPickup = getActualDepTimeAtPickup(vehId, numStops - 1, distToPickup, pickup,
                                                                       context, routeState, inputConfig);
            const int vehDepTimeAtPrevStop = std::max(routeState.schedDepTimesFor(vehId)[numStops - 1],
                                                      context.originalRequest.requestTime);
            const int detourUntilDepAtPickup = actualDepTimeAtPickup - vehDepTimeAtPrevStop;
            assert(!((bool) (detourUntilDepAtPickup < 0)));
            const int minDetour = detourUntilDepAtPickup + minDistToDropoff;

            if (time_utils::isServiceTimeConstraintViolated(veh, context, minDetour, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(pickup.walkingDist, inputConfig.pickupRadius);
            const int waitViolationCost = F::calcWaitViolationCost(actualDepTimeAtPickup, context);
            const int waitTimeIncludingWalking = actualDepTimeAtPickup - context.originalRequest.requestTime;
            assert(waitTimeIncludingWalking >= 0);
            const int minTripTime = waitTimeIncludingWalking + context.minDirectPDDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);

            return F::calcVehicleCost(minDetour) + walkingCost + waitViolationCost + minTripCost;
        }

        template<typename RequestContext>
        int calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(const int distToDropoff,
                                                                          const PDLoc &dropoff,
                                                                          const RequestContext &context) const {
            return calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(dropoff.walkingDist, distToDropoff,
                                                                                 stopTime, context);
        }

        template<typename RequestContext>
        int calcCostLowerBoundForDropoffAfterLastStopIndependentOfVehicle(const int dropoffWalkingDist,
                                                                          const int distToDropoff,
                                                                          const int minTripTimeToLastStop,
                                                                          const RequestContext &context) const {
            assert(distToDropoff < INFTY);
            if (distToDropoff >= INFTY)
                return INFTY;


            const int minDetour = distToDropoff + stopTime;

            const int walkingCost = F::calcWalkingCost(dropoffWalkingDist, inputConfig.dropoffRadius);
            const int minTripTime = minTripTimeToLastStop + distToDropoff + dropoffWalkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);

            // Independent of vehicle so we cannot know here which existing passengers may be affected
            // const int minAddedTripCostOfOthers = 0;

            return F::calcVehicleCost(minDetour) + walkingCost + minTripCost;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcCostLowerBoundForKDropoffsAfterLastStop(const typename LabelSet::DistanceLabel &distancesToDropoffs,
                                                    const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                                                    const RequestContext &context) const {

            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, turn the values into usable placeholder values (revert back later).
            const LabelMask distToDropoffInftyMask = ~(distancesToDropoffs < INFTY);
            const DistanceLabel adaptedDistToDropoff = select(distToDropoffInftyMask, 0, distancesToDropoffs);

            const DistanceLabel minDropoffDetours = adaptedDistToDropoff + stopTime;

            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists, inputConfig.dropoffRadius);

            DistanceLabel minTripTimes = adaptedDistToDropoff + dropoffWalkingDists;
            minTripTimes.max(context.originalReqDirectDist);
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, context);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;

            // Revert placeholder values at indices i where distancesToDropoffs >= INFTY back to INFTY.
            costLowerBound.setIf(DistanceLabel(INFTY), distToDropoffInftyMask);

            return costLowerBound;
        }

        template<typename RequestContext>
        int calcCostForPairedAssignmentAfterLastStop(const int vehTimeTillDepAtPickup,
                                                     const int psgTimeTillDepAtPickup,
                                                     const int directDist,
                                                     const int pickupWalkingDist,
                                                     const int dropoffWalkingDist,
                                                     const RequestContext &context) const {
            const int detourCost = F::calcVehicleCost(vehTimeTillDepAtPickup + directDist + stopTime);
            const int tripCost = F::calcTripCost(psgTimeTillDepAtPickup + directDist + dropoffWalkingDist, context);
            const int walkingCost = F::calcWalkingCost(pickupWalkingDist, inputConfig.pickupRadius) +
                                    F::calcWalkingCost(dropoffWalkingDist, inputConfig.dropoffRadius);
            const int waitViolationCost = F::calcWaitViolationCost(
                    context.originalRequest.requestTime + psgTimeTillDepAtPickup, context);
            // Pickup after last stop so no added trip costs for existing passengers.
            return detourCost + tripCost + walkingCost + waitViolationCost;
        }

        // Calculates the cost for the pickup side of an assignment that can be known without knowing which
        // dropoff is used or where the dropoff will be inserted.
        template<typename RequestContext>
        int calcMinKnownPickupSideCost(const Vehicle &veh, const int pickupIndex, const int initialPickupDetour,
                                       const int walkingDist, const int depTimeAtPickup,
                                       const RequestContext &context) const {
            using namespace time_utils;

            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto residualDetourAtEnd = calcResidualPickupDetour(veh.vehicleId, pickupIndex, numStops - 1,
                                                                      initialPickupDetour, routeState);
            if (isServiceTimeConstraintViolated(veh, context, residualDetourAtEnd, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(walkingDist, inputConfig.pickupRadius);
            const int addedTripTimeOfOthers = calcAddedTripTimeInInterval(veh.vehicleId, pickupIndex, numStops - 1,
                                                                          initialPickupDetour, routeState);

            const int changeInTripTimeCosts = F::calcChangeInTripCostsOfExistingPassengers(
                    addedTripTimeOfOthers);
            const int waitViolation = F::calcWaitViolationCost(depTimeAtPickup, context);
            const int minTripTime = (depTimeAtPickup - context.originalRequest.requestTime) + context.minDirectPDDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);
            return F::calcVehicleCost(initialPickupDetour) + changeInTripTimeCosts
                   + minTripCost
                   + walkingCost + waitViolation;
        }

        // Calculates the cost for the dropoff side of an assignment that can be known without knowing which
        // pickup is used or where the pickup will be inserted.
        template<typename RequestContext>
        int calcMinKnownDropoffSideCost(const Vehicle &veh, const int dropoffIndex,
                                        const int initialDropoffDetour, const int walkingDist,
                                        const RequestContext &context) const {
            using namespace time_utils;

            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto residualDetourAtEnd = calcResidualTotalDetour(veh.vehicleId, dropoffIndex, dropoffIndex,
                                                                     numStops - 1, 0, initialDropoffDetour, routeState);

            if (isServiceTimeConstraintViolated(veh, context, residualDetourAtEnd, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(walkingDist, inputConfig.dropoffRadius);
            const int minAddedTripTimeOfOthers = calcAddedTripTimeInInterval(veh.vehicleId, dropoffIndex, numStops - 1,
                                                                             initialDropoffDetour, routeState);
            const int minChangeInTripTimeCosts = F::calcChangeInTripCostsOfExistingPassengers(minAddedTripTimeOfOthers);

            const int minTripTime = context.minDirectPDDist + walkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);

            return F::calcVehicleCost(initialDropoffDetour) + walkingCost + minChangeInTripTimeCosts + minTripCost;
        }

        template<typename RequestContext>
        inline bool
        isDropoffCostPromisingForAfterLastStop(const PDLoc &dropoff, const RequestContext &context) const {
            const auto walkMinCost =
                    F::calcTripCost(dropoff.walkingDist, context) +
                    F::calcWalkingCost(dropoff.walkingDist, inputConfig.dropoffRadius);
            const auto vehMinCost =
                    F::calcVehicleCost(dropoff.vehDistToCenter) + F::calcTripCost(dropoff.vehDistToCenter, context);
            return walkMinCost <= vehMinCost;
        }


    private:

        template<typename RequestContext>
        int calcCost(const Assignment &asgn,
                     const RequestContext &context, const int initialPickupDetour,
                     const int residualDetourAtEnd, const int depTimeAtPickup,
                     const bool dropoffAtExistingStop, const int addedTripTimeForExistingPassengers) const {
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return INFTY;

            using namespace time_utils;
            const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, asgn, initialPickupDetour,
                                                              dropoffAtExistingStop, routeState, inputConfig);
            const int tripTime = arrTimeAtDropoff - context.originalRequest.requestTime + asgn.dropoff->walkingDist;

            const auto walkingCost = F::calcWalkingCost(asgn.pickup->walkingDist, inputConfig.pickupRadius) +
                                     F::calcWalkingCost(asgn.dropoff->walkingDist, inputConfig.dropoffRadius);
            const auto tripCost = F::calcTripCost(tripTime, context);
            const auto waitTimeViolationCost = F::calcWaitViolationCost(depTimeAtPickup, context);
            const auto changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(
                    addedTripTimeForExistingPassengers);
            const auto vehCost = F::calcVehicleCost(residualDetourAtEnd);

            return vehCost + walkingCost + tripCost + waitTimeViolationCost + changeInTripCostsOfOthers;

        }

        const RouteStateT &routeState;
        const InputConfig &inputConfig;
        const int stopTime;
    };

    static constexpr int PSG_COST_SCALE = KARRI_PSG_COST_SCALE; // CMake compile time parameter
    static constexpr int VEH_COST_SCALE = KARRI_VEH_COST_SCALE; // CMake compile time parameter
    static constexpr int WALKING_COST_SCALE = KARRI_WALKING_COST_SCALE; // CMake compile time parameter
    static constexpr int WAIT_PENALTY_SCALE = KARRI_WAIT_PENALTY_SCALE; // CMake compile time parameter
    static constexpr int TRIP_PENALTY_SCALE = KARRI_TRIP_PENALTY_SCALE; // CMake compile time parameter
    template<typename RouteStateT>
    using CostCalculator = CostCalculatorTemplate<TimeIsMoneyCostFunction<PSG_COST_SCALE, WALKING_COST_SCALE, VEH_COST_SCALE, WAIT_PENALTY_SCALE, TRIP_PENALTY_SCALE>, RouteStateT>;
}