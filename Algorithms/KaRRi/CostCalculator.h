/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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
#include "Algorithms/KaRRi/TransferPoints/TransferPoint.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Tools/Constants.h"
#include "Tools/Workarounds.h"
#include "Algorithms/KaRRi/AssignmentCostFunctions/TimeIsMoneyCostFunction.h"

namespace karri {

    // A facility for computing the cost of an assignment of a request into a vehicle's route.
    template<typename CostFunctionT>
    class CostCalculatorTemplate {

        using F = CostFunctionT;

    public:

        using CostFunction = CostFunctionT;

        explicit CostCalculatorTemplate(const RouteState &routeState, const Fleet &fleet)
                : routeState(routeState),
                  fleet(fleet),
                  stopTime(InputConfig::getInstance().stopTime),
                  detourComputer(routeState) {}

        template<typename RequestContext>
        RequestCost calc(const Assignment &asgn, const RequestContext &context) {
            return calcBase<true>(asgn, context);
        }

        template<typename RequestContext>
        RequestCost calcWithoutHardConstraints(const Assignment &asgn, const RequestContext &context) {
            return calcBase<false>(asgn, context);
        }

        // Calculates the objective value for a given assignment.
        template<bool checkHardConstraints, typename RequestContext>
        RequestCost calcBase(const Assignment &asgn, const RequestContext &context) {

            using namespace time_utils;
            KASSERT(asgn.vehicle && asgn.pickup && asgn.dropoff);
            if (!asgn.vehicle || !asgn.pickup || !asgn.dropoff)
                return RequestCost::INFTY_COST();

            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY)
                return RequestCost::INFTY_COST();
            const int vehId = asgn.vehicle->vehicleId;
            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, context, routeState);
            const bool pickupAtExistingStop = isPickupAtExistingStop(*asgn.pickup, vehId, context.now(),
                                                                    asgn.pickupStopIdx, routeState);
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            if constexpr (checkHardConstraints) {
                const auto initialPickupDetour = calcInitialPickupDetour(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                                                         actualDepTimeAtPickup, asgn.distFromPickup,
                                                                         context, routeState);
                if (doesPickupDetourViolateRiderHardConstraints(vehId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                                                initialPickupDetour, routeState))
                    return RequestCost::INFTY_COST();

                if (isOccupancyConstraintViolated(*asgn.vehicle, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                                  dropoffAtExistingStop, context, routeState))
                    return RequestCost::INFTY_COST();
            }

            // Compute detour and added trip times for existing riders for every vehicle that is affected
            // through transfer dependencies
            detourComputer.computeDetours(asgn, context);
            int addedTripTime = 0;
            int totalResidualDetours = 0;
            for (const auto &stopId: detourComputer.stopIdsSeen) {
                const int newArrTime = detourComputer.newArrTimes[stopId];
                const int newDepTime = detourComputer.newDepTimes[stopId];
                const int stopIdx = routeState.stopPositionOf(stopId);
                const int stopVehId = routeState.vehicleIdOf(stopId);
                const int schedArrTime = routeState.schedArrTimesFor(stopVehId)[stopIdx];
                if (newArrTime > schedArrTime) {
                    KASSERT(stopIdx > 0);
                    const auto numDropoffsPs = routeState.numDropoffsPrefixSumFor(stopVehId);
                    const int numDropoffs = numDropoffsPs[stopIdx] - numDropoffsPs[stopIdx - 1];
                    addedTripTime += (newArrTime - schedArrTime) * numDropoffs;
                }
                if (stopIdx == routeState.numStopsOf(stopVehId) - 1) {
                    const int schedDepTime = routeState.schedDepTimesFor(stopVehId)[stopIdx];
                    const int residualDetour = std::max(0, newDepTime - schedDepTime);
                    // For every vehicle whose last stop is reached later due to the assignment, check whether the
                    // service time constraint still holds.
                    if (checkHardConstraints &&
                        isServiceTimeConstraintViolated(fleet[stopVehId], context, residualDetour, routeState))
                        return RequestCost::INFTY_COST();

                    totalResidualDetours += residualDetour;
                }
            }

            const int numStops = routeState.numStopsOf(vehId);

            // Count the detour for pickup and/or dropoff after last stop.
            if (asgn.dropoffStopIdx == numStops - 1 && !dropoffAtExistingStop) {
                totalResidualDetours += asgn.distToDropoff + stopTime;
            }
            if (asgn.pickupStopIdx == numStops - 1 && !pickupAtExistingStop) {
                totalResidualDetours += asgn.distToPickup + stopTime;
            }

            if (checkHardConstraints && asgn.dropoffStopIdx < numStops - 1) {
                const int depTimeRightAfterDropoff = detourComputer.newDepTimes[routeState.stopIdsFor(vehId)[
                        asgn.dropoffStopIdx + 1]];
                const int detourRightAfterDropoff = depTimeRightAfterDropoff -
                                                    routeState.schedDepTimesFor(vehId)[asgn.dropoffStopIdx + 1];
                if (detourRightAfterDropoff > 0 &&
                    doesDropoffDetourViolateRiderHardConstraints(vehId, asgn.dropoffStopIdx, detourRightAfterDropoff,
                                                                 routeState))
                    return RequestCost::INFTY_COST();
            }

            int arrTimeAtDropoff;
            if (asgn.pickupStopIdx == asgn.dropoffStopIdx) {
                arrTimeAtDropoff = actualDepTimeAtPickup + asgn.distToDropoff;
            } else {
                const int stopIdBeforeDropoff = routeState.stopIdsFor(vehId)[asgn.dropoffStopIdx];
                arrTimeAtDropoff = std::max(routeState.schedDepTimesFor(vehId)[asgn.dropoffStopIdx],
                                            detourComputer.newDepTimes[stopIdBeforeDropoff]) +
                                   asgn.distToDropoff;
            }

            if (checkHardConstraints && asgn.dropoffStopIdx == numStops - 1) {
                const int depTimeAtLastStop = std::max(routeState.schedDepTimesFor(vehId)[numStops - 1], context.originalRequest.requestTime);
                const auto residualDetour = arrTimeAtDropoff + (!dropoffAtExistingStop * stopTime) -
                        depTimeAtLastStop;
                if (isServiceTimeConstraintViolated(fleet[vehId], context, residualDetour, routeState))
                    return RequestCost::INFTY_COST();
            }

            const int tripTime = arrTimeAtDropoff - context.originalRequest.requestTime + asgn.dropoff->walkingDist;

            // Apply cost function to time values
            RequestCost cost;
            cost.walkingCost = F::calcWalkingCost(asgn.pickup->walkingDist, InputConfig::getInstance().pickupRadius) +
                               F::calcWalkingCost(asgn.dropoff->walkingDist, InputConfig::getInstance().dropoffRadius);
            cost.tripCost = F::calcTripCost(tripTime, context);
            cost.waitTimeViolationCost = F::calcWaitViolationCost(actualDepTimeAtPickup, context);
            cost.changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(addedTripTime);
            cost.vehCost = F::calcVehicleCost(totalResidualDetours);
            cost.total = cost.vehCost + cost.walkingCost + cost.tripCost + cost.waitTimeViolationCost +
                         cost.changeInTripCostsOfOthers;
            return cost;
        }

        template<typename RequestContext>
        RequestCost calc(AssignmentWithTransfer &asgn, RequestContext &context) {
            return calcBase<true>(asgn, context);
        }

//        template<typename RequestContext>
//        RequestCost calcLowerBound(AssignmentWithTransfer &asgn, RequestContext &context) const {
//            return calcBaseLowerBound<true>(asgn, context);
//        }

        template<bool checkHardConstraints, typename RequestContext>
        RequestCost calcBase(AssignmentWithTransfer &asgn, const RequestContext &context) {
            using namespace time_utils;

            if (!asgn.pVeh || !asgn.dVeh || !asgn.pickup || !asgn.dropoff)
                return RequestCost::INFTY_COST();
            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToTransferPVeh == INFTY || asgn.distFromTransferPVeh == INFTY ||
                asgn.distToTransferDVeh == INFTY || asgn.distFromTransferDVeh == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY) {
                return RequestCost::INFTY_COST();
            }

            const int pVehId = asgn.pVeh->vehicleId;
            const int dVehId = asgn.dVeh->vehicleId;
            const auto numStopsPVeh = routeState.numStopsOf(pVehId);
            const auto numStopsDVeh = routeState.numStopsOf(dVehId);

            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, context, routeState);
            const bool pickupAtExistingStop = isPickupAtExistingStop(*asgn.pickup, pVehId, context.now(),
                                                                     asgn.pickupIdx, routeState);
            const bool transferPVehAtExistingStop = isTransferAtExistingStopPVeh(asgn, routeState);
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            if constexpr (checkHardConstraints) {
                const auto initialPickupDetour = calcInitialPickupDetour(pVehId, asgn.pickupIdx, asgn.transferIdxDVeh,
                                                                         actualDepTimeAtPickup, asgn.distFromPickup,
                                                                         context, routeState);
                if (doesPickupDetourViolateRiderHardConstraints(pVehId, asgn.pickupIdx, asgn.transferIdxPVeh,
                                                                initialPickupDetour, routeState))
                    return RequestCost::INFTY_COST();

                if (isOccupancyConstraintViolated(*asgn.pVeh, asgn.pickupIdx, asgn.transferIdxPVeh,
                                                  transferPVehAtExistingStop, context, routeState))
                    return RequestCost::INFTY_COST();

                if (isOccupancyConstraintViolated(*asgn.dVeh, asgn.transferIdxDVeh, asgn.dropoffIdx,
                                                  dropoffAtExistingStop, context, routeState))
                    return RequestCost::INFTY_COST();
            }

            // Compute detour and added trip times for existing riders for every vehicle that is affected
            // through transfer dependencies
            detourComputer.computeDetours(asgn, context);
            int addedTripTime = 0;
            int totalResidualDetours = 0;
            for (const auto &stopId: detourComputer.stopIdsSeen) {
                const int newArrTime = detourComputer.newArrTimes[stopId];
                const int newDepTime = detourComputer.newDepTimes[stopId];
                const int stopIdx = routeState.stopPositionOf(stopId);
                const int stopVehId = routeState.vehicleIdOf(stopId);
                const int schedArrTime = routeState.schedArrTimesFor(stopVehId)[stopIdx];
                if (newArrTime > schedArrTime) {
                    KASSERT(stopIdx > 0);
                    const auto numDropoffsPs = routeState.numDropoffsPrefixSumFor(stopVehId);
                    const int numDropoffs = numDropoffsPs[stopIdx] - numDropoffsPs[stopIdx - 1];
                    addedTripTime += (newArrTime - schedArrTime) * numDropoffs;
                }
                if (stopIdx == routeState.numStopsOf(stopVehId) - 1) {
                    const int schedDepTime = routeState.schedDepTimesFor(stopVehId)[stopIdx];
                    const int residualDetour = std::max(0, newDepTime - schedDepTime);
                    // For every vehicle whose last stop is reached later due to the assignment, check whether the
                    // service time constraint still holds.
                    if (checkHardConstraints &&
                        isServiceTimeConstraintViolated(fleet[stopVehId], context, residualDetour, routeState))
                        return RequestCost::INFTY_COST();

                    totalResidualDetours += residualDetour;
                }
            }

            // Count the detour for pickup and/or transfer after last stop of pVeh.
            if (asgn.transferIdxPVeh == numStopsPVeh - 1 && !transferPVehAtExistingStop) {
                totalResidualDetours += asgn.distToTransferPVeh + stopTime;
            }
            if (asgn.pickupIdx == numStopsPVeh - 1 && !pickupAtExistingStop) {
                totalResidualDetours += asgn.distToPickup + stopTime;
            }


            const int riderArrTimeAtTransfer = computeRiderArrTimeAtTransfer(asgn, actualDepTimeAtPickup,
                                                                             transferPVehAtExistingStop, detourComputer,
                                                                             routeState);
            const int depTimeAtTransfer = computeDVehDepTimeAtTransfer(asgn, riderArrTimeAtTransfer, detourComputer,
                                                                       routeState, context);

            // Count the detour for transfer and/or dropoff after last stop of dVeh.
            if (asgn.dropoffIdx == numStopsDVeh - 1) {
                totalResidualDetours += asgn.distToDropoff + stopTime;
                if (asgn.transferIdxDVeh == numStopsDVeh - 1) {
                    totalResidualDetours += depTimeAtTransfer - routeState.schedDepTimesFor(dVehId)[numStopsDVeh - 1];
                }
            }

            // Check if service time constraint for pickup vehicle is violated for case that transfer is at last stop
            if (checkHardConstraints && asgn.transferIdxPVeh == numStopsPVeh - 1) {
                const int depTimeAtLastStop = std::max(routeState.schedDepTimesFor(pVehId)[numStopsPVeh - 1], context.originalRequest.requestTime);
                const auto residualDetour = riderArrTimeAtTransfer + (!transferPVehAtExistingStop * stopTime) -
                                            depTimeAtLastStop;
                if (isServiceTimeConstraintViolated(fleet[pVehId], context, residualDetour, routeState))
                    return RequestCost::INFTY_COST();
            }

            if constexpr (checkHardConstraints) {
                // Check rider hard constraints with detour after transfer in pVeh
                if (asgn.transferIdxPVeh < numStopsPVeh - 1) {
                    const int depTimeRightAfterTransferPVeh =
                            detourComputer.newDepTimes[routeState.stopIdsFor(pVehId)[asgn.transferIdxPVeh + 1]];
                    const int detourRightAfterTransferPVeh =
                            depTimeRightAfterTransferPVeh - routeState.schedDepTimesFor(pVehId)[
                                    asgn.transferIdxPVeh + 1];
                    if (detourRightAfterTransferPVeh > 0 &&
                        doesDropoffDetourViolateRiderHardConstraints(pVehId, asgn.transferIdxPVeh,
                                                                     detourRightAfterTransferPVeh, routeState))
                        return RequestCost::INFTY_COST();
                }

                // Check rider hard constraints with detour after transfer in dVeh
                if (asgn.transferIdxDVeh < numStopsDVeh - 1) {
                    const int depTimeRightAfterTransferDVeh =
                            detourComputer.newDepTimes[routeState.stopIdsFor(dVehId)[asgn.transferIdxDVeh + 1]];
                    const int detourRightAfterTransferDVeh =
                            depTimeRightAfterTransferDVeh - routeState.schedDepTimesFor(dVehId)[
                                    asgn.transferIdxDVeh + 1];
                    if (detourRightAfterTransferDVeh > 0 &&
                        doesPickupDetourViolateRiderHardConstraints(dVehId, asgn.transferIdxDVeh, asgn.dropoffIdx,
                                                                    detourRightAfterTransferDVeh, routeState))
                        return RequestCost::INFTY_COST();
                }

                // Check rider hard constraints with detour after dropoff in dVeh
                if (asgn.dropoffIdx < numStopsDVeh - 1) {
                    const int depTimeRightAfterDropoff =
                            detourComputer.newDepTimes[routeState.stopIdsFor(dVehId)[asgn.dropoffIdx + 1]];
                    const int detourRightAfterDropoff =
                            depTimeRightAfterDropoff - routeState.schedDepTimesFor(dVehId)[asgn.dropoffIdx + 1];
                    if (detourRightAfterDropoff > 0 &&
                        doesDropoffDetourViolateRiderHardConstraints(dVehId, asgn.dropoffIdx, detourRightAfterDropoff,
                                                                     routeState))
                        return RequestCost::INFTY_COST();
                }
            }

            // Compute trip time of new rider
            const int arrTimeAtDropoff = computeArrTimeAtDropoffAfterTransfer(
                    asgn, depTimeAtTransfer, detourComputer, routeState);

            // Check if service time constraint for dVeh is violated for case that dropoff is at last stop
            if (checkHardConstraints && asgn.dropoffIdx == numStopsDVeh - 1) {
                const int depTimeAtLastStop = std::max(routeState.schedDepTimesFor(dVehId)[numStopsDVeh - 1], context.originalRequest.requestTime);
                const auto residualDetour = arrTimeAtDropoff + (!dropoffAtExistingStop * stopTime) -
                                            depTimeAtLastStop;
                if (isServiceTimeConstraintViolated(fleet[dVehId], context, residualDetour, routeState))
                    return RequestCost::INFTY_COST();
            }

            const int tripTime = arrTimeAtDropoff - context.originalRequest.requestTime + asgn.dropoff->walkingDist;
            const int waitTimePickup = actualDepTimeAtPickup - context.originalRequest.requestTime;

            // Apply cost function to time values
            RequestCost cost;
            cost.walkingCost = F::calcWalkingCost(asgn.pickup->walkingDist, InputConfig::getInstance().pickupRadius) +
                               F::calcWalkingCost(asgn.dropoff->walkingDist, InputConfig::getInstance().dropoffRadius);
            cost.tripCost = F::calcTripCost(tripTime, context);
            cost.waitTimeViolationCost = F::calcWaitViolationCost(riderArrTimeAtTransfer, depTimeAtTransfer,
                                                                  waitTimePickup, context);
            cost.changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(addedTripTime);
            cost.vehCost = F::calcVehicleCost(totalResidualDetours);
            cost.total = cost.vehCost + cost.walkingCost + cost.tripCost + cost.waitTimeViolationCost +
                         cost.changeInTripCostsOfOthers;
            return cost;
        }

        // Expects an AssignmentWithTransfer that contains all information except the dropoff and dropoff index, i.e.,
        // already has the pickup vehicle, dropoff vehicle, pickup, transfer point, pickup index, and transfer index
        // in both vehicles.
        template<bool checkHardConstraints, typename RequestContext>
        RequestCost calcLowerBoundForPartialOrdinaryTransfer(AssignmentWithTransfer &asgn, const RequestContext &context) {
            using namespace time_utils;
            KASSERT(asgn.pVeh && asgn.dVeh && asgn.pickup);
            KASSERT(!asgn.dropoff);
            KASSERT(asgn.pickupIdx >= 0 && asgn.pickupIdx < routeState.numStopsOf(asgn.pVeh->vehicleId));
            KASSERT(asgn.transferIdxPVeh >= 0 && asgn.transferIdxPVeh < routeState.numStopsOf(asgn.pVeh->vehicleId));
            KASSERT(asgn.transferIdxDVeh >= 0 && asgn.transferIdxDVeh < routeState.numStopsOf(asgn.dVeh->vehicleId));
            KASSERT(asgn.distToPickup != INFTY && asgn.distFromPickup != INFTY &&
                   asgn.distToTransferPVeh != INFTY && asgn.distFromTransferPVeh != INFTY &&
                   asgn.distToTransferDVeh != INFTY && asgn.distFromTransferDVeh != INFTY);

            const int pVehId = asgn.pVeh->vehicleId;
            const int dVehId = asgn.dVeh->vehicleId;
            const auto numStopsPVeh = routeState.numStopsOf(pVehId);
            const auto numStopsDVeh = routeState.numStopsOf(dVehId);

            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, context, routeState);
            const bool pickupAtExistingStop = isPickupAtExistingStop(*asgn.pickup, pVehId, context.now(),
                                                                     asgn.pickupIdx, routeState);
            const bool transferPVehAtExistingStop = isTransferAtExistingStopPVeh(asgn, routeState);
            if constexpr (checkHardConstraints) {
                const auto initialPickupDetour = calcInitialPickupDetour(pVehId, asgn.pickupIdx, asgn.transferIdxDVeh,
                                                                         actualDepTimeAtPickup, asgn.distFromPickup,
                                                                         context, routeState);
                if (doesPickupDetourViolateRiderHardConstraints(pVehId, asgn.pickupIdx, asgn.transferIdxPVeh,
                                                                initialPickupDetour, routeState))
                    return RequestCost::INFTY_COST();

                if (isOccupancyConstraintViolated(*asgn.pVeh, asgn.pickupIdx, asgn.transferIdxPVeh,
                                                  transferPVehAtExistingStop, context, routeState))
                    return RequestCost::INFTY_COST();
            }

            // Compute detour and added trip times for existing riders for every vehicle that is affected
            // through transfer dependencies
            detourComputer.computeDetours<true>(asgn, context);
            int minAddedTripTime = 0;
            int minTotalResidualDetours = 0;
            for (const auto &stopId: detourComputer.stopIdsSeen) {
                const int newArrTime = detourComputer.newArrTimes[stopId];
                const int newDepTime = detourComputer.newDepTimes[stopId];
                const int stopIdx = routeState.stopPositionOf(stopId);
                const int stopVehId = routeState.vehicleIdOf(stopId);
                const int schedArrTime = routeState.schedArrTimesFor(stopVehId)[stopIdx];
                if (newArrTime > schedArrTime) {
                    KASSERT(stopIdx > 0);
                    const auto numDropoffsPs = routeState.numDropoffsPrefixSumFor(stopVehId);
                    const int numDropoffs = numDropoffsPs[stopIdx] - numDropoffsPs[stopIdx - 1];
                    minAddedTripTime += (newArrTime - schedArrTime) * numDropoffs;
                }
                if (stopIdx == routeState.numStopsOf(stopVehId) - 1) {
                    const int schedDepTime = routeState.schedDepTimesFor(stopVehId)[stopIdx];
                    const int residualDetour = std::max(0, newDepTime - schedDepTime);
                    // For every vehicle whose last stop is reached later due to the assignment, check whether the
                    // service time constraint still holds.
                    if (checkHardConstraints &&
                        isServiceTimeConstraintViolated(fleet[stopVehId], context, residualDetour, routeState))
                        return RequestCost::INFTY_COST();

                    minTotalResidualDetours += residualDetour;
                }
            }

            // Count the detour for pickup and/or transfer after last stop of pVeh.
            if (asgn.transferIdxPVeh == numStopsPVeh - 1 && !transferPVehAtExistingStop) {
                minTotalResidualDetours += asgn.distToTransferPVeh + stopTime;
            }
            if (asgn.pickupIdx == numStopsPVeh - 1 && !pickupAtExistingStop) {
                minTotalResidualDetours += asgn.distToPickup + stopTime;
            }


            const int riderArrTimeAtTransfer = computeRiderArrTimeAtTransfer(asgn, actualDepTimeAtPickup,
                                                                             transferPVehAtExistingStop, detourComputer,
                                                                             routeState);
            const int depTimeAtTransfer = computeDVehDepTimeAtTransfer(asgn, riderArrTimeAtTransfer, detourComputer,
                                                                       routeState, context);

            // Check if service time constraint for pickup vehicle is violated for case that transfer is at last stop
            if (checkHardConstraints && asgn.transferIdxPVeh == numStopsPVeh - 1) {
                const int depTimeAtLastStop = std::max(routeState.schedDepTimesFor(pVehId)[numStopsPVeh - 1], context.originalRequest.requestTime);
                const auto residualDetour = riderArrTimeAtTransfer + (!transferPVehAtExistingStop * stopTime) -
                                            depTimeAtLastStop;
                if (isServiceTimeConstraintViolated(fleet[pVehId], context, residualDetour, routeState))
                    return RequestCost::INFTY_COST();
            }

            if constexpr (checkHardConstraints) {
                // Check rider hard constraints with detour after transfer in pVeh
                if (asgn.transferIdxPVeh < numStopsPVeh - 1) {
                    const int depTimeRightAfterTransferPVeh =
                            detourComputer.newDepTimes[routeState.stopIdsFor(pVehId)[asgn.transferIdxPVeh + 1]];
                    const int detourRightAfterTransferPVeh =
                            depTimeRightAfterTransferPVeh - routeState.schedDepTimesFor(pVehId)[
                                    asgn.transferIdxPVeh + 1];
                    if (detourRightAfterTransferPVeh > 0 &&
                        doesDropoffDetourViolateRiderHardConstraints(pVehId, asgn.transferIdxPVeh,
                                                                     detourRightAfterTransferPVeh, routeState))
                        return RequestCost::INFTY_COST();
                }

                // Check rider hard constraints with detour after transfer in dVeh
                if (asgn.transferIdxDVeh < numStopsDVeh - 1) {
                    const int depTimeRightAfterTransferDVeh =
                            detourComputer.newDepTimes[routeState.stopIdsFor(dVehId)[asgn.transferIdxDVeh + 1]];
                    const int detourRightAfterTransferDVeh =
                            depTimeRightAfterTransferDVeh - routeState.schedDepTimesFor(dVehId)[
                                    asgn.transferIdxDVeh + 1];
                    if (detourRightAfterTransferDVeh > 0 &&
                        doesPickupDetourViolateRiderHardConstraints(dVehId, asgn.transferIdxDVeh, INVALID_INDEX,
                                                                    detourRightAfterTransferDVeh, routeState))
                        return RequestCost::INFTY_COST();
                }
            }

            // Compute minimum trip time of new rider
            const int minTripTime = riderArrTimeAtTransfer - context.originalRequest.requestTime;
            const int waitTimePickup = actualDepTimeAtPickup - context.originalRequest.requestTime;

            // Apply cost function to time values
            RequestCost minCost;
            minCost.walkingCost = F::calcWalkingCost(asgn.pickup->walkingDist, InputConfig::getInstance().pickupRadius);
            minCost.tripCost = F::calcTripCost(minTripTime, context);
            minCost.waitTimeViolationCost = F::calcWaitViolationCost(riderArrTimeAtTransfer, depTimeAtTransfer,
                                                                  waitTimePickup, context);
            minCost.changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(minAddedTripTime);
            minCost.vehCost = F::calcVehicleCost(minTotalResidualDetours);
            minCost.total = minCost.vehCost + minCost.walkingCost + minCost.tripCost + minCost.waitTimeViolationCost +
                    minCost.changeInTripCostsOfOthers;
            return minCost;
        }



//
//        template<bool checkHardConstraints, typename RequestContext>
//        RequestCost calcBaseLowerBound(AssignmentWithTransfer &asgn, const RequestContext &context) const {
//            const bool unfinishedPVeh = asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed;
//
//            RequestCost costPVeh;
//            if (unfinishedPVeh) {
//                costPVeh = calcPartialCostForPVehLowerBound<checkHardConstraints>(asgn, context);
//            } else {
//                costPVeh = calcPartialCostForPVeh<checkHardConstraints>(asgn, context);
//            }
//
//            if (costPVeh.total >= INFTY) {
//                return RequestCost::INFTY_COST();
//            }
//
//            KASSERT(asgn.dropoff);
//            KASSERT(costPVeh.walkingCost >= 0 && costPVeh.tripCost >= 0 &&
//                   costPVeh.waitTimeViolationCost >= 0 && costPVeh.changeInTripCostsOfOthers >= 0 &&
//                   costPVeh.vehCost >= 0);
//
//            if (!asgn.dropoff) {
//                return RequestCost::INFTY_COST();
//            }
//
//            if (asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY || costPVeh.total >= INFTY) {
//                return RequestCost::INFTY_COST();
//            }
//
//            using namespace time_utils;
//
//            const int vehId = asgn.dVeh->vehicleId;
//            const auto numStops = routeState.numStopsOf(vehId);
//
//            const auto actualDepTimeAtTransfer = getActualDepTimeAtTransfer(asgn, context, routeState);
//            const auto initialTransferDetour = std::max(
//                    calcInitialTransferDetourDVeh(asgn, actualDepTimeAtTransfer, context, routeState), 0);
//
//            int addedTripTime = std::max(
//                    calcAddedTripTimeInInterval(vehId, asgn.transferIdxDVeh, asgn.dropoffIdx, initialTransferDetour,
//                                                routeState), 0);
//            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
//
//            const auto initialDropoffDetour = std::max(
//                    calcInitialDropoffDetour(asgn, dropoffAtExistingStop, routeState), 0);
//
//            const auto detourRightAfterDropoff = std::max(calcDetourRightAfterDropoff(asgn, initialTransferDetour,
//                                                                                      initialDropoffDetour, routeState),
//                                                          0);
//
//            const auto residualDetourAtEnd = std::max(calcResidualTotalDetourForStopAfterDropoff(asgn.dVeh->vehicleId,
//                                                                                                 asgn.dropoffIdx,
//                                                                                                 numStops - 1,
//                                                                                                 detourRightAfterDropoff,
//                                                                                                 routeState), 0);
//
//            if (checkHardConstraints &&
//                isAnyHardConstraintViolatedDVeh(asgn, context, initialTransferDetour, detourRightAfterDropoff,
//                                                residualDetourAtEnd, dropoffAtExistingStop, routeState)) {
//                return RequestCost::INFTY_COST();
//            }
//
//            addedTripTime += calcAddedTripTimeAffectedByTransferAndDropoff(asgn, detourRightAfterDropoff, routeState);
//
//            return calcFinalCost(asgn, costPVeh, context, initialTransferDetour, residualDetourAtEnd,
//                                 actualDepTimeAtTransfer,
//                                 dropoffAtExistingStop, addedTripTime);
//        }

        template<typename RequestContext>
        RequestCost calcMinCostForTransferPoint(const TransferPoint &tp, const RequestContext &context) const {
            using namespace time_utils;
            RequestCost cost = {0, 0, 0, 0, 0, 0};
            const auto pVehId = tp.pVeh->vehicleId;
            const auto numStopsPVeh = routeState.numStopsOf(pVehId);
            const bool transferAtExistingStopPVeh =
                    tp.loc == routeState.stopLocationsFor(pVehId)[tp.stopIdxPVeh];
            int initialDetourPVeh = 0;
            if (!transferAtExistingStopPVeh) {

                const auto legLengthPVeh = calcLengthOfLegStartingAt(tp.stopIdxPVeh, pVehId,
                                                                     routeState);
                initialDetourPVeh = tp.distancePVehToTransfer + InputConfig::getInstance().stopTime +
                                    tp.distancePVehFromTransfer - legLengthPVeh;
                KASSERT(tp.stopIdxPVeh == 0 || initialDetourPVeh >= 0);
                initialDetourPVeh = std::max(initialDetourPVeh, 0);
            }
            const auto residualDetourPVeh = calcResidualPickupDetour(pVehId, tp.stopIdxPVeh, numStopsPVeh,
                                                                     initialDetourPVeh, routeState);
            cost.vehCost += F::calcVehicleCost(residualDetourPVeh);
            const auto addedTripTimePVeh = calcAddedTripTimeInInterval(pVehId, tp.stopIdxPVeh,
                                                                       numStopsPVeh - 1, initialDetourPVeh, routeState);
            cost.changeInTripCostsOfOthers += F::calcChangeInTripCostsOfExistingPassengers(addedTripTimePVeh);

            const auto dVehId = tp.dVeh->vehicleId;
            const auto numStopsDVeh = routeState.numStopsOf(dVehId);
            const bool transferAtExistingStopDVeh =
                    tp.loc == routeState.stopLocationsFor(dVehId)[tp.stopIdxDVeh];
            int initialDetourDVeh = 0;
            if (!transferAtExistingStopDVeh) {
                const auto legLengthDVeh = calcLengthOfLegStartingAt(tp.stopIdxDVeh, dVehId,
                                                                     routeState);
                initialDetourDVeh = tp.distanceDVehToTransfer + InputConfig::getInstance().stopTime +
                                    tp.distanceDVehFromTransfer - legLengthDVeh;
                KASSERT(tp.stopIdxDVeh == 0 || initialDetourDVeh >= 0);
                initialDetourDVeh = std::max(initialDetourDVeh, 0);
            }
            const auto residualDetourDVeh = calcResidualPickupDetour(dVehId, tp.stopIdxDVeh,
                                                                     numStopsDVeh, initialDetourDVeh, routeState);
            cost.vehCost += F::calcVehicleCost(residualDetourDVeh);
            const auto addedTripTimeDVeh = calcAddedTripTimeInInterval(dVehId, tp.stopIdxDVeh,
                                                                       numStopsDVeh - 1, initialDetourDVeh, routeState);
            cost.changeInTripCostsOfOthers += F::calcChangeInTripCostsOfExistingPassengers(addedTripTimeDVeh);

            const auto minArrTimeAtTransferPVeh =
                    std::max(getVehDepTimeAtStopForRequest(pVehId, tp.stopIdxPVeh, context, routeState) +
                             tp.distancePVehToTransfer, context.originalRequest.requestTime);
            const auto minArrTimeAtTransferDVeh =
                    std::max(getVehDepTimeAtStopForRequest(dVehId, tp.stopIdxDVeh, context, routeState) +
                             tp.distanceDVehToTransfer, context.originalRequest.requestTime);
            const auto psgDepTimeAtTransfer =
                    std::max(minArrTimeAtTransferPVeh,
                             minArrTimeAtTransferDVeh +
                             !transferAtExistingStopDVeh * InputConfig::getInstance().stopTime);
            const auto minTripTime = psgDepTimeAtTransfer - context.originalRequest.requestTime;
            KASSERT(minTripTime >= 0);
            cost.tripCost += F::calcTripCost(minTripTime, context);

            cost.total = cost.walkingCost + cost.waitTimeViolationCost + cost.vehCost + cost.tripCost +
                         cost.changeInTripCostsOfOthers;
            return cost;
        }

//        template<bool checkHardConstraints, typename RequestContext>
//        RequestCost calcPartialCostForPVeh(AssignmentWithTransfer &asgn, const RequestContext &context) const {
//            using namespace time_utils;
//
//            KASSERT(asgn.pVeh && asgn.pickup);
//            if (!asgn.pVeh || !asgn.pickup) {
//                return RequestCost::INFTY_COST();
//            }
//
//            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
//                asgn.distToTransferPVeh == INFTY || asgn.distFromTransferPVeh == INFTY) {
//                return RequestCost::INFTY_COST();
//            }
//
//            const int vehId = asgn.pVeh->vehicleId;
//            const auto numStops = routeState.numStopsOf(vehId);
//            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, context, routeState);
//            asgn.depAtPickup = actualDepTimeAtPickup;
//            KASSERT(actualDepTimeAtPickup >= context.originalRequest.requestTime);
//            const auto initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, context, routeState);
//            KASSERT(initialPickupDetour >= 0);
//
//            int addedTripTime = calcAddedTripTimeInInterval(vehId, asgn.pickupIdx, asgn.transferIdxPVeh,
//                                                            initialPickupDetour, routeState);
//            bool transferAtExistingStop = isTransferAtExistingStopPVeh(asgn, routeState);
//            asgn.transferAtStopPVeh = transferAtExistingStop;
//
//            const auto initalTransferDetour = calcInitialTransferDetourPVeh(asgn, transferAtExistingStop, routeState);
//            KASSERT(asgn.pickupIdx == asgn.transferIdxPVeh || initalTransferDetour >= 0);
//
//            const auto detourRightAfterTransfer = calcDetourRightAfterTransferPVeh(asgn, initialPickupDetour,
//                                                                                   initalTransferDetour, routeState);
//            // The detourRightAfterTransfer is greater than or equal to zero by definition of the method
//
//            const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(asgn.pVeh->vehicleId,
//                                                                                        asgn.transferIdxPVeh,
//                                                                                        numStops - 1,
//                                                                                        detourRightAfterTransfer,
//                                                                                        routeState);
//            // The residualDetourAtEnd is greater than or equal to zero by definition of the method
//
//            if (checkHardConstraints && isAnyHardConstraintViolatedPVeh(asgn, context, initialPickupDetour,
//                                                                        detourRightAfterTransfer, residualDetourAtEnd,
//                                                                        transferAtExistingStop, routeState)) {
//                return RequestCost::INFTY_COST();
//            }
//
//            addedTripTime += calcAddedTripTimeAffectedByPickupAndTransfer(asgn, detourRightAfterTransfer, routeState);
//            KASSERT(addedTripTime >= 0);
//
//            return calcCostPVeh(asgn, context, initialPickupDetour, residualDetourAtEnd, actualDepTimeAtPickup,
//                                transferAtExistingStop, addedTripTime);
//        }
//
//        template<bool checkHardConstraints, typename RequestContext>
//        RequestCost
//        calcPartialCostForPVehLowerBound(AssignmentWithTransfer &asgn, const RequestContext &context) const {
//            using namespace time_utils;
//
//            KASSERT(asgn.pVeh && asgn.pickup);
//            if (!asgn.pVeh || !asgn.pickup) {
//                return RequestCost::INFTY_COST();
//            }
//
//            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY || asgn.distToTransferPVeh == INFTY ||
//                asgn.distFromTransferPVeh == INFTY) {
//                return RequestCost::INFTY_COST();
//            }
//
//
//            const int vehId = asgn.pVeh->vehicleId;
//            const auto numStops = routeState.numStopsOf(vehId);
//            const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, context, routeState);
//            const auto initialPickupDetour = std::max(
//                    calcInitialPickupDetour(asgn, actualDepTimeAtPickup, context, routeState), 0);
//            int addedTripTime = std::max(
//                    calcAddedTripTimeInInterval(vehId, asgn.pickupIdx, asgn.transferIdxPVeh, initialPickupDetour,
//                                                routeState), 0);
//
//            const bool transferAtExistingStop = isTransferAtExistingStopPVeh(asgn, routeState);
//
//            const auto initalTransferDetour = std::max(
//                    calcInitialTransferDetourPVeh(asgn, transferAtExistingStop, routeState), 0);
//
//            const auto detourRightAfterTransfer = calcDetourRightAfterTransferPVeh(asgn, initialPickupDetour,
//                                                                                   initalTransferDetour, routeState);
//            const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(asgn.pVeh->vehicleId,
//                                                                                        asgn.transferIdxPVeh,
//                                                                                        numStops - 1,
//                                                                                        detourRightAfterTransfer,
//                                                                                        routeState);
//
//            if (checkHardConstraints && isAnyHardConstraintViolatedPVeh(asgn, context, initialPickupDetour,
//                                                                        detourRightAfterTransfer, residualDetourAtEnd,
//                                                                        transferAtExistingStop, routeState)) {
//                return RequestCost::INFTY_COST();
//            }
//
//            addedTripTime += calcAddedTripTimeAffectedByPickupAndTransfer(asgn, detourRightAfterTransfer, routeState);
//
//            return calcCostPVeh(asgn, context, initialPickupDetour, residualDetourAtEnd, actualDepTimeAtPickup,
//                                transferAtExistingStop, addedTripTime);
//        }

        // Calculate the cost for a passenger moving to their destination independently without using a vehicle.
        template<typename RequestContext>
        static int calcCostForNotUsingVehicle(const int walkingDist, const int travelTimeOfDestEdge,
                                              const RequestContext &context) {
            KASSERT(walkingDist >= travelTimeOfDestEdge);

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
            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY ||
                asgn.distToDropoff == INFTY || asgn.distFromDropoff == INFTY)
                return INFTY;

            KASSERT(asgn.pickupStopIdx == asgn.dropoffStopIdx);
            const auto stopIdx = asgn.pickupStopIdx;
            const auto vehId = asgn.vehicle->vehicleId;

            const int minDetour = asgn.distToPickup + asgn.distToDropoff + asgn.distFromDropoff -
                                  calcLengthOfLegStartingAt(stopIdx, vehId, routeState);
            if (doesDropoffDetourViolateHardConstraints(*asgn.vehicle, context, stopIdx, minDetour, routeState))
                return INFTY;

            const int tripTimeLowerBound = asgn.distToDropoff;
            const auto tripCostLowerBound = F::calcTripCost(tripTimeLowerBound, context);

            return F::calcVehicleCost(minDetour) + tripCostLowerBound;

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
                                                                          routeState);

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
            const int walkingCost = F::calcWalkingCost(pickup.walkingDist, InputConfig::getInstance().pickupRadius);
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
                const typename LabelSet::DistanceLabel &detourTillDepAtPickup,
                const typename LabelSet::DistanceLabel &tripTimeTillDepAtPickup,
                const typename LabelSet::DistanceLabel &directDist,
                const typename LabelSet::DistanceLabel &pickupWalkingDists,
                const RequestContext &context) const {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;
            KASSERT(directDist.horizontalMin() >= 0 && directDist.horizontalMax() < INFTY);
            KASSERT(pickupWalkingDists.horizontalMin() >= 0 && pickupWalkingDists.horizontalMax() < INFTY);


            // Calculations with INFTY don't work like mathematical infinity, so set cost to INFTY later.
            const LabelMask inftyMask = ~((detourTillDepAtPickup < INFTY) & (tripTimeTillDepAtPickup < INFTY));
            // const DistanceLabel adaptedVehTimeTillDepAtPickup = select(vehTimeInftyMask, 0, vehTimeTillDepAtPickup);

            const DistanceLabel detourCost = F::calcKVehicleCosts(detourTillDepAtPickup + directDist + stopTime);
            const DistanceLabel tripCost = F::calcKTripCosts(tripTimeTillDepAtPickup + directDist, context);
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists,
                                                                   InputConfig::getInstance().pickupRadius);
            const DistanceLabel waitViolationCost = F::calcKWaitViolationCosts(
                    context.originalRequest.requestTime + tripTimeTillDepAtPickup, context);
            // Pickup after last stop so no added trip costs for existing passengers.
            DistanceLabel minCost = detourCost + tripCost + walkingCost + waitViolationCost;

            // Set cost to INFTY where input times were invalid
            minCost.setIf(DistanceLabel(INFTY), inftyMask);

            return minCost;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcLowerBoundCostForKPairedPickupAndTransferAssignmentsAfterLastStop(
                const typename LabelSet::DistanceLabel &detourTillDepAtPickup,
                const typename LabelSet::DistanceLabel &tripTimeTillDepAtPickup,
                const typename LabelSet::DistanceLabel &minPickupToDropoffDist,
                const typename LabelSet::DistanceLabel &pickupWalkingDists,
                const RequestContext &context) const {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;
            KASSERT(detourTillDepAtPickup.horizontalMin() >= 0 && detourTillDepAtPickup.horizontalMax() < INFTY);
            KASSERT(tripTimeTillDepAtPickup.horizontalMin() >= 0 && tripTimeTillDepAtPickup.horizontalMax() < INFTY);
            KASSERT(pickupWalkingDists.horizontalMin() >= 0 && pickupWalkingDists.horizontalMax() < INFTY);


            // Calculations with INFTY don't work like mathematical infinity, so set cost to INFTY later.
            const LabelMask inftyMask = ~((detourTillDepAtPickup < INFTY) & (tripTimeTillDepAtPickup < INFTY));
            // const DistanceLabel adaptedVehTimeTillDepAtPickup = select(vehTimeInftyMask, 0, vehTimeTillDepAtPickup);

            const DistanceLabel detourCost = F::calcKVehicleCosts(detourTillDepAtPickup + stopTime);
            const DistanceLabel tripCost = F::calcKTripCosts(tripTimeTillDepAtPickup + minPickupToDropoffDist, context);
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists,
                                                                   InputConfig::getInstance().pickupRadius);
            const DistanceLabel waitViolationCost = F::calcKWaitViolationCosts(
                    context.originalRequest.requestTime + tripTimeTillDepAtPickup, context);
            // Pickup after last stop so no added trip costs for existing passengers.
            DistanceLabel minCost = detourCost + tripCost + walkingCost + waitViolationCost;

            // Set cost to INFTY where input times were invalid
            minCost.setIf(DistanceLabel(INFTY), inftyMask);

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
            KASSERT(psgArrTimesAtPickups.horizontalMin() >= 0 && psgArrTimesAtPickups.horizontalMax() < INFTY);
            KASSERT(distancesToDest.horizontalMin() >= 0 && distancesToDest.horizontalMax() < INFTY);
            KASSERT(pickupWalkingDists.horizontalMin() >= 0 && pickupWalkingDists.horizontalMax() < INFTY);

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
            const DistanceLabel walkingCost = F::calcKWalkingCosts(pickupWalkingDists,
                                                                   InputConfig::getInstance().pickupRadius);
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
                                                                       context, routeState);
            const int vehDepTimeAtPrevStop = std::max(routeState.schedDepTimesFor(vehId)[numStops - 1],
                                                      context.originalRequest.requestTime);
            const int detourUntilDepAtPickup = actualDepTimeAtPickup - vehDepTimeAtPrevStop;
            KASSERT(!((bool) (detourUntilDepAtPickup < 0)));
            const int minDetour = detourUntilDepAtPickup + minDistToDropoff;

            if (time_utils::isServiceTimeConstraintViolated(veh, context, minDetour, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(pickup.walkingDist, InputConfig::getInstance().pickupRadius);
            const int waitViolationCost = F::calcWaitViolationCost(actualDepTimeAtPickup, context);
            const int waitTimeIncludingWalking = actualDepTimeAtPickup - context.originalRequest.requestTime;
            KASSERT(waitTimeIncludingWalking >= 0);
            const int minTripTime = waitTimeIncludingWalking + context.minDirectPDDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);

            return F::calcVehicleCost(minDetour) + walkingCost + waitViolationCost + minTripCost;
        }

        template<typename RequestContext>
        int calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(const int distToDropoff,
                                                                                 const PDLoc &dropoff,
                                                                                 const RequestContext &context) const {
            return calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(dropoff.walkingDist,
                                                                                        distToDropoff,
                                                                                        stopTime, context);
        }

        template<typename RequestContext>
        int calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(const int dropoffWalkingDist,
                                                                                 const int distToDropoff,
                                                                                 const int minTripTimeToLastStop,
                                                                                 const RequestContext &context) const {
            KASSERT(distToDropoff < INFTY);
            if (distToDropoff >= INFTY)
                return INFTY;

            const int minDetour = distToDropoff + (distToDropoff != 0) * stopTime;

            const int walkingCost = F::calcWalkingCost(dropoffWalkingDist, InputConfig::getInstance().dropoffRadius);
            const int minTripTime = minTripTimeToLastStop + distToDropoff + dropoffWalkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);

            // Independent of vehicle so we cannot know here which existing passengers may be affected
            // const int minAddedTripCostOfOthers = 0;

            return F::calcVehicleCost(minDetour) + walkingCost + minTripCost;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinDistToDropoff(
                const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                const typename LabelSet::DistanceLabel &distToDropoff,
                const typename LabelSet::DistanceLabel &minTripTimeToLastStop,
                const RequestContext &context) const {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, set cost to INFTY later.
            const LabelMask inftyMask = ~(distToDropoff < INFTY);

            const DistanceLabel minDropoffDetours = distToDropoff + stopTime;
            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists,
                                                                    InputConfig::getInstance().dropoffRadius);
            DistanceLabel minTripTimes =
                    minTripTimeToLastStop + distToDropoff + dropoffWalkingDists;
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, context);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;
            costLowerBound.setIf(DistanceLabel(INFTY), inftyMask);

            return costLowerBound;
        }

        template<typename RequestContext>
        int calcVehicleIndependentCostLowerBoundForDALSWithKnownMinArrTime(const int dropoffWalkingDist,
                                                                           const int minDistToDropoff,
                                                                           const int minArrTimeAtDropoff,
                                                                           const RequestContext &context) const {
            KASSERT(minArrTimeAtDropoff >= context.originalRequest.requestTime);
            KASSERT(minDistToDropoff < INFTY);
            if (minDistToDropoff >= INFTY)
                return INFTY;

            const int minDetour = minDistToDropoff + stopTime;
            const int walkingCost = F::calcWalkingCost(dropoffWalkingDist, InputConfig::getInstance().dropoffRadius);
            const int minTripTime = minArrTimeAtDropoff - context.originalRequest.requestTime + dropoffWalkingDist;
            const int minTripCost = F::calcTripCost(minTripTime, context);

            return F::calcVehicleCost(minDetour) + walkingCost + minTripCost;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinArrTime(
                const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                const typename LabelSet::DistanceLabel &minDistToDropoff,
                const typename LabelSet::DistanceLabel &minArrTimeAtDropoff,
                const RequestContext &context) const {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, set cost to INFTY later.
            const LabelMask inftyMask = ~((minDistToDropoff < INFTY) & (minArrTimeAtDropoff < INFTY));

            const DistanceLabel minDropoffDetours = minDistToDropoff + stopTime;
            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists,
                                                                    InputConfig::getInstance().dropoffRadius);
            DistanceLabel minTripTimes =
                    minArrTimeAtDropoff + dropoffWalkingDists - DistanceLabel(context.originalRequest.requestTime);
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, context);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;
            costLowerBound.setIf(DistanceLabel(INFTY), inftyMask);

            return costLowerBound;
        }

        template<typename LabelSet, typename RequestContext>
        typename LabelSet::DistanceLabel
        calcKVehicleDependentCostLowerBoundsForDALSWithKnownDistToDropoff(
                const int vehId,
                const typename LabelSet::DistanceLabel &dropoffWalkingDists,
                const typename LabelSet::DistanceLabel &distToDropoff,
                const typename LabelSet::DistanceLabel &minTripTimeToLastStop,
                const RequestContext &context) const {
            using DistanceLabel = typename LabelSet::DistanceLabel;
            using LabelMask = typename LabelSet::LabelMask;

            // For dropoffs with a distanceToDropoff of INFTY, set cost to INFTY later.
            const LabelMask inftyMask = ~(distToDropoff < INFTY);

            const DistanceLabel minDropoffDetours = distToDropoff + stopTime;
            const DistanceLabel walkingCosts = F::calcKWalkingCosts(dropoffWalkingDists,
                                                                    InputConfig::getInstance().dropoffRadius);
            const auto depTimeAtLastStop = routeState.schedDepTimesFor(vehId)[routeState.numStopsOf(vehId) - 1];
            DistanceLabel minTripTimes =
                    minTripTimeToLastStop + DistanceLabel(depTimeAtLastStop - context.originalRequest.requestTime) +
                    distToDropoff + dropoffWalkingDists;
            const DistanceLabel minTripCosts = F::calcKTripCosts(minTripTimes, context);

            // Independent of pickup so we cannot know here which existing passengers may be affected by pickup detour
            // const DistanceLabel minAddedTripCostOfOthers = 0;

            DistanceLabel costLowerBound = F::calcKVehicleCosts(minDropoffDetours) + walkingCosts + minTripCosts;
            costLowerBound.setIf(DistanceLabel(INFTY), inftyMask);

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
            const int walkingCost = F::calcWalkingCost(pickupWalkingDist, InputConfig::getInstance().pickupRadius) +
                                    F::calcWalkingCost(dropoffWalkingDist, InputConfig::getInstance().dropoffRadius);
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

            const int walkingCost = F::calcWalkingCost(walkingDist, InputConfig::getInstance().pickupRadius);
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
        // pickup is used or where the pickup will be inserted. Does not consider any bounds on trip time.
        template<typename RequestContext>
        int calcMinKnownDropoffSideCostWithoutTripTime(const Vehicle &veh, const int dropoffIndex,
                                                       const int initialDropoffDetour, const int walkingDist,
                                                       const RequestContext &context) const {
            using namespace time_utils;

            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto residualDetourAtEnd = calcResidualTotalDetour(veh.vehicleId, dropoffIndex, dropoffIndex,
                                                                     numStops - 1, initialDropoffDetour,
                                                                     initialDropoffDetour, routeState);

            if (isServiceTimeConstraintViolated(veh, context, residualDetourAtEnd, routeState))
                return INFTY;

            const int walkingCost = F::calcWalkingCost(walkingDist, InputConfig::getInstance().dropoffRadius);
            const int minAddedTripTimeOfOthers = calcAddedTripTimeInInterval(veh.vehicleId, dropoffIndex, numStops - 1,
                                                                             initialDropoffDetour, routeState);
            const int minChangeInTripTimeCosts = F::calcChangeInTripCostsOfExistingPassengers(minAddedTripTimeOfOthers);

            return F::calcVehicleCost(residualDetourAtEnd) + walkingCost + minChangeInTripTimeCosts;
        }

        // Calculates the cost for the dropoff side of an assignment that can be known without knowing which
        // pickup is used or where the pickup will be inserted.
        template<typename RequestContext>
        int calcMinKnownDropoffSideCost(const Vehicle &veh, const int dropoffIndex,
                                        const int initialDropoffDetour, const int walkingDist,
                                        const RequestContext &context) const {
            return calcMinKnownDropoffSideCostWithoutTripTime(veh, dropoffIndex, initialDropoffDetour, walkingDist,
                                                              context) +
                   F::calcTripCost(context.minDirectPDDist + walkingDist, context);
        }

        template<typename RequestContext>
        inline bool
        isDropoffCostPromisingForAfterLastStop(const PDLoc &dropoff, const RequestContext &context) const {
            const auto walkMinCost =
                    F::calcTripCost(dropoff.walkingDist, context) +
                    F::calcWalkingCost(dropoff.walkingDist, InputConfig::getInstance().dropoffRadius);
            const auto vehMinCost =
                    F::calcVehicleCost(dropoff.vehDistToCenter) + F::calcTripCost(dropoff.vehDistToCenter, context);
            return walkMinCost <= vehMinCost;
        }


//        template<typename RequestContext>
//        int calcCostLowerBoundForOrdinaryDropoffWithPickupIndex(const int vehId,
//                                                                const PDLoc &dropoff,
//                                                                const int pickupIdx,
//                                                                const int dropoffIdx,
//                                                                const int distToDropoff,
//                                                                const int distFromDropoff,
//                                                                const RequestContext &context) const {
//            using namespace time_utils;
//            const auto numStops = routeState.numStopsOf(vehId);
//            KASSERT(distToDropoff < INFTY);
//            KASSERT(pickupIdx <= dropoffIdx && dropoffIdx < routeState.numStopsOf(vehId) - 1);
//
//            // Compute lower bound for costs with transfer between i and i + 1, considering trip time
//            // from stop i + 1 to the dropoff and detour to the dropoff. This lower bound also holds for
//            // any stop before i so if it is worse than the best known cost, we can stop for this vehicle.
//            const auto minTripTimeToStopBeforeDropoff = pickupIdx == dropoffIdx ? 0 :
//                                                        routeState.schedArrTimesFor(vehId)[dropoffIdx] -
//                                                        routeState.schedDepTimesFor(vehId)[pickupIdx + 1];
//            const int minTripTime = minTripTimeToStopBeforeDropoff + distToDropoff + dropoff.walkingDist;
//
//
//            const auto dropoffAtExistingStop = isDropoffAtExistingStop(vehId, pickupIdx, dropoffIdx, dropoff.loc,
//                                                                       routeState);
//            KASSERT(!dropoffAtExistingStop || distToDropoff == 0);
//            const int initialDropoffDetour = calcInitialDropoffDetour(vehId, dropoffIdx, distToDropoff, distFromDropoff,
//                                                                      dropoffAtExistingStop, routeState);
//            const int minResidualDetour = calcResidualTotalDetour(vehId, pickupIdx, dropoffIdx, numStops - 1, 0,
//                                                                  initialDropoffDetour, routeState);
//
//            const int minAddedTripTime = calcAddedTripTimeInInterval(vehId, dropoffIdx, numStops - 1,
//                                                                     initialDropoffDetour, routeState);
//
//            const int walkingCost = F::calcWalkingCost(dropoff.walkingDist, InputConfig::getInstance().dropoffRadius);
//            const int minTripCost = F::calcTripCost(minTripTime, context);
//            const int minChangeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(minAddedTripTime);
//            const int minDetourCost = F::calcVehicleCost(minResidualDetour);
//
//            return walkingCost + minTripCost + minChangeInTripCostsOfOthers + minDetourCost;
//        }

    private:

//        template<typename RequestContext>
//        RequestCost calcCostPVeh(AssignmentWithTransfer &asgn,
//                                 const RequestContext &context, const int initialPickupDetour,
//                                 const int residualDetourAtEnd, const int depTimeAtPickup,
//                                 const bool transferAtExistingStop,
//                                 const int addedTripTimeForExistingPassengers) const {
//
//            if (!asgn.pVeh || !asgn.pickup) {
//                return RequestCost::INFTY_COST();
//            }
//
//            using namespace time_utils;
//
//            const auto arrTimeAtTransfer = getArrTimeAtTransfer(depTimeAtPickup, asgn, initialPickupDetour,
//                                                                transferAtExistingStop, routeState);
//            asgn.arrAtTransferPoint = arrTimeAtTransfer;
//            KASSERT(asgn.pickupPairedLowerBoundUsed || asgn.pickupBNSLowerBoundUsed ||
//                   asgn.arrAtTransferPoint > asgn.depAtPickup);
//
//            const int pickupIdx = asgn.pickupIdx;
//            const bool pickupAtExistingStop = isPickupAtExistingStop(*asgn.pickup, asgn.pVeh->vehicleId,
//                                                                     context.originalRequest.requestTime, pickupIdx,
//                                                                     routeState);
//            (void) pickupAtExistingStop;
//
//            KASSERT(asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed ||
//                   asgn.pickupIdx != asgn.transferIdxPVeh || transferAtExistingStop ||
//                   asgn.arrAtTransferPoint != asgn.depAtPickup);
//            const int tripTime = arrTimeAtTransfer - context.originalRequest.requestTime;
//
//            const auto walkingCostPVeh = F::calcWalkingCost(asgn.pickup->walkingDist,
//                                                            InputConfig::getInstance().pickupRadius);
//            const auto tripCostPVeh = F::calcTripCost(tripTime, context);
//            asgn.waitTimeAtPickup = depTimeAtPickup - context.originalRequest.requestTime;
//            const auto waitTimeViolationCost = F::calcWaitViolationCost(depTimeAtPickup, context);
//            KASSERT(addedTripTimeForExistingPassengers >= 0);
//            const auto changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(
//                    addedTripTimeForExistingPassengers);
//            KASSERT(changeInTripCostsOfOthers >= 0);
//            const auto vehCostPVeh = F::calcVehicleCost(residualDetourAtEnd);
//
//            RequestCost costPVeh;
//            costPVeh.walkingCost = walkingCostPVeh;
//            costPVeh.tripCost = tripCostPVeh;
//            costPVeh.waitTimeViolationCost = waitTimeViolationCost;
//            costPVeh.changeInTripCostsOfOthers = changeInTripCostsOfOthers;
//            costPVeh.vehCost = vehCostPVeh;
//
//            KASSERT(walkingCostPVeh >= 0 && tripCostPVeh >= 0 && waitTimeViolationCost >= 0 &&
//                   changeInTripCostsOfOthers >= 0 && vehCostPVeh >= 0);
//
//            const int total =
//                    vehCostPVeh + walkingCostPVeh + tripCostPVeh + waitTimeViolationCost + changeInTripCostsOfOthers;
//            KASSERT(total >= 0);
//
//            costPVeh.total = total;
//
//            KASSERT(costPVeh.total >= 0);
//            return costPVeh;
//        }
//
//        template<typename RequestContext>
//        RequestCost
//        calcFinalCost(AssignmentWithTransfer &asgn, const RequestCost &costPVeh, const RequestContext &context,
//                      const int initialTransferDetour,
//                      const int residualDetourAtEnd, const int actualDepTimeAtTransfer,
//                      const bool dropoffAtExistingStop, const int addedTripTime) const {
//            if (!asgn.dVeh || !asgn.pVeh || !asgn.pickup || !asgn.dropoff) {
//                return RequestCost::INFTY_COST();
//            }
//
//            using namespace time_utils;
//
//            const int arrTimeAtDropoff = getArrTimeAtDropoff(actualDepTimeAtTransfer, asgn, initialTransferDetour,
//                                                             dropoffAtExistingStop, routeState);
//            const int tripTime = arrTimeAtDropoff - asgn.arrAtTransferPoint + asgn.dropoff->walkingDist;
//            asgn.tripTimeDVeh = tripTime;
//            asgn.arrAtDropoff = arrTimeAtDropoff;
//            asgn.requestTime = context.originalRequest.requestTime;
//
//            const int walkingCost = F::calcWalkingCost(asgn.dropoff->walkingDist,
//                                                       InputConfig::getInstance().dropoffRadius);
//            const int tripCost = F::calcTripCost(tripTime, context);
//            const int waitTimeViolationCost = F::calcWaitViolationCost(asgn.arrAtTransferPoint, actualDepTimeAtTransfer,
//                                                                       asgn.waitTimeAtPickup, context);
//            const int changeInTripCostsOfOthers = F::calcChangeInTripCostsOfExistingPassengers(addedTripTime);
//            const int vehCost = F::calcVehicleCost(residualDetourAtEnd);
//
//            if (walkingCost >= INFTY || tripCost >= INFTY || waitTimeViolationCost >= INFTY ||
//                changeInTripCostsOfOthers >= INFTY || vehCost >= INFTY) {
//                return RequestCost::INFTY_COST();
//            }
//
//            RequestCost cost;
//            cost.walkingCost = costPVeh.walkingCost + walkingCost;
//            cost.tripCost = costPVeh.tripCost + tripCost;
//            cost.waitTimeViolationCost = costPVeh.waitTimeViolationCost + waitTimeViolationCost;
//            cost.changeInTripCostsOfOthers = costPVeh.changeInTripCostsOfOthers + changeInTripCostsOfOthers;
//            cost.vehCost = costPVeh.vehCost + vehCost;
//            KASSERT(cost.walkingCost >= 0 && cost.tripCost >= 0 && cost.waitTimeViolationCost >= 0 &&
//                   cost.changeInTripCostsOfOthers >= 0 && cost.vehCost >= 0);
//
//            if (cost.walkingCost >= INFTY || cost.tripCost >= INFTY ||
//                cost.waitTimeViolationCost >= INFTY || cost.changeInTripCostsOfOthers >= INFTY ||
//                cost.vehCost >= INFTY) {
//                return RequestCost::INFTY_COST();
//            }
//
//            int total = (costPVeh.total + vehCost + walkingCost + tripCost + waitTimeViolationCost +
//                         changeInTripCostsOfOthers);
//            KASSERT(total >= 0);
//            cost.total = total;
//            return cost;
//        }


        const RouteState &routeState;
        const Fleet &fleet;
        const int stopTime;

        time_utils::DetourComputer detourComputer;
    };


    static constexpr int PSG_COST_SCALE = KARRI_PSG_COST_SCALE; // CMake compile time parameter
    static constexpr int VEH_COST_SCALE = KARRI_VEH_COST_SCALE; // CMake compile time parameter
    static constexpr int WALKING_COST_SCALE = KARRI_WALKING_COST_SCALE; // CMake compile time parameter
    static constexpr int WAIT_PENALTY_SCALE = KARRI_WAIT_PENALTY_SCALE; // CMake compile time parameter
    static constexpr int TRIP_PENALTY_SCALE = KARRI_TRIP_PENALTY_SCALE; // CMake compile time parameter

    using CostCalculator = CostCalculatorTemplate<TimeIsMoneyCostFunction<PSG_COST_SCALE, WALKING_COST_SCALE, VEH_COST_SCALE, WAIT_PENALTY_SCALE, TRIP_PENALTY_SCALE>>;
}