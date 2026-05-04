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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/TransferPoints/TransferPoint.h"
#include "cassert"
#include "DataStructures/Containers/TimestampedVector.h"

#define DO_INLINE true
#if DO_INLINE
#define INLINE inline
#else
#define INLINE
#endif

namespace karri::time_utils {

    template<typename RequestContext>
    static INLINE int getVehDepTimeAtStopForRequest(const int vehId, const int stopIdx, const RequestContext &context,
                                                    const RouteState &routeState) {
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        return (numStops == 1 ? std::max(minDepTimes[0], context.originalRequest.requestTime) : minDepTimes[stopIdx]);
    }

    static INLINE bool isMakingStop(const int vehId, const int now, const RouteState &routeState) {
        return routeState.schedDepTimesFor(vehId)[0] > now;
    }

    static INLINE bool isPickupAtExistingStop(const PDLoc &pickup, const int vehId, const int now, const int stopIndex,
                                              const RouteState &routeState) {
        return (stopIndex > 0 || isMakingStop(vehId, now, routeState)) &&
               pickup.loc == routeState.stopLocationsFor(vehId)[stopIndex];
    }

    static INLINE bool isTransferAtExistingStopPVeh(const AssignmentWithTransfer &asgn, const RouteState &routeState) {
        return asgn.pickupIdx != asgn.transferIdxPVeh &&
               asgn.transfer.loc == routeState.stopLocationsFor(asgn.pVeh->vehicleId)[asgn.transferIdxPVeh];
    }

    static INLINE bool
    isTransferAtExistingStopDVeh(const AssignmentWithTransfer &asgn, const int now, const RouteState &routeState) {
        return (asgn.transferIdxDVeh > 0 || isMakingStop(asgn.dVeh->vehicleId, now, routeState) ||
                routeState.numStopsOf(asgn.dVeh->vehicleId) == 1) &&
               asgn.transfer.loc == routeState.stopLocationsFor(asgn.dVeh->vehicleId)[asgn.transferIdxDVeh];
    }

    template<typename LabelSet>
    static INLINE typename LabelSet::LabelMask
    isPickupAtExistingStop(const typename LabelSet::DistanceLabel &pickupLocs, const int vehId, const int now,
                           const int stopIndex, const RouteState &routeState) {
        if (!(stopIndex > 0 || isMakingStop(vehId, now, routeState))) return false;

        return pickupLocs == typename LabelSet::DistanceLabel(routeState.stopLocationsFor(vehId)[stopIndex]);
    }

    template<typename LabelSet>
    static INLINE typename LabelSet::LabelMask
    isPickupAtExistingStop(const std::array<PDLoc, LabelSet::K> &pickups, const int vehId, const int now,
                           const int stopIndex, const RouteState &routeState) {
        typename LabelSet::DistanceLabel pickupLocations;
        for (int i = 0; i < LabelSet::K; ++i) {
            pickupLocations[i] = pickups[i].location;
        }
        return isPickupAtExistingStop(pickupLocations, vehId, now, stopIndex, routeState);
    }

    template<typename RequestContext>
    static INLINE int getActualDepTimeAtPickup(const int vehId, const int stopIndexBeforePickup, const int distToPickup,
                                               const PDLoc &pickup, const RequestContext &context,
                                               const RouteState &routeState) {
        const bool atStop = isPickupAtExistingStop(pickup, vehId, context.now(), stopIndexBeforePickup, routeState);
        const auto minVehicleDepTimeAtPickup =
                getVehDepTimeAtStopForRequest(vehId, stopIndexBeforePickup, context, routeState) +
                !atStop * (distToPickup + InputConfig::getInstance().stopTime);

        // We assume a pickup at an existing stop takes no additional counting of stopTime, irrespective of when the
        // passenger arrives there. The vehicle can depart as soon as both the vehicle and the passenger are at the
        // location. This is how LOUD originally did it, so we adhere to it.
        return std::max(minVehicleDepTimeAtPickup, context.getPassengerArrAtPickup(pickup.id));
    }

//    template<typename RequestContext>
//    static INLINE int getActualDepTimeAtTransfer(const AssignmentWithTransfer &asgn, const RequestContext &context,
//                                                 const RouteState &routeState) {
//        const bool atStop = isTransferAtExistingStopDVeh(asgn, context.originalRequest.requestTime, routeState);
//
//        const auto minVehicleDepTimeAtTransfer =
//                getVehDepTimeAtStopForRequest(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, context, routeState) +
//                !atStop * (asgn.distToTransferDVeh + InputConfig::getInstance().stopTime);
//
//        return std::max(minVehicleDepTimeAtTransfer, asgn.arrAtTransferPoint);
//    }

    template<typename RequestContext>
    static INLINE int
    getActualDepTimeAtPickup(const Assignment &asgn, const RequestContext &context, const RouteState &routeState) {
        return getActualDepTimeAtPickup(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.distToPickup, *asgn.pickup,
                                        context, routeState);
    }

    template<typename RequestContext>
    static INLINE int getActualDepTimeAtPickup(const AssignmentWithTransfer &asgn, const RequestContext &context,
                                               const RouteState &routeState) {
        return getActualDepTimeAtPickup(asgn.pVeh->vehicleId, asgn.pickupIdx, asgn.distToPickup, *asgn.pickup, context,
                                        routeState);
    }

    template<typename LabelSet, typename RequestContext>
    static INLINE typename LabelSet::DistanceLabel
    getActualDepTimeAtPickup(const int vehId, const int stopIndexBeforePickup,
                             const typename LabelSet::DistanceLabel &distToPickup,
                             const typename LabelSet::DistanceLabel &pickupLocs,
                             const typename LabelSet::DistanceLabel &passengerArrTimesAtPickups,
                             const RequestContext &context,
                             const RouteState &routeState) {
        using LabelMask = typename LabelSet::LabelMask;

        const LabelMask atStop = isPickupAtExistingStop<LabelSet>(pickupLocs, vehId, context.now(),
                                                                  stopIndexBeforePickup, routeState);

        // We assume a pickup at an existing stop takes no additional counting of stopTime, irrespective of when the
        // passenger arrives there. The vehicle can depart as soon as both the vehicle and the passenger are at the
        // location. This is how LOUD originally did it, so we adhere to it.
        auto minDepTimeAtPickup = getVehDepTimeAtStopForRequest(vehId, stopIndexBeforePickup, context, routeState) +
                                  select(atStop, 0, distToPickup + InputConfig::getInstance().stopTime);
        minDepTimeAtPickup.max(passengerArrTimesAtPickups);
        return minDepTimeAtPickup;
    }

    static INLINE int
    calcLengthOfLegStartingAt(const int stopIndex, const int vehicleId, const RouteState &routeState) {
        if (stopIndex + 1 == routeState.numStopsOf(vehicleId))
            return 0;
        const auto &minDepTimes = routeState.schedDepTimesFor(vehicleId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehicleId);
        return minArrTimes[stopIndex + 1] - minDepTimes[stopIndex];
    }

    static INLINE bool isDropoffAtExistingStop(const int vehId, const int pickupIdx, const int dropoffIdx,
                                               const int dropoffLoc, const RouteState &routeState) {
        return pickupIdx != dropoffIdx && dropoffLoc == routeState.stopLocationsFor(vehId)[dropoffIdx];
    }

    static INLINE bool isDropoffAtExistingStop(const Assignment &asgn, const RouteState &routeState) {
        return isDropoffAtExistingStop(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                       asgn.dropoff->loc, routeState);
    }

    static INLINE bool isDropoffAtExistingStop(const AssignmentWithTransfer &asgn, const RouteState &routeState) {
        KASSERT(asgn.dropoffIdx < routeState.numStopsOf(asgn.dVeh->vehicleId));
        return isDropoffAtExistingStop(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, asgn.dropoffIdx,
                                       asgn.dropoff->loc, routeState);
    }

    static INLINE int
    getArrTimeAtDropoff(const int actualDepTimeAtPickup, const Assignment &asgn, const int initialPickupDetour,
                        const bool dropoffAtExistingStop, const RouteState &routeState) {
        const auto pickupIndex = asgn.pickupStopIdx;
        const auto dropoffIndex = asgn.dropoffStopIdx;
        const auto &minDepTimes = routeState.schedDepTimesFor(asgn.vehicle->vehicleId);
        const auto &minArrTimes = routeState.schedArrTimesFor(asgn.vehicle->vehicleId);
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(asgn.vehicle->vehicleId);

        if (pickupIndex == dropoffIndex) {
            return actualDepTimeAtPickup + asgn.distToDropoff;
        }

        KASSERT(dropoffIndex > 0);
        const auto stopLengthsBetween = vehWaitTimesPrefixSum[dropoffIndex - 1] - vehWaitTimesPrefixSum[pickupIndex];
        const auto arrTimeAtPrevious =
                minArrTimes[dropoffIndex] + std::max(initialPickupDetour - stopLengthsBetween, 0);

        if (dropoffAtExistingStop) {
            return arrTimeAtPrevious;
        }

        const auto depTimeAtPrevious = std::max(minDepTimes[dropoffIndex],
                                                arrTimeAtPrevious + InputConfig::getInstance().stopTime);
        return depTimeAtPrevious + asgn.distToDropoff;
    }

    static INLINE int
    getArrTimeAtDropoff(const int actualDepTimeAtTransfer, const AssignmentWithTransfer &asgn,
                        const int initialTransferDetour, const bool dropoffAtExistingStop,
                        const RouteState &routeState) {
        const auto vehIdDVeh = asgn.dVeh->vehicleId;
        const auto pickupIndex = asgn.transferIdxDVeh;
        const auto dropoffIndex = asgn.dropoffIdx;
        const auto &minDepTimes = routeState.schedDepTimesFor(vehIdDVeh);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehIdDVeh);
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(vehIdDVeh);

        KASSERT(pickupIndex < routeState.numStopsOf(vehIdDVeh) && dropoffIndex < routeState.numStopsOf(vehIdDVeh));

        if (pickupIndex == dropoffIndex) {
            return actualDepTimeAtTransfer + asgn.distToDropoff;
        }

        KASSERT(dropoffIndex > 0 && pickupIndex >= 0);
        const auto stopLengthsBetween = vehWaitTimesPrefixSum[dropoffIndex - 1] - vehWaitTimesPrefixSum[pickupIndex];
        const auto arrTimeAtPrevious =
                minArrTimes[dropoffIndex] + std::max(initialTransferDetour - stopLengthsBetween, 0);

        if (dropoffAtExistingStop) {
            return arrTimeAtPrevious;
        }

        const auto depTimeAtPrevious = std::max(minDepTimes[dropoffIndex],
                                                arrTimeAtPrevious + InputConfig::getInstance().stopTime);
        return depTimeAtPrevious + asgn.distToDropoff;
    }

    static INLINE int
    getArrTimeAtTransfer(const int actualDepTimeAtPickup, const AssignmentWithTransfer &asgn,
                         const int initialPickupDetour,
                         const bool transferAtExistingStop, const RouteState &routeState) {
        const auto vehIdPVeh = asgn.pVeh->vehicleId;
        const auto pickupIndex = asgn.pickupIdx;
        const auto transferIndex = asgn.transferIdxPVeh;
        const auto &minDepTimes = routeState.schedDepTimesFor(vehIdPVeh);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehIdPVeh);
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(vehIdPVeh);

        KASSERT(pickupIndex < routeState.numStopsOf(vehIdPVeh) && transferIndex < routeState.numStopsOf(vehIdPVeh));

        if (pickupIndex == transferIndex) {
            return actualDepTimeAtPickup + asgn.distToTransferPVeh;
        }

        KASSERT(pickupIndex >= 0 && transferIndex > 0);
        const auto stopLengthsBetween = vehWaitTimesPrefixSum[transferIndex - 1] - vehWaitTimesPrefixSum[pickupIndex];
        const auto arrTimeAtPrevious =
                minArrTimes[transferIndex] + std::max(initialPickupDetour - stopLengthsBetween, 0);

        if (transferAtExistingStop) {
            return arrTimeAtPrevious;
        }

        const auto depTimeAtPrevious = std::max(minDepTimes[transferIndex],
                                                arrTimeAtPrevious + InputConfig::getInstance().stopTime);
        return depTimeAtPrevious + asgn.distToTransferPVeh;
    }

    // Returns the accumulated vehicle wait time in the stop interval (fromIndex, toIndex].
    static INLINE int
    getTotalVehWaitTimeInInterval(const int vehId, const int fromIndex, const int toIndex,
                                  const RouteState &routeState) {
        if (fromIndex >= toIndex) return 0;
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(vehId);
        return vehWaitTimesPrefixSum[toIndex] - vehWaitTimesPrefixSum[fromIndex];
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const int vehId, const int pickupIndex, const int dropoffIndex, const int depTimeAtPickup,
                            const int distFromPickup, const RequestContext &context,
                            const RouteState &routeState) {
        const auto vehDepTimeAtPrevStop = getVehDepTimeAtStopForRequest(vehId, pickupIndex, context, routeState);
        const auto timeUntilDep = depTimeAtPickup - vehDepTimeAtPrevStop;
        KASSERT(pickupIndex != dropoffIndex || timeUntilDep >= 0);

        if (pickupIndex == dropoffIndex)
            return timeUntilDep;

        const int leg = calcLengthOfLegStartingAt(pickupIndex, vehId, routeState);
        const int sum = timeUntilDep + distFromPickup - leg;

        return sum;
    }

    // Returns the additional time that is needed for the vehicle asgn.vehicle to drive from its stop at index
    // asgn.pickupStopIdx to the given pickup, perform the pickup and drive to its next scheduled stop at index
    // asgn.pickupStopIdx + 1 instead of driving from the stop at asgn.pickupStopIdx directly to the stop
    // at asgn.pickupStopIdx + 1.
    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const Assignment &asgn, const int depTimeAtPickup, const RequestContext &context,
                            const RouteState &routeState) {
        return calcInitialPickupDetour(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                       depTimeAtPickup, asgn.distFromPickup,
                                       context, routeState);
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const AssignmentWithTransfer &asgn, const int depTimeAtPickup,
                            const RequestContext &context,
                            const RouteState &routeState) {
        return calcInitialPickupDetour(asgn.pVeh->vehicleId, asgn.pickupIdx, asgn.transferIdxPVeh,
                                       depTimeAtPickup, asgn.distFromPickup,
                                       context, routeState);
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialTransferDetourDVeh(const AssignmentWithTransfer &asgn, int depTimeAtTransfer,
                                  const RequestContext &context, const RouteState &routeState) {
        const auto vehDepTimeAtPrevStop = getVehDepTimeAtStopForRequest(asgn.dVeh->vehicleId, asgn.transferIdxDVeh,
                                                                        context, routeState);
        const auto timeUntilDep = depTimeAtTransfer - vehDepTimeAtPrevStop;
        KASSERT(asgn.transferIdxDVeh != asgn.dropoffIdx || timeUntilDep >= 0);

        if ((asgn.transferIdxDVeh == asgn.dropoffIdx))
            return timeUntilDep;

        const auto legLength = calcLengthOfLegStartingAt(asgn.transferIdxDVeh, asgn.dVeh->vehicleId, routeState);
        return timeUntilDep + asgn.distFromTransferDVeh - legLength;
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const Assignment &asgn, const RequestContext &context,
                            const RouteState &routeState) {
        const auto actualDepTime = getActualDepTimeAtPickup(asgn, context, routeState);
        return calcInitialPickupDetour(asgn, actualDepTime, context, routeState);
    }

    static INLINE int
    calcOnlyDrivingTimeInInitialPickupDetour(const Assignment &asgn, const bool isPickupAtExistingStop,
                                             const RouteState &routeState) {
        if (isPickupAtExistingStop) return 0;
        if (asgn.pickupStopIdx == asgn.dropoffStopIdx)
            return asgn.distToPickup;
        const auto lengthOfReplacedLeg = calcLengthOfLegStartingAt(asgn.pickupStopIdx, asgn.vehicle->vehicleId,
                                                                   routeState);
        return asgn.distToPickup + asgn.distFromPickup - lengthOfReplacedLeg;
    }

    // Returns the additional time needed for the vehicle to perform a pickup with initialPickupDetour after its stop
    // at pickupIndex and drive until stop toIndex instead of going to stop toIndex according to its current schedule.
    // The residual pickup detour can be smaller than the initial detour since the vehicle may currently wait for
    // passengers at stops between (pickupIndex, toIndex) which is time that it can now spend driving instead which
    // reduces the additional operation time incurred by the new pickup compared to the initial detour for the pickup.
    // Does not subtract the vehicle wait at toIndex itself if one exists, i.e. this is the residual pickup detour that
    // arrives at toIndex.
    static INLINE int
    calcResidualPickupDetour(const int vehId, const int pickupIndex, const int toIndex, const int initialPickupDetour,
                             const RouteState &routeState) {
        const auto vehWaitTime = getTotalVehWaitTimeInInterval(vehId, pickupIndex, toIndex - 1, routeState);
        return std::max(initialPickupDetour - vehWaitTime, 0);
    }

    // Returns the additional time that is needed for the vehicle asgn.vehicle to drive from its stop at index
    // asgn.dropoffStopIdx to the given dropoff, perform the dropoff and drive to its next scheduled stop at index
    // asgn.dropoffStopIdx + 1 instead of driving from the stop at asgn.dropoffStopIdx directly to the stop
    // at asgn.dropoffStopIdx + 1.
    static INLINE int
    calcInitialDropoffDetour(const int vehId, const int dropoffIndex, const int distToDropoff,
                             const int distFromDropoff,
                             const bool dropoffAtExistingStop,
                             const RouteState &routeState) {
        if (dropoffAtExistingStop) return 0;
        const auto lengthOfReplacedLeg = calcLengthOfLegStartingAt(dropoffIndex, vehId, routeState);
        return distToDropoff + InputConfig::getInstance().stopTime + distFromDropoff - lengthOfReplacedLeg;
    }

    // Returns the additional time that is needed for the vehicle asgn.vehicle to drive from its stop at index
    // asgn.dropoffStopIdx to the given dropoff, perform the dropoff and drive to its next scheduled stop at index
    // asgn.dropoffStopIdx + 1 instead of driving from the stop at asgn.dropoffStopIdx directly to the stop
    // at asgn.dropoffStopIdx + 1.
    static INLINE int
    calcInitialDropoffDetour(const Assignment &asgn, const bool dropoffAtExistingStop,
                             const RouteState &routeState) {
        return calcInitialDropoffDetour(asgn.vehicle->vehicleId, asgn.dropoffStopIdx, asgn.distToDropoff,
                                        asgn.distFromDropoff, dropoffAtExistingStop, routeState);
    }

    static INLINE int
    calcInitialDropoffDetour(const AssignmentWithTransfer &asgn, const bool dropoffAtExistingStop,
                             const RouteState &routeState) {
        return calcInitialDropoffDetour(asgn.dVeh->vehicleId, asgn.dropoffIdx, asgn.distToDropoff, asgn.distFromDropoff,
                                        dropoffAtExistingStop, routeState);
    }

    static INLINE int
    calcInitialTransferDetourPVeh(const AssignmentWithTransfer &asgn, const bool transferAtExistingStop,
                                  const RouteState &routeState) {
        if (transferAtExistingStop) return 0;

        const auto lengthOfReplacedLeg = calcLengthOfLegStartingAt(asgn.transferIdxPVeh, asgn.pVeh->vehicleId,
                                                                   routeState);

        return asgn.distToTransferPVeh + InputConfig::getInstance().stopTime + asgn.distFromTransferPVeh -
               lengthOfReplacedLeg;
    }

    static INLINE int
    calcDetourRightAfterDropoff(const int vehId, const int pickupIndex, const int dropoffIndex,
                                const int initialPickupDetour, const int initialDropoffDetour,
                                const RouteState &routeState) {
        const auto detour =
                calcResidualPickupDetour(vehId, pickupIndex, dropoffIndex + 1, initialPickupDetour, routeState) +
                initialDropoffDetour;
        KASSERT(detour >= 0 || pickupIndex == dropoffIndex);
        return std::max(detour, 0);
    }

    static INLINE int
    calcDetourRightAfterDropoff(const Assignment &asgn, const int initialPickupDetour, const int initialDropoffDetour,
                                const RouteState &routeState) {
        return calcDetourRightAfterDropoff(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                           initialPickupDetour, initialDropoffDetour, routeState);
    }

    static INLINE int
    calcDetourRightAfterDropoff(const AssignmentWithTransfer &asgn, const int initialTransferDetour,
                                const int initalDropoffDetour, const RouteState &routeState) {
        return calcDetourRightAfterDropoff(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, asgn.dropoffIdx,
                                           initialTransferDetour, initalDropoffDetour, routeState);
    }

    static INLINE int
    calcDetourRightAfterTransferPVeh(const AssignmentWithTransfer &asgn, const int initialPickupDetour,
                                     const int initialTransferDetour, const RouteState &routeState) {
        const auto detour = calcResidualPickupDetour(asgn.pVeh->vehicleId, asgn.pickupIdx, asgn.transferIdxPVeh,
                                                     initialPickupDetour, routeState) + initialTransferDetour;

        KASSERT(detour >= 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
        return std::max(detour, 0);
    }

    static INLINE int
    calcResidualTotalDetourForStopAfterDropoff(const int vehId, const int dropoffIndex, const int toIndex,
                                               const int detourRightAfterDropoff, const RouteState &routeState) {
        KASSERT(toIndex >= dropoffIndex);
        const auto vehWaitTime = getTotalVehWaitTimeInInterval(vehId, dropoffIndex, toIndex - 1, routeState);
        return std::max(detourRightAfterDropoff - vehWaitTime, 0);
    }

    static INLINE int
    calcResidualTotalDetour(const int vehId, const int pickupIndex, const int dropoffIndex, const int toIndex,
                            const int initialPickupDetour, const int detourRightAfterDropoff,
                            const RouteState &routeState) {
        KASSERT(toIndex >= pickupIndex);
        if (toIndex <= dropoffIndex) {
            return calcResidualPickupDetour(vehId, pickupIndex, toIndex, initialPickupDetour, routeState);
        }

        return calcResidualTotalDetourForStopAfterDropoff(vehId, dropoffIndex, toIndex, detourRightAfterDropoff,
                                                          routeState);
    }

    static INLINE int calcResidualTotalDetour(const Assignment &asgn, const int toIndex, const int initialPickupDetour,
                                              const int detourRightAfterDropoff, const RouteState &routeState) {
        return calcResidualTotalDetour(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                       toIndex, initialPickupDetour, detourRightAfterDropoff, routeState);
    }

    static INLINE int
    calcAddedTripTimeInInterval(const int vehId, const int fromIndex, const int toIndex, const int detourAtFromIndex,
                                const RouteState &routeState) {

        KASSERT(detourAtFromIndex >= 0);

        if (detourAtFromIndex == 0 || fromIndex == toIndex) {
            return 0;
        }
        const auto &vehWaitTimePrefixSums = routeState.vehWaitTimesPrefixSumFor(vehId);
        const auto &vehWaitTimesAtDropoffsPrefixSums = routeState.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId);
        const auto &numDropoffsPrefixSums = routeState.numDropoffsPrefixSumFor(vehId);

        // We consider the interval sum of vehicle wait times between fromIndex and an existing dropoff of a
        // request r as a buffer to the added trip time for r since those vehicle wait times will now be used for
        // driving the detour instead.
        // Let toIndex := initial given value of resDetourUntilIdx.
        // Let d be an existing dropoff between fromIndex and toIndex. If the buffer from fromIndex to d is smaller
        // than the detour at fromIndex, then there is a residual detour that the passenger of d experiences as
        // added trip time. If the buffer between fromIndex and d is large enough to counteract the whole detour, then
        // there is no added trip time.
        // We find the largest index resDetourUntilIdx that still has a residual detour by decrementing
        // starting at toIndex. Then, all dropoffs later than resDetourUntilIdx experience no added trip time and all
        // dropoffs between fromIndex and resDetourUntilIdx do.
        // For the dropoffs before resDetourUntilIdx, we can then simply subtract the sum of the buffers that each
        // dropoff can utilize from the sum of detours (i.e. the given detour * the number of affected dropoffs).
        // This only works if all affected dropoffs experience a positive residual detour since otherwise in the sum a
        // "negative" detour at a later dropoff would be able to reduce the added trip time of an earlier dropoff which
        // we have to prevent.
        //
        // In most cases, we expect a vehicle to only have a small total wait time so resDetourUntilIdx will likely
        // simply be equal to toIndex. Thus, in most cases, we can compute the added trip time in constant time.
        int resDetourUntilIdx = toIndex;
        while (resDetourUntilIdx > fromIndex &&
               getTotalVehWaitTimeInInterval(vehId, fromIndex, resDetourUntilIdx - 1, routeState) >=
               detourAtFromIndex) {
            --resDetourUntilIdx;
        }
        KASSERT(getTotalVehWaitTimeInInterval(vehId, fromIndex, resDetourUntilIdx - 1, routeState) < detourAtFromIndex);

        const auto numDropoffsInInterval = numDropoffsPrefixSums[resDetourUntilIdx] - numDropoffsPrefixSums[fromIndex];
        const auto sumOfBuffersOfDropoffsInInterval =
                vehWaitTimesAtDropoffsPrefixSums[resDetourUntilIdx] - vehWaitTimesAtDropoffsPrefixSums[fromIndex] -
                numDropoffsInInterval * vehWaitTimePrefixSums[fromIndex];
        return numDropoffsInInterval * detourAtFromIndex - sumOfBuffersOfDropoffsInInterval;
    }


    static INLINE int calcAddedTripTimeAffectedByPickupAndDropoff(const int vehId, const int newDropoffIndex,
                                                                  const int detourRightAfterDropoff,
                                                                  const RouteState &routeState) {
        KASSERT(detourRightAfterDropoff >= 0);
        const auto &numStops = routeState.numStopsOf(vehId);
        KASSERT(newDropoffIndex < numStops);
        return calcAddedTripTimeInInterval(vehId, newDropoffIndex, numStops - 1, detourRightAfterDropoff, routeState);
    }

    static INLINE int calcAddedTripTimeAffectedByTransferAndDropoff(const int vehId, const int newDropoffIndex,
                                                                    const int detourRightAfterDropoff,
                                                                    const RouteState &routeState) {
        KASSERT(detourRightAfterDropoff >= 0);
        const auto numStops = routeState.numStopsOf(vehId);
        KASSERT(newDropoffIndex < numStops);

        return calcAddedTripTimeInInterval(vehId, newDropoffIndex, numStops - 1, detourRightAfterDropoff, routeState);
    }

    static INLINE int
    calcAddedTripTimeAffectedByTransferAndDropoff(const AssignmentWithTransfer &asgn, const int detourRightAfterDropoff,
                                                  const RouteState &routeState) {
        return calcAddedTripTimeAffectedByTransferAndDropoff(asgn.dVeh->vehicleId, asgn.dropoffIdx,
                                                             detourRightAfterDropoff, routeState);
    }

    static INLINE int
    calcAddedTripTimeAffectedByPickupAndDropoff(const Assignment &asgn, const int detourRightAfterDropoff,
                                                const RouteState &routeState) {
        return calcAddedTripTimeAffectedByPickupAndDropoff(asgn.vehicle->vehicleId, asgn.dropoffStopIdx,
                                                           detourRightAfterDropoff, routeState);
    }

    static INLINE int
    calcAddedTripTimeAffectedByPickupAndTransfer(const AssignmentWithTransfer &asgn, const int detourRightAfterTransfer,
                                                 const RouteState &routeState) {
        return calcAddedTripTimeAffectedByPickupAndDropoff(asgn.pVeh->vehicleId, asgn.transferIdxPVeh,
                                                           detourRightAfterTransfer, routeState);
    }


    template<typename RequestContext>
    static INLINE bool
    isServiceTimeConstraintViolated(const Vehicle &veh, const RequestContext &context, const int residualDetourAtEnd,
                                    const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);

        return std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
               endServiceTime;
    }

    template<typename RequestContext>
    static INLINE bool
    isOccupancyConstraintViolated(const Vehicle &veh, const int pickupIndex, const int dropoffIndex,
                                  const bool dropoffAtExistingStop, const RequestContext &context,
                                  const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto &occupancies = routeState.occupanciesFor(vehId);

        // If somewhere between pickup and dropoff the vehicle is already full, we cannot insert another passenger.
        for (int i = pickupIndex; i < dropoffIndex; ++i)
            if (occupancies[i] + context.originalRequest.numRiders > veh.capacity)
                return true;
        if (!dropoffAtExistingStop && occupancies[dropoffIndex] + context.originalRequest.numRiders > veh.capacity)
            return true;

        return false;
    }

    static INLINE bool
    doesPickupDetourViolateRiderHardConstraints(const int vehId, const int pickupIndex, const int dropoffIndex,
                                                const int detourRightAfterPickup, const RouteState &routeState) {
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);

        // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (pickupIndex + 1 == numStops)
            return false;

        // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
        // arrival time, this assignment violates some trip time or wait time hard constraint.
        if (pickupIndex != dropoffIndex && detourRightAfterPickup != 0 &&
            minArrTimes[pickupIndex + 1] + detourRightAfterPickup > maxArrTimes[pickupIndex + 1])
            return true;

        return false;
    }

    static INLINE bool
    doesDropoffDetourViolateRiderHardConstraints(const int vehId, const int dropoffIndex,
                                                 const int detourRightAfterDropoff, const RouteState &routeState) {
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);

        // If the dropoff is inserted at/after the last stop, the service time constraint is kept and the pickup does
        // not violate the trip time or wait time constraints, the assignment is ok.
        if (dropoffIndex + 1 == numStops)
            return false;

        // If the total detour moves the planned arrival time at the stop after the dropoff past the latest permissible
        // arrival time, this assignment violates some trip time or wait time constraint.
        if (detourRightAfterDropoff != 0 &&
            minArrTimes[dropoffIndex + 1] + detourRightAfterDropoff > maxArrTimes[dropoffIndex + 1])
            return true;

        return false;
    }

    template<typename RequestContext>
    static INLINE bool isAnyHardConstraintViolated(const Vehicle &veh, const int pickupIndex, const int dropoffIndex,
                                                   const RequestContext &context, const int initialPickupDetour,
                                                   const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                                   const bool dropoffAtExistingStop, const RouteState &routeState) {
        return doesPickupDetourViolateRiderHardConstraints(veh.vehicleId, pickupIndex, dropoffIndex,
                                                           initialPickupDetour, routeState) ||
               doesDropoffDetourViolateRiderHardConstraints(veh.vehicleId, dropoffIndex, detourRightAfterDropoff,
                                                            routeState) ||
               isServiceTimeConstraintViolated(veh, context, residualDetourAtEnd, routeState) ||
               isOccupancyConstraintViolated(veh, pickupIndex, dropoffIndex, dropoffAtExistingStop, context,
                                             routeState);
    }

    template<typename RequestContext>
    static INLINE bool
    isAnyHardConstraintViolated(const AssignmentWithTransfer &asgn, const RequestContext &context,
                                const int initialPickupDetour,
                                const int detourRightAfterTransfer, const int residualDetourAtEnd,
                                const bool transferAtExistingStop, const RouteState &routeState) {
        return isAnyHardConstraintViolated(*asgn.pVeh, asgn.pickupIdx, asgn.transferIdxPVeh, context,
                                           initialPickupDetour, detourRightAfterTransfer, residualDetourAtEnd,
                                           transferAtExistingStop, routeState);
    }

    template<typename RequestContext>
    static INLINE bool
    isAnyHardConstraintViolated(const Assignment &asgn, const RequestContext &context, const int initialPickupDetour,
                                const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                const bool dropoffAtExistingStop, const RouteState &routeState) {
        return isAnyHardConstraintViolated(*asgn.vehicle, asgn.pickupStopIdx, asgn.dropoffStopIdx, context,
                                           initialPickupDetour, detourRightAfterDropoff, residualDetourAtEnd,
                                           dropoffAtExistingStop, routeState);
    }

    template<typename RequestContext>
    static INLINE bool isAnyHardConstraintViolated(const int arrAtTransfer, const Vehicle &dVeh, const int transferIdx,
                                                   const int dropoffIdx,
                                                   const RequestContext &context, const int initialTransferDetour,
                                                   const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                                   const bool dropoffAtExistingStop, const RouteState &routeState) {
        const auto vehId = dVeh.vehicleId;
        const auto endServiceTime = dVeh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);
        const auto &occupancies = routeState.occupanciesFor(vehId);

        KASSERT(dropoffIdx >= 0 && transferIdx >= 0 && "isAnyHardConstraintViolated");

        // If departure time at the last stop (which may be the time of issuing this request if the vehicle is currently
        // idling) is moved past the end of the service time by the total detour, the assignment violates the service
        // time constraint.
        if (std::max(minDepTimes[numStops - 1], arrAtTransfer) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (transferIdx + 1 == numStops)
            return false;

        KASSERT(transferIdx + 1 < numStops && "isAnyHardConstraintViolated");

        // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
        // arrival time, this assignment violates some trip time or wait time hard constraint.
        if (transferIdx != dropoffIdx && initialTransferDetour != 0 &&
            minArrTimes[transferIdx + 1] + initialTransferDetour > maxArrTimes[transferIdx + 1])
            return true;

        // If somewhere between pickup and dropoff the vehicle is already full, we cannot insert another passenger.
        for (int i = transferIdx; i < transferIdx; ++i)
            if (occupancies[i] + context.originalRequest.numRiders > dVeh.capacity)
                return true;
        if (!dropoffAtExistingStop && occupancies[transferIdx] + context.originalRequest.numRiders > dVeh.capacity)
            return true;

        // If the dropoff is inserted at/after the last stop, the service time constraint is kept and the pickup does
        // not violate the trip time or wait time constraints, the assignment is ok.
        if (dropoffIdx + 1 == numStops)
            return false;

        KASSERT(dropoffIdx + 1 < numStops && "isAnyHardConstraintViolated");

        // If the total detour moves the planned arrival time at the stop after the dropoff past the latest permissible
        // arrival time, this assignment violates some trip time or wait time constraint.
        if (detourRightAfterDropoff != 0 &&
            minArrTimes[dropoffIdx + 1] + detourRightAfterDropoff > maxArrTimes[dropoffIdx + 1])
            return true;

        return false;
    }

//    template<typename RequestContext>
//    static INLINE bool
//    isAnyHardConstraintViolatedPVeh(const AssignmentWithTransfer &asgn, const RequestContext &context,
//                                    const int initalPickupDetour, const int detourRightAfterTransfer,
//                                    const int residualDetourAtEndPVeh,
//                                    const bool transferAtExistingStop,
//                                    const RouteState &routeState) {
//
//        return isAnyHardConstraintViolated(*asgn.pVeh, asgn.pickupIdx, asgn.transferIdxPVeh, context,
//                                           initalPickupDetour, detourRightAfterTransfer, residualDetourAtEndPVeh,
//                                           transferAtExistingStop, routeState);
//    }
//
//    template<typename RequestContext>
//    static INLINE bool
//    isAnyHardConstraintViolatedDVeh(const AssignmentWithTransfer &asgn, const RequestContext &context,
//                                    const int initalTransferDetour,
//                                    const int detourRightAfterDropoff, const int residualDetourAtEnd,
//                                    const bool dropoffAtExistingStop, const RouteState &routeState) {
//
//        return isAnyHardConstraintViolated(asgn.arrAtTransferPoint, *asgn.dVeh, asgn.transferIdxDVeh, asgn.dropoffIdx,
//                                           context,
//                                           initalTransferDetour, detourRightAfterDropoff, residualDetourAtEnd,
//                                           dropoffAtExistingStop, routeState);
//    }

    template<typename RequestContext>
    static INLINE bool
    doesPickupDetourViolateHardConstraints(const Vehicle &veh, const RequestContext &context, const int pickupIndex,
                                           const int initialPickupDetour, const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);

        // If departure time at the last stop (which may be the time of issuing this request) is moved past
        // the end of the service time by the pickup detour, the assignment violates the service time constraint.
        const auto residualDetourAtEnd = calcResidualPickupDetour(vehId, pickupIndex, numStops - 1, initialPickupDetour,
                                                                  routeState);
        if (std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (pickupIndex + 1 == numStops)
            return false;

        // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
        // arrival time, this assignment violates some trip time or wait time hard constraint.
        if (initialPickupDetour != 0 &&
            minArrTimes[pickupIndex + 1] + initialPickupDetour > maxArrTimes[pickupIndex + 1])
            return true;

        return false;
    }

    template<typename RequestContext>
    static INLINE bool
    doesDropoffDetourViolateHardConstraints(const Vehicle &veh, const RequestContext &context, const int dropoffIndex,
                                            const int initialDropoffDetour, const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);

        // If departure time at the last stop (which may be the time of issuing this request) is moved past
        // the end of the service time by the dropoff detour, the assignment violates the service time constraint.
        const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(vehId, dropoffIndex, numStops - 1,
                                                                                    initialDropoffDetour, routeState);
        if (std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the dropoff is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (dropoffIndex + 1 == numStops)
            return false;

        // If the dropoff detour moves the planned arrival time at the stop after the dropoff past the latest
        // permissible arrival time, this assignment violates some trip time or wait time hard constraint.
        if (initialDropoffDetour != 0 &&
            minArrTimes[dropoffIndex + 1] + initialDropoffDetour > maxArrTimes[dropoffIndex + 1])
            return true;

        return false;
    }


    // Facility to compute the detour at every affected stop for an assignment or an assignment with transfer.
    // Respects dependencies between vehicle routes caused by transfers.
    struct DetourComputer {

    public:

        explicit DetourComputer(const RouteState &routeState) :
                newArrTimes(routeState.getMaxStopId() + 1, -1),
                newDepTimes(routeState.getMaxStopId() + 1, -1),
                stopIdsSeen(routeState.getMaxStopId() + 1),
                routeState(routeState) {}


        template<typename RequestContext>
        void computeDetours(const Assignment &asgn, const RequestContext &context) {
            newArrTimes.clear();
            newDepTimes.clear();
            stopIdsSeen.clear();
            if (newArrTimes.size() <= routeState.getMaxStopId()) {
                newArrTimes.resize(routeState.getMaxStopId() + 1);
                newDepTimes.resize(routeState.getMaxStopId() + 1);
                stopIdsSeen.reserve(routeState.getMaxStopId() + 1);
            }

            const auto asgnVehId = asgn.vehicle->vehicleId;
            if (asgn.pickupStopIdx == routeState.numStopsOf(asgnVehId) - 1) {
                // For PALS assignments, no stop experiences any detour since both pickup and dropoff are appended as
                // new stops.
                return;
            }


            static const auto &stopTime = InputConfig::getInstance().stopTime;

            // Starting with vehicle of assignment, iterate through the stops of every affected vehicle and compute the
            // arrival times and departure times that would result from inserting the assignment.
            // Whenever a transfer and forward dependency to a different route is encountered, append the stop with
            // the new departure time at the transfer to the queue of routes to process.
            const int arrTimeRightAfterPickup =
                    std::max(routeState.schedArrTimesFor(asgnVehId)[asgn.pickupStopIdx + 1],
                             getActualDepTimeAtPickup(asgn, context, routeState) +
                             (asgn.pickupStopIdx < asgn.dropoffStopIdx ? asgn.distFromPickup :
                              asgn.distToDropoff + stopTime + asgn.distFromDropoff));
            const int depTimeRightAfterPickup = std::max(routeState.schedDepTimesFor(asgnVehId)[asgn.pickupStopIdx + 1],
                                                         arrTimeRightAfterPickup + stopTime);
            const auto afterPickupStopId = routeState.stopIdsFor(asgnVehId)[asgn.pickupStopIdx + 1];
            newArrTimes[afterPickupStopId] = arrTimeRightAfterPickup;
            newDepTimes[afterPickupStopId] = depTimeRightAfterPickup;

            const auto afterDropoffStopId = (asgn.dropoffStopIdx == routeState.numStopsOf(asgnVehId) - 1) ? INVALID_ID :
                                            routeState.stopIdsFor(asgnVehId)[asgn.dropoffStopIdx + 1];
            const auto dropoffAtStop = isDropoffAtExistingStop(asgn, routeState);
            const auto distViaDropoff = asgn.distFromDropoff + (dropoffAtStop ? 0 : stopTime + asgn.distToDropoff);

            propagate(afterPickupStopId, afterDropoffStopId, distViaDropoff);
        }

        template<bool UnknownDropoff = false, typename RequestContext>
        void computeDetours(const AssignmentWithTransfer &asgn, const RequestContext &context) {
            newArrTimes.clear();
            newDepTimes.clear();
            stopIdsSeen.clear();
            if (newArrTimes.size() <= routeState.getMaxStopId()) {
                newArrTimes.resize(routeState.getMaxStopId() + 1);
                newDepTimes.resize(routeState.getMaxStopId() + 1);
                stopIdsSeen.reserve(routeState.getMaxStopId() + 1);
            }

            const auto pVehId = asgn.pVeh->vehicleId;
            const auto dVehId = asgn.dVeh->vehicleId;


            static const auto &stopTime = InputConfig::getInstance().stopTime;


            if (asgn.pickupIdx < routeState.numStopsOf(pVehId) - 1) {
                const int arrTimeRightAfterPickup =
                        std::max(routeState.schedArrTimesFor(pVehId)[asgn.pickupIdx + 1],
                                 getActualDepTimeAtPickup(asgn, context, routeState) +
                                 (asgn.pickupIdx < asgn.transferIdxPVeh ? asgn.distFromPickup :
                                  asgn.distToTransferPVeh + stopTime + asgn.distFromTransferPVeh));
                const int depTimeRightAfterPickup = std::max(routeState.schedDepTimesFor(pVehId)[asgn.pickupIdx + 1],
                                                             arrTimeRightAfterPickup + stopTime);
                const auto afterPickupStopId = routeState.stopIdsFor(pVehId)[asgn.pickupIdx + 1];
                newArrTimes[afterPickupStopId] = arrTimeRightAfterPickup;
                newDepTimes[afterPickupStopId] = depTimeRightAfterPickup;

                const auto afterTransferPVehStopId = (asgn.transferIdxPVeh == routeState.numStopsOf(pVehId) - 1)
                                                     ? INVALID_ID :
                                                     routeState.stopIdsFor(pVehId)[asgn.transferIdxPVeh + 1];
                const auto transferPVehAtStop = isTransferAtExistingStopPVeh(asgn, routeState);
                const auto distViaTransferPVeh =
                        asgn.distFromTransferPVeh + (transferPVehAtStop ? 0 : stopTime + asgn.distToTransferPVeh);

                propagate(afterPickupStopId, afterTransferPVehStopId, distViaTransferPVeh);
            }

            if (asgn.transferIdxDVeh < routeState.numStopsOf(dVehId) - 1) {
                const bool transferDVehAtStop = isTransferAtExistingStopDVeh(asgn, context.originalRequest.requestTime,
                                                                             routeState);
                const int stopIdBeforeTransferDVeh = routeState.stopIdsFor(dVehId)[asgn.transferIdxDVeh];
                const int depTimeBeforeTransferDVeh = std::max(
                        routeState.schedDepTimesFor(dVehId)[asgn.transferIdxDVeh],
                        newDepTimes[stopIdBeforeTransferDVeh]);
                const auto minVehicleDepTimeAtTransferDVeh =
                        depTimeBeforeTransferDVeh + !transferDVehAtStop * (asgn.distToTransferDVeh + stopTime);
                const bool transferPVehAtStop = isTransferAtExistingStopPVeh(asgn, routeState);
                const int stopIdBeforeTransferPVeh = routeState.stopIdsFor(pVehId)[asgn.transferIdxPVeh];

                int minRiderArrTimeAtTransferPVeh;
                if (transferPVehAtStop) {
                    minRiderArrTimeAtTransferPVeh = std::max(routeState.schedArrTimesFor(pVehId)[asgn.transferIdxPVeh],
                                                             newArrTimes[stopIdBeforeTransferPVeh]);
                } else {
                    const auto depTimeBeforeTransferPVeh = (asgn.pickupIdx == asgn.transferIdxPVeh) ?
                                                           getActualDepTimeAtPickup(asgn, context, routeState) :
                                                           std::max(routeState.schedDepTimesFor(
                                                                            pVehId)[asgn.transferIdxPVeh],
                                                                    newDepTimes[stopIdBeforeTransferPVeh]);
                    minRiderArrTimeAtTransferPVeh = depTimeBeforeTransferPVeh + asgn.distToTransferPVeh;
                }

                const auto depTimeAtTransfer = std::max(minVehicleDepTimeAtTransferDVeh,
                                                        minRiderArrTimeAtTransferPVeh);
                const int arrTimeRightAfterTransferDVeh = std::max(
                        routeState.schedArrTimesFor(dVehId)[asgn.transferIdxDVeh + 1],
                        depTimeAtTransfer +
                        (UnknownDropoff || asgn.transferIdxDVeh < asgn.dropoffIdx ? asgn.distFromTransferDVeh : asgn.distToDropoff +
                                                                                              stopTime +
                                                                                              asgn.distFromDropoff));
                const int depTimeRightAfterTransferDVeh = std::max(
                        routeState.schedDepTimesFor(dVehId)[asgn.transferIdxDVeh] + 1,
                        arrTimeRightAfterTransferDVeh + stopTime);

                const auto afterTransferDVehStopId = routeState.stopIdsFor(dVehId)[asgn.transferIdxDVeh + 1];
                newArrTimes[afterTransferDVehStopId] = arrTimeRightAfterTransferDVeh;
                newDepTimes[afterTransferDVehStopId] = depTimeRightAfterTransferDVeh;

                int afterDropoffStopId = INVALID_ID;
                int distViaDropoff = INFTY;
                if (!UnknownDropoff && asgn.dropoffIdx < routeState.numStopsOf(dVehId) - 1) {
                    afterDropoffStopId = routeState.stopIdsFor(dVehId)[asgn.dropoffIdx + 1];
                    const auto dropoffAtStop = isDropoffAtExistingStop(asgn, routeState);
                    distViaDropoff = asgn.distFromDropoff + (dropoffAtStop ? 0 : stopTime + asgn.distToDropoff);
                }

                propagate(afterTransferDVehStopId, afterDropoffStopId, distViaDropoff);
            }

        }


        TimestampedVector<int> newArrTimes;
        TimestampedVector<int> newDepTimes;

        Subset stopIdsSeen;

    private:

        void propagate(const int afterPickupStopId, const int afterDropoffStopId, const int distViaDropoff) {
            static const auto &stopTime = InputConfig::getInstance().stopTime;

            std::vector<int> firstStopIdsInRoutesToProcess;
            firstStopIdsInRoutesToProcess.push_back(afterPickupStopId);
            while (!firstStopIdsInRoutesToProcess.empty()) {

                auto stopId = firstStopIdsInRoutesToProcess.back();
                firstStopIdsInRoutesToProcess.pop_back();
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto initialStopIdx = routeState.stopPositionOf(stopId);
                const auto numStops = routeState.numStopsOf(vehId);
                const auto stopIds = routeState.stopIdsFor(vehId);
                const auto schedDepTimes = routeState.schedDepTimesFor(vehId);
                const auto schedArrTimes = routeState.schedArrTimesFor(vehId);

                for (int i = initialStopIdx; i < numStops; ++i) {
                    // Invariant: newDepTimes is already correct for all stops up to and including i. newArrTimes is
                    // either correct or equal to -1, the initial value.
                    stopId = stopIds[i];
                    KASSERT(newDepTimes[stopId] != -1);
                    stopIdsSeen.insert(stopId);

                    const auto arrTimeAtStop = newArrTimes[stopId];
                    for (const auto &dependentStopId: routeState.getForwardDependencies(stopId)) {

                        // The dropoff vehicle (dependent vehicle) can only leave the transfer stop after the rider
                        // that transfers has reached the stop in the pickup vehicle. Set new earliest departure time
                        // at transfer stop in dropoff vehicle and mark vehicle to propagate if necessary.

                        // If the dependent stop has already been processed with an equal or later departure time,
                        // skip it. This avoids loops in the transfer dependency graph where one or more riders switch
                        // in both directions at a transfer.
                        if (newDepTimes[dependentStopId] >= arrTimeAtStop)
                            continue;

                        newDepTimes[dependentStopId] = arrTimeAtStop;
                        firstStopIdsInRoutesToProcess.push_back(dependentStopId);
                    }

                    const auto depTimeAtStop = newDepTimes[stopId];
                    if (depTimeAtStop <= schedDepTimes[i]) {
                        // If the departure time at i does not change, the schedule remains the same from i onwards.
                        break;
                    }

                    if (i == numStops - 1)
                        break;

                    const auto nextStopId = stopIds[i + 1];
                    int nextArrTime = depTimeAtStop;
                    if (nextStopId == afterDropoffStopId) {
                        nextArrTime += distViaDropoff;
                    } else {
                        const auto legLength = schedDepTimes[i + 1] - schedArrTimes[i];
                        nextArrTime += legLength;
                    }
                    newArrTimes[nextStopId] = std::max(newArrTimes[nextStopId], nextArrTime);
                    const int nextDepTime = std::max(nextArrTime + stopTime, schedDepTimes[i + 1]);
                    newDepTimes[nextStopId] = std::max(newDepTimes[nextStopId], nextDepTime);
                }
            }
        }

        const RouteState &routeState;

    };

    int computeRiderArrTimeAtTransfer(const AssignmentWithTransfer &asgn, const int depTimeAtPickup,
                                      const bool transferPVehAtExistingStop,
                                      DetourComputer &detourComputer,
                                      const RouteState &routeState) {
        const auto pVehId = asgn.pVeh->vehicleId;
        const int stopIdBeforeTransferPVeh = routeState.stopIdsFor(pVehId)[asgn.transferIdxPVeh];

        if (transferPVehAtExistingStop) {
            return std::max(routeState.schedArrTimesFor(pVehId)[asgn.transferIdxPVeh],
                            detourComputer.newArrTimes[stopIdBeforeTransferPVeh]);
        }

        const int depTimeBeforeTransferPVeh = (asgn.pickupIdx == asgn.transferIdxPVeh) ?
                                              depTimeAtPickup :
                                              std::max(routeState.schedDepTimesFor(pVehId)[asgn.transferIdxPVeh],
                                                       detourComputer.newDepTimes[stopIdBeforeTransferPVeh]);
        return depTimeBeforeTransferPVeh + asgn.distToTransferPVeh;
    }

    template<typename RequestContext>
    int computeDVehDepTimeAtTransfer(const AssignmentWithTransfer &asgn,
                                     const int riderArrTimeAtTransfer,
                                     DetourComputer &detourComputer,
                                     const RouteState &routeState,
                                     const RequestContext &context) {
        const auto dVehId = asgn.dVeh->vehicleId;
        const int stopIdBeforeTransferDVeh = routeState.stopIdsFor(dVehId)[asgn.transferIdxDVeh];
        const bool transferDVehAtExistingStop =
                isTransferAtExistingStopDVeh(asgn, context.originalRequest.requestTime, routeState);
        const int depTimeBeforeTransferDVeh = std::max(routeState.schedDepTimesFor(dVehId)[asgn.transferIdxDVeh],
                                                       detourComputer.newDepTimes[stopIdBeforeTransferDVeh]);
        const int minVehicleDepTimeAtTransferDVeh =
                depTimeBeforeTransferDVeh +
                !transferDVehAtExistingStop * (asgn.distToTransferDVeh + InputConfig::getInstance().stopTime);
        return std::max(minVehicleDepTimeAtTransferDVeh, riderArrTimeAtTransfer);
    }

    int computeArrTimeAtDropoffAfterTransfer(const AssignmentWithTransfer &asgn,
                                             const int depTimeAtTransfer,
                                             DetourComputer &detourComputer,
                                             const RouteState &routeState) {
        if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
            // If the dropoff is at the same stop as the transfer, we can use the departure time at the transfer.
            return depTimeAtTransfer + asgn.distToDropoff;
        }

        const auto dVehId = asgn.dVeh->vehicleId;
        const int stopIdBeforeDropoff = routeState.stopIdsFor(dVehId)[asgn.dropoffIdx];
        const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
        if (dropoffAtExistingStop) {
            return std::max(routeState.schedArrTimesFor(dVehId)[asgn.dropoffIdx],
                            detourComputer.newArrTimes[stopIdBeforeDropoff]);
        }
        const int depTimeBeforeDropoff = std::max(routeState.schedDepTimesFor(dVehId)[asgn.dropoffIdx],
                                                  detourComputer.newDepTimes[stopIdBeforeDropoff]);
        return depTimeBeforeDropoff + asgn.distToDropoff;

    }

} // end namespace