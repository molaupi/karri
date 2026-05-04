#pragma once
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/BaseObjects/BestAsgnType.h"
#include "Algorithms/KaRRi/RiderModeChoice/TaxiResult.h"

namespace karri::mode_choice {
    class TaxiResultConstructor {
    public:
        TaxiResultConstructor(const RouteState &routeState) : routeState(routeState), detourComputer(routeState) {
        }

        TaxiResult constructTaxiResult(const RequestState &requestState) {
            TaxiResult result;
            using namespace time_utils;
            if (requestState.improvementThroughTransfer()) {
                const auto &bestAsgn = requestState.getBestAssignmentWithTransfer();
                detourComputer.computeDetours(bestAsgn, requestState);
                const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, requestState, routeState);
                const int arrTimeAtTransfer = computeRiderArrTimeAtTransfer(
                    bestAsgn, depTimeAtPickup, isTransferAtExistingStopPVeh(bestAsgn, routeState),
                    detourComputer, routeState);
                const int depTimeAtTransfer = computeDVehDepTimeAtTransfer(
                    bestAsgn, arrTimeAtTransfer, detourComputer, routeState, requestState);
                const int arrTimeAtDropoff = computeArrTimeAtDropoffAfterTransfer(
                    bestAsgn, depTimeAtTransfer, detourComputer, routeState);
                result.inVehicleTime = arrTimeAtTransfer - depTimeAtPickup + arrTimeAtDropoff - depTimeAtTransfer;
                result.waitTime = depTimeAtPickup - requestState.originalRequest.requestTime - bestAsgn.pickup.
                                  walkingDist + depTimeAtTransfer - arrTimeAtTransfer;
                result.walkTime = bestAsgn.pickup.walkingDist + bestAsgn.dropoff.walkingDist;
                result.cost = requestState.getBestCostWithTransfer();
                result.asgnType = TWO_LEGS;
            } else {
                const auto &bestAsgn = requestState.getBestAssignmentWithoutTransfer();
                const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, requestState, routeState);
                const auto initialPickupDetour = calcInitialPickupDetour(
                    bestAsgn, depTimeAtPickup, requestState, routeState);
                const auto dropoffAtExistingStop = isDropoffAtExistingStop(bestAsgn, routeState);
                const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, bestAsgn, initialPickupDetour,
                                                                  dropoffAtExistingStop, routeState);
                result.inVehicleTime = arrTimeAtDropoff - depTimeAtPickup;
                result.waitTime = depTimeAtPickup - requestState.originalRequest.requestTime - bestAsgn.pickup.
                                  walkingDist;
                result.walkTime = bestAsgn.pickup.walkingDist + bestAsgn.dropoff.walkingDist;
                result.cost = requestState.getBestCostWithoutTransfer();
                result.asgnType = ONE_LEG;
            }
            return result;
        }

    private:
        const RouteState &routeState;
        time_utils::DetourComputer detourComputer;
    };
}
