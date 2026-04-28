#pragma once

#include <kassert/kassert.hpp>

#include "TransportMode.h"
#include "ModeChoiceInputStats.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri::mode_choice {
    // Decides whether a rider accepts an assignment based on the request and the assignment finder response.
    // Mode choice decision is based on KaRRi cost. Will only return Ped or Taxi, depending on which has smaller cost.
    template<bool AlwaysVehicle>
    class KaRRiCostCriterion {
    public:
        explicit KaRRiCostCriterion(const RouteState &routeState)
            : routeState(routeState),
              calculator(routeState),
              walkingCost(INFTY),
              taxiCost(INFTY) {
        }

        void init() {
            walkingCost = INFTY;
            taxiCost = INFTY;
            stats.reset();
        }

        void registerPed(const int walkingTimeTenthsOfSeconds, const RequestState &requestState) {
            KASSERT(walkingTimeTenthsOfSeconds >= 0 && walkingTimeTenthsOfSeconds < INFTY);
            if constexpr (!AlwaysVehicle) {
                walkingCost = CostCalculator::calcCostForNotUsingVehicle(walkingTimeTenthsOfSeconds, requestState);
            }
            stats.walkTravelTime = tenthsOfSecondsToMinutes(walkingTimeTenthsOfSeconds);
        }

        void registerCar(const int) {
            // no op
        }

        void registerPublicTransport(const PTJourneyData&) {
            // no op
        }

        void registerTaxi(const Assignment &assignment, const RequestState &requestState) {
            using namespace time_utils;
            KASSERT(assignment.vehicle);

            taxiCost = calculator.calc(assignment, requestState);

            // Write stats object
            const auto depTimeAtPickup = getActualDepTimeAtPickup(assignment, requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(assignment, depTimeAtPickup, requestState,
                                                                     routeState);
            const auto dropoffAtExistingStop = isDropoffAtExistingStop(assignment, routeState);
            const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, assignment, initialPickupDetour,
                                                              dropoffAtExistingStop, routeState);
            const double taxiTravelTimeMinutes = tenthsOfSecondsToMinutes(arrTimeAtDropoff - depTimeAtPickup);
            const double taxiWaitTimeMinutes = tenthsOfSecondsToMinutes(
                depTimeAtPickup - requestState.originalRequest.requestTime - assignment.pickup.walkingDist);
            const double taxiAccEgrTimeMinutes = tenthsOfSecondsToMinutes(
                assignment.pickup.walkingDist + assignment.dropoff.walkingDist);
            stats.taxiTravelTime = taxiTravelTimeMinutes;
            stats.taxiWaitTime = taxiWaitTimeMinutes;
            stats.taxiAccEgrTime = taxiAccEgrTimeMinutes;
        }

        // Executes mode choice with previously registered modes.
        TransportMode apply() const {
            if constexpr (AlwaysVehicle) {
                if (taxiCost == INFTY)
                    return TransportMode::None;
                return TransportMode::Taxi;
            }
            if (walkingCost < taxiCost)
                return TransportMode::Ped;
            if (taxiCost == INFTY)
                return TransportMode::None;
            return TransportMode::Taxi;
        }

        const ModeChoiceInputStats &getStats() const {
            return stats;
        }

    private:
        static constexpr double tenthsOfSecondsToMinutes(const int timeInTenthsOfSeconds) {
            return static_cast<double>(timeInTenthsOfSeconds) / 600.0;
        }

        const RouteState &routeState;
        CostCalculator calculator;

        int walkingCost;
        int taxiCost;

        ModeChoiceInputStats stats;
    };
} // karri
