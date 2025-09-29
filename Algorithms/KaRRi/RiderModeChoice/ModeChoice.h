#pragma once

#include "Tools/Logging/NullLogger.h"
#include "Tools/Logging/LogManager.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "TransportMode.h"
#include "ModeChoiceInput.h"

namespace karri::mode_choice {

    template<typename CriterionT, typename LoggerT = NullLogger>
    class ModeChoice {

    public:

        ModeChoice(const RouteState& routeState) : criterion(), routeState(routeState),
                                                   logger(LogManager<LoggerT>::getLogger("modechoice.csv",
                         "request_id,"
                         "walk_travel_time,"
                         "car_travel_time,"
                         "taxi_travel_time,"
                         "taxi_wait_time,"
                         "taxi_accegr_time,"
                         "mode\n")) {}

        template<typename AsgnFinderResponseT>
        TransportMode chooseMode(const Request &req, const AsgnFinderResponseT &resp) const {
            using namespace time_utils;

            ModeChoiceInput input;

            input.walkTravelTime = tenthsOfSecondsToMinutes(resp.odWalkingDist);
            input.privateCarTravelTime = tenthsOfSecondsToMinutes(resp.originalReqDirectDist);

            const auto &bestAsgn = resp.getBestAssignment();
            if (bestAsgn.vehicle) {
                const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, resp, routeState);
                const auto initialPickupDetour = calcInitialPickupDetour(bestAsgn, depTimeAtPickup, resp, routeState);
                const auto dropoffAtExistingStop = isDropoffAtExistingStop(bestAsgn, routeState);
                const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, bestAsgn, initialPickupDetour,
                                                                  dropoffAtExistingStop, routeState);
                input.taxiTravelTime = tenthsOfSecondsToMinutes(arrTimeAtDropoff - depTimeAtPickup);
                input.taxiWaitTime = tenthsOfSecondsToMinutes(
                        depTimeAtPickup - req.requestTime - bestAsgn.pickup.walkingDist);
                input.taxiAccEgrTime = tenthsOfSecondsToMinutes(bestAsgn.pickup.walkingDist + bestAsgn.dropoff.walkingDist);
            }

            const auto choice = criterion.apply(input);

            logger << req.requestId << ","
                   << input.walkTravelTime << ","
                   << input.privateCarTravelTime << ","
                   << input.taxiTravelTime << ","
                   << input.taxiWaitTime << ","
                   << input.taxiAccEgrTime << ","
                   << (choice == TransportMode::Car ? "Car" : choice == TransportMode::Ped ? "Ped" : "Taxi") << "\n";

            return choice;
        }

    private:

        static constexpr double tenthsOfSecondsToMinutes(const int timeInTenthsOfSeconds) {
            return static_cast<double>(timeInTenthsOfSeconds) / 600.0;
        }

        CriterionT criterion;
        const RouteState &routeState;
        LoggerT& logger;
    };

} // namespace karri::mode_choice
